#include <SPI.h>
#include <mcp2515.h>

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

const int SPI_CS_PIN = 5;
MCP2515 mcp2515(SPI_CS_PIN); // Set CS pin

struct Motor {
  uint8_t can_id;
  float p_in;
  float v_in;
  float kp_in;
  float kd_in;
  float t_in;
  float p_out;
  float v_out;
  float t_out;
  int s;
  long prevTime;

  Motor(uint8_t id) : can_id(id), p_in(0.0f), v_in(0.0f), kp_in(30.0f), kd_in(1.0f),
                      t_in(0.0f), p_out(0.0f), v_out(0.0f), t_out(0.0f), 
                      s(1), prevTime(0) {}
};

#define NUM_MOTORS 2
Motor motors[NUM_MOTORS] = { Motor(0x00), Motor(0x01) }; // Example with 2 motors
struct can_frame canMsg;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  
  Serial.println("CAN BUS Shield is ok!");
  delay(2000);
  
  for (int i = 0; i < NUM_MOTORS; i++) {
    EnterMotorMode(motors[i]);
  }
}

void loop() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    float p_step = 6.28 * sin(3.14 * 0.5 * (millis() / 1000.0));
    motors[i].p_in = constrain(p_step / (i + 1), P_MIN, P_MAX);
    pack_cmd(motors[i]);
  }

  // Receive CAN
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    for (int i = 0; i < NUM_MOTORS; i++) {
      if (canMsg.can_id == motors[i].can_id) {
        unpack_reply(motors[i]);
        break;
      }
    }
  }
}

void unpack_reply(Motor& motor) {
  /// CAN reply packet structure ///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and +30 rad/s
  /// 12 bit current, between -40 and 40
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows. For each quantity, bit 0 is LSB
  /// 0: [position [15-8]]
  /// 1: [position [7-0]]
  /// 2: [velocity [11-4]]
  /// 3: [velocity [3-0], current [11-8]]
  /// 4: [current [7-0]]

  byte buf[8];
  for (int i = 0; i < canMsg.can_dlc; i++) {
    buf[i] = canMsg.data[i];
  }

  /// unpack ints from CAN buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1] << 8) | buf[2];
  unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
  unsigned int i_int = ((buf[4] & 0xF) << 8) | buf[5];
  
  /// convert uints to floats ///
  motor.p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  motor.v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  motor.t_out = uint_to_float(i_int, -T_MAX, T_MAX, 12);
}

void EnterMotorMode(Motor& motor) {
  // Enter Motor Mode (enable)
  canMsg.can_id = motor.can_id;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0xFF;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0xFF;
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFC;
  mcp2515.sendMessage(&canMsg);
  Serial.print("Motor ");
  Serial.print(motor.can_id);
  Serial.println(" ENTER");
}

void ExitMotorMode(Motor& motor) {
  canMsg.can_id = motor.can_id;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0xFF;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0xFF;
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFD;
  mcp2515.sendMessage(&canMsg);
}

void Zero(Motor& motor) {
  canMsg.can_id = motor.can_id;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0xFF;
  canMsg.data[1] = 0xFF;
  canMsg.data[2] = 0xFF;
  canMsg.data[3] = 0xFF;
  canMsg.data[4] = 0xFF;
  canMsg.data[5] = 0xFF;
  canMsg.data[6] = 0xFF;
  canMsg.data[7] = 0xFE;
  mcp2515.sendMessage(&canMsg);
}

void pack_cmd(Motor& motor) {
  /// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit velocity command, between -30 and +30 rad/s
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN packet is 8 8-bit words
  /// Formatted as follows. For each quantity, bit 0 is LSB
  /// 0: [position [15-8]]
  /// 1: [position [7-0]]
  /// 2: [velocity [11-4]]
  /// 3: [velocity [3-0], kp [11-8]]
  /// 4: [kp [7-0]]
  /// 5: [kd [11-4]]
  /// 6: [kd [3-0], torque [11-8]]
  /// 7: [torque [7-0]]

  /// limit data to be within bounds ///
  float p_des = constrain(motor.p_in, P_MIN, P_MAX);
  float v_des = constrain(motor.v_in, V_MIN, V_MAX);
  float kp = constrain(motor.kp_in, KP_MIN, KP_MAX);
  float kd = constrain(motor.kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(motor.t_in, T_MIN, T_MAX);

  /// convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///
  canMsg.can_id = motor.can_id;
  canMsg.can_dlc = 8;
  canMsg.data[0] = p_int >> 8;
  canMsg.data[1] = p_int & 0xFF;
  canMsg.data[2] = v_int >> 4;
  canMsg.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  canMsg.data[4] = kp_int & 0xFF;
  canMsg.data[5] = kd_int >> 4;
  canMsg.data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  canMsg.data[7] = t_int & 0xFF;
  mcp2515.sendMessage(&canMsg);
}

unsigned int float_to_uint(float x, float x_min, float x_max, int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  unsigned int pgg = 0;
  if (bits == 12) {
    pgg = (unsigned int)((x - offset) * 4095.0 / span);
  }
  if (bits == 16) {
    pgg = (unsigned int)((x - offset) * 65535.0 / span);
  }
  return pgg;
}

float uint_to_float(unsigned int x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  float pgg = 0;
  if (bits == 12) {
    pgg = ((float)x_int) * span / 4095.0 + offset;
  }
  if (bits == 16) {
    pgg = ((float)x_int) * span / 65535.0 + offset;
  }
  return pgg;
}
