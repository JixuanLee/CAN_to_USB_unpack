#ifndef AGILEX_MESSAGE_H
#define AGILEX_MESSAGE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <std_msgs/Header.h>
#include <chrono>

typedef struct {
  uint8_t high_byte;
  uint8_t low_byte;
} struct16_t;

/***************** 控制下发 messages *****************/
typedef enum {
  VEHICLE_STATE_NORMAL = 0x00,
  VEHICLE_STATE_ESTOP = 0x01,
  VEHICLE_STATE_EXCEPTION = 0x02
} ChassisVehicleState;

typedef enum {
  CONTROL_MODE_STANDBY = 0x00,
  CONTROL_MODE_CAN = 0x01
} ChassisControlMode;

typedef struct {
  float linear_velocity;
  float angular_velocity;
  bool brake_flag;
} MotionCommandMessage;

// typedef struct {
//   bool enable_braking;
// } BrakingCommandMessage;



/**************** 反馈上报 messages *****************/

typedef struct {
  // ChassisVehicleState vehicle_state;
  // ChassisControlMode control_mode;
  // float battery_voltage;
  // uint16_t error_code;
  uint8_t uwb_enable;
  uint8_t res2;
  uint8_t res3;
  uint8_t res4;
  uint8_t res5;
  uint8_t res6;
  uint8_t res7;
  uint8_t res8;
} SystemStateMessage;

typedef struct {
  uint8_t uwb_enable;
  uint8_t res2;
  uint8_t res3;
  uint8_t res4;
  uint8_t res5;
  uint8_t res6;
  uint8_t res7;
  uint8_t res8;
  } SystemStateFrame;

typedef struct {
  float linear_velocity;
  float angular_velocity;  // only valid for differential drivering
  float lateral_velocity;
  float steering_angle;  // only valid for ackermann steering
} MotionStateMessage;

typedef struct {
  struct16_t linear_velocity;
  struct16_t angular_velocity;
  uint8_t res5;
  uint8_t res6;
  uint8_t res7;
  uint8_t brake_flag;
} MotionCommandFrame;



typedef struct {
  ChassisControlMode mode;
} ControlModeConfigMessage;

typedef struct {
  uint8_t can_enable_flag;
} ControlModeConfigFrame;

typedef enum {
  ChassisMsgUnkonwn = 0x00,

  ChassisMsgControlModeConfig, // CAN enable

  ChassisMsgMotionCommand, // Motion to vel

  ChassisMsgSystemState, // some enable from vel
  ChassisMsgMotionState
} MsgType;

typedef struct {
  MsgType type;

  union {
    MotionCommandMessage motion_command_msg;
    SystemStateMessage system_state_msg;
    MotionStateMessage motion_state_msg;
    ControlModeConfigMessage control_mode_config_msg;
  } body;

} ChassisMessage;

struct CoreStateMsgGroup {
  std::chrono::time_point<std::chrono::steady_clock> time_stamp;

  SystemStateMessage system_state;
  MotionStateMessage motion_state;
};

// typedef struct {
//   std_msgs::Header header;
  
//   double linear_velocity;
//   double angular_velocity;

//   uint8_t base_state;
//   uint8_t control_mode;
//   uint16_t fault_code;
//   double battery_voltage;
// } status_msg_to_ros;

#endif /* AGILEX_MESSAGE_H */
