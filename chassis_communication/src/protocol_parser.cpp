#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <linux/can.h>

#include "chassis_communication/chassis_message.h"
#include "chassis_communication/protocol_process.h"

  // 接收CAN数据，解包
  bool ProtocolParser::DecodeMessage(const struct can_frame *rx_frame, ChassisMessage *msg) 
  {
    bool ret = true;
    msg->type = ChassisMsgUnkonwn;

    // 协议的帧ID对应解包
    switch (rx_frame->can_id) {

      case ((uint32_t)0x211): { //0x211
        msg->type = ChassisMsgSystemState;
        SystemStateFrame *frame = (SystemStateFrame *)(rx_frame->data);
        msg->body.system_state_msg.uwb_enable = frame->uwb_enable;
        msg->body.system_state_msg.res2 = frame->res2;
        msg->body.system_state_msg.res3 = frame->res3;
        msg->body.system_state_msg.res4 = frame->res4;
        msg->body.system_state_msg.res5 = frame->res5;
        msg->body.system_state_msg.res6 = frame->res6;
        msg->body.system_state_msg.res7 = frame->res7;
        msg->body.system_state_msg.res8 = frame->res8;
        break;
      }
      case ((uint32_t)0x221):  //0x221
      {
        // msg->type = ChassisMsgMotionState;
        // MotionStateFrame *frame = (MotionStateFrame *)(rx_frame->data);
        // msg->body.motion_state_msg.linear_velocity =
        //     (int16_t)((uint16_t)(frame->linear_velocity.low_byte) |
        //               (uint16_t)(frame->linear_velocity.high_byte) << 8) /
        //     1000.0;
        // msg->body.motion_state_msg.angular_velocity =
        //     (int16_t)((uint16_t)(frame->angular_velocity.low_byte) |
        //               (uint16_t)(frame->angular_velocity.high_byte) << 8) /
        //     1000.0;
        // msg->body.motion_state_msg.lateral_velocity =
        //     (int16_t)((uint16_t)(frame->lateral_velocity.low_byte) |
        //               (uint16_t)(frame->lateral_velocity.high_byte) << 8) /
        //     1000.0;
        // msg->body.motion_state_msg.steering_angle =
        //     (int16_t)((uint16_t)(frame->steering_angle.low_byte) |
        //               (uint16_t)(frame->steering_angle.high_byte) << 8) /
        //     1000.0;
        break;
      }

      default:
      {
        ret = false;
        break;
      }
    }
    return ret;   
  }

  bool ProtocolParser::EncodeMessage(const ChassisMessage *msg, struct can_frame *tx_frame) 
  {
    bool ret = true;
    switch (msg->type) 
    {
        case ChassisMsgControlModeConfig: 
        {
        tx_frame->can_id = ((uint32_t)0x41); //0x421
        tx_frame->can_dlc = 1;
        ControlModeConfigFrame frame;
        frame.can_enable_flag = msg->body.control_mode_config_msg.mode;

        memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
        break;
      }

      case ChassisMsgMotionCommand: 
      {
        tx_frame->can_id = ((uint32_t)0x111); //0x111
        tx_frame->can_dlc = 8;
        MotionCommandFrame frame;
        int16_t linear_cmd = (int16_t)(msg->body.motion_command_msg.linear_velocity * 1000);
        int16_t angular_cmd = (int16_t)(msg->body.motion_command_msg.angular_velocity * 1000);

        frame.linear_velocity.high_byte = (uint8_t)(linear_cmd >> 8);
        frame.linear_velocity.low_byte = (uint8_t)(linear_cmd & 0x00ff);
        frame.angular_velocity.high_byte = (uint8_t)(angular_cmd >> 8);
        frame.angular_velocity.low_byte = (uint8_t)(angular_cmd & 0x00ff);
        frame.res5 = 0;
        frame.res6 = 0;
        frame.res7 = 0;
        frame.brake_flag = msg->body.motion_command_msg.brake_flag;

        memcpy(tx_frame->data, (uint8_t *)(&frame), tx_frame->can_dlc);
        break;
      }

      default: {
        ret = false;
        break;
      }
    }
    return ret;
  }

  // uint8_t ProtocolParser::CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) 
  // {
  //   uint8_t checksum = 0x00;
  //   checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
  //   for (int i = 0; i < (dlc - 1); ++i) checksum += data[i];
  //   return checksum;
  // }
