#ifndef PROTOCOL_DETECTOR_H
#define PROTOCOL_DETECTOR_H

#include <atomic>
#include <std_msgs/String.h>
#include "async_can.hpp"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <linux/can.h>
#include "chassis_communication/chassis_message.h"

class ProtocolDetector 
{
public:
  bool IsConnect(std::string can_name);

private:
  std::shared_ptr<AsyncCAN> can_;
  // void ParseCANFrame(can_frame *rx_frame);
}  ;

class ProtocolParser
{
public:
  bool DecodeMessage(const struct can_frame *rx_frame, ChassisMessage *msg) ;
  bool EncodeMessage(const ChassisMessage *msg, struct can_frame *tx_frame) ;
  // uint8_t CalculateChecksum(uint16_t id, uint8_t *data, uint8_t dlc) ;
};

#endif /* PROTOCOL_DETECTOR_H */
