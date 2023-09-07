
#include "chassis_communication/protocol_process.h"

  //调用 can_ 指向的 AsyncCAN 对象的 SetReceiveCallback 方法，设置一个接收 CAN 帧的回调函数,接收数据为can_对象的CAN帧数据，解包帧头
bool ProtocolDetector::IsConnect(std::string can_name) 
{
  can_ = std::make_shared<AsyncCAN>(can_name);
  // can_->SetReceiveCallback( std::bind(&ProtocolDetector::ParseCANFrame, this, std::placeholders::_1));
  return can_->Open();
}

// 收到CAN帧后的回调，解析帧头，暂时不需要
// void ProtocolDetector::ParseCANFrame(can_frame *rx_frame) 
// {
//   switch (rx_frame->can_id) {
//       // state feedback frame with id 0x151 is unique to V1 protocol
//     case 0x151: {

//       break;
//     }
//       // rc state frame with id 0x241 is unique to V2 protocol
//     case 0x241: {

//       break;
//     }
//     default:
//       break;
//   }
// }
