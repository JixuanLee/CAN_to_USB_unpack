#include "chassis_communication/can_port.h"
#include "chassis_communication/protocol_process.h"

  ProtocolParser protocolparser;

  ChassisCanPort::~ChassisCanPort()
  {
    if (can_ != nullptr && can_->IsOpened()) 
      can_->Close();
  }

  bool ChassisCanPort::Connect(std::string can_name) 
  {
    can_ = std::make_shared<AsyncCAN>(can_name); // 设置底层接口对象
    can_->SetReceiveCallback(std::bind(&ChassisCanPort::ParseCANFrame, this, std::placeholders::_1)); // 设置底层接口回调函数
    return can_->Open(); // 测试CAN连接状态
  }

  void ChassisCanPort::EnableCommandedMode() 
  {
    ChassisMessage msg;
    msg.type = ChassisMsgControlModeConfig;
    msg.body.control_mode_config_msg.mode = CONTROL_MODE_CAN;

    if (can_ != nullptr && can_->IsOpened()) 
    {
      can_frame frame;
      if (protocolparser.EncodeMessage(&msg, &frame)) 
        can_->SendFrame(frame);
    }
  }


  //接收信息并分类提取，赋值给core_state_msgs_
  void ChassisCanPort::ParseCANFrame(can_frame *rx_frame) 
  {
    ChassisMessage status_msg;

    if (protocolparser.DecodeMessage(rx_frame, &status_msg)) 
    { 
      ChassisCanPort::GetCoreStateFromCAN(status_msg);
    }
  }

  void ChassisCanPort::GetCoreStateFromCAN(const ChassisMessage &status_msg) 
  {
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    switch (status_msg.type) {
      case ChassisMsgSystemState: 
      {
        core_state_msgs_.time_stamp = std::chrono::steady_clock::now();
        core_state_msgs_.system_state = status_msg.body.system_state_msg;
        break;
      }

      case ChassisMsgMotionState: 
      {
        core_state_msgs_.time_stamp = std::chrono::steady_clock::now();
        core_state_msgs_.motion_state = status_msg.body.motion_state_msg;
        break;
      }
      
      default:
        break;
    }
    
  }

CoreStateMsgGroup ChassisCanPort::GetCoreStateToROS()
{
    std::lock_guard<std::mutex> guard(core_state_mtx_);
    return core_state_msgs_;
}