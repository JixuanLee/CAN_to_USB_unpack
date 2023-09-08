#ifndef CAN_PORT_H
#define CAN_PORT_H
#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <atomic>
#include <iostream>
#include <chrono>
#include "async_can.hpp"
#include "chassis_communication/chassis_message.h"
#include "chassis_communication/protocol_process.h"


class ChassisCanPort
{
public:    
  ~ChassisCanPort();

  //  接收上报的CAN帧
  //ConnectPort: 在CAN设备和本地计算机之间创建端口代理。这允许该类使用TCP/IP套接字与CAN设备进行通信
  //bind: 这个方法用于解析从CAN设备接收到的CAN帧，并将它们传递给占位符_1
  //ConnectPort函数会将can_name参数和一个回调函数传递给TCP/IP套接字的异步读取操作。
  //当socket接收到一个CAN帧时，就会调用回调函数，并将接收到的CAN帧作为参数传递给该函数
  bool ConnectGet(std::string can_name) ;

  void EnableCommandedMode();

  CoreStateMsgGroup GetCoreStateToROS();


 protected:
  std::mutex core_state_mtx_;
  CoreStateMsgGroup core_state_msgs_;
  std::shared_ptr<AsyncCAN> can_;

  using CANFrameRxCallback = AsyncCAN::ReceiveCallback;

  // 定义一个名为ConnectPort的函数，它有三个参数：一个std::string类型的dev_name，表示要连接的网络设备的名称；
  // 一个CANFrameRxCallback类型的cb，表示接收到CAN帧时要调用的回调函数；一个bool类型的返回值，表示连接是否成功。
  // 在函数体中，创建一个AsyncCAN类型的智能指针can_，并用dev_name作为参数初始化它。
  // AsyncCAN是一个封装了网络设备操作的类，它可以异步地接收和发送CAN帧。
  // 调用can_指针所指对象的SetReceiveCallback方法，将cb作为参数传递给它。
  // 这样，当网络设备收到一个CAN帧时，就会自动调用cb函数来处理它。

  void ParseCANFrame(can_frame *rx_frame) ;

  void GetCoreStateFromCAN(const ChassisMessage &status_msg) ;
};

#endif /* CAN_PORT_H */
