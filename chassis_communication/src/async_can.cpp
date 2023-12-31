
#include "chassis_communication/async_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>

#include <iostream>

//socketcan_stream_，它是一个boost::asio::posix::stream_descriptor类型的对象。
//这个对象表示一个socketcan设备，可以用于异步输入/输出操作。
//构造函数将io_context_对象作为参数传递给socketcan_stream_构造函数。
//io_context_对象是AsyncCAN类的一个成员，它提供了对操作系统I/O服务的访问
AsyncCAN::AsyncCAN(std::string can_port)
    : port_(can_port), socketcan_stream_(io_context_) {}

AsyncCAN::~AsyncCAN() { Close(); }

bool AsyncCAN::Open() 
{
  try 
  {
    const size_t iface_name_size = strlen(port_.c_str()) + 1;
    if (iface_name_size > IFNAMSIZ) return false;

    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (can_fd_ < 0) return false;

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    memcpy(ifr.ifr_name, port_.c_str(), iface_name_size);

    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr); // 使用 ioctl() 函数 将套接字与 can 设备绑定
    if (ioctl_result < 0) 
    {
      Close();
      return false;
    }

    struct sockaddr_can addr;
    memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    //将套接字与CAN0绑定
    const int bind_result = bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr)); 
    if (bind_result < 0) 
    {
      Close();
      return false;
    }

    port_opened_ = true;
    std::cout << "开始监听CAN端口: " << port_ << std::endl;
  } 
  catch (std::system_error &e) 
  {
    port_opened_ = false;
    std::cout << e.what() << std::endl;
    return false;
  }

  // 将套接字can_fd_与asio库关联起来，这样就可以使用asio库提供的异步输入输出接口来操作套接字
  socketcan_stream_.assign(can_fd_); 

// 将一个可调用对象添加到asio::io_context对象的事件队列中，等待被执行
// ReadFromPort(): 异步从CAN总线端口【读取】一帧数据，并注册回调函数来处理接收到的数据帧
#if ASIO_VERSION < 101200L
  io_context_.post(std::bind(&AsyncCAN::ReadFromPort, this, 
                             std::ref(rcv_frame_),
                             std::ref(socketcan_stream_))); 
#else
  asio::post(io_context_,
             std::bind(&AsyncCAN::ReadFromPort, this, std::ref(rcv_frame_),
                       std::ref(socketcan_stream_)));
#endif

  // start io thread在其他线程中处理io数据
  io_thread_ = std::thread([this]() { io_context_.run(); });

  return true;
}

void AsyncCAN::Close() 
{
  io_context_.stop();
  if (io_thread_.joinable()) io_thread_.join();
  io_context_.reset();
  
  // release port fd
  const int close_result = ::close(can_fd_);
  can_fd_ = -1;

  port_opened_ = false;
}

bool AsyncCAN::IsOpened() const 
{ 
  return port_opened_; 
}

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame) 
{
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

void AsyncCAN::ReadFromPort(struct can_frame &rec_frame, asio::posix::basic_stream_descriptor<> &stream) 
{
  auto sthis = shared_from_this();
  stream.async_read_some(
      asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](asio::error_code error, size_t bytes_transferred) {
        if (error) {
          sthis->Close();
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        sthis->ReadFromPort(std::ref(sthis->rcv_frame_), std::ref(sthis->socketcan_stream_));
      });
}

void AsyncCAN::SendFrame(const struct can_frame &frame) 
{
  socketcan_stream_.async_write_some( asio::buffer(&frame, sizeof(frame)),[](asio::error_code error, size_t bytes_transferred) 
  {
        if (error) {
          std::cerr << "Failed to send CAN frame" << std::endl;
        }
      });
}

