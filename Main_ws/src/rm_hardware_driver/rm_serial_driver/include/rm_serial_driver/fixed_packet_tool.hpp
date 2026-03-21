// Copyright (C) 2021 RoboMaster-OSS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
#define SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_

// std
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <string>
#include <thread>
// project
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/transporter_interface.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::serial_driver {

template <int capacity = 16>
class FixedPacketTool {
public:
  using SharedPtr = std::shared_ptr<FixedPacketTool>;
  FixedPacketTool() = delete;
  explicit FixedPacketTool(std::shared_ptr<TransporterInterface> transporter)
  : transporter_(transporter) {
    if (!transporter) {
      throw std::invalid_argument("transporter is nullptr");
    }
    FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
  }

  ~FixedPacketTool() { enbaleRealtimeSend(false); }

  bool isOpen() { return transporter_->isOpen(); }
  void enbaleRealtimeSend(bool enable);
  void enbaleDataPrint(bool enable) { use_data_print_ = enable; }
  bool sendPacket(const FixedPacket<capacity> &packet);
  bool recvPacket(FixedPacket<capacity> &packet);

  std::string getErrorMessage() { return transporter_->errorMessage(); }

private:
  bool checkPacket(uint8_t *tmp_buffer, int recv_len);
  bool simpleSendPacket(const FixedPacket<capacity> &packet);

private:
  std::shared_ptr<TransporterInterface> transporter_;
  // data
  uint8_t tmp_buffer_[capacity];       // NOLINT
  uint8_t recv_buffer_[capacity * 2];  // NOLINT
  int recv_buf_len_;
  // for realtime sending
  bool use_realtime_send_{false};
  bool use_data_print_{false};
  std::mutex realtime_send_mut_;
  std::unique_ptr<std::thread> realtime_send_thread_;
  std::queue<FixedPacket<capacity>> realtime_packets_;
};

template <int capacity>
bool FixedPacketTool<capacity>::checkPacket(uint8_t *buffer, int recv_len) {
  // 检查长度
  if (recv_len != capacity) {
    return false;
  }
  // 检查帧头，帧尾,
  // if ((buffer[0] != 0xff) || (buffer[capacity - 1] != 0x0d)) {
  //修改为spr的通信协议
  if ((buffer[0] != 0xFF) || (buffer[capacity - 1] != 0xFE)) {
    return false;
  }
  // TODO(gezp): 检查check_byte(buffer[capacity-2]),可采用异或校验(BCC)
  return true;
}

template <int capacity>
bool FixedPacketTool<capacity>::simpleSendPacket(const FixedPacket<capacity> &packet) {
  if (transporter_->write(packet.buffer(), capacity) == capacity) {
    return true;
  } else {
    // reconnect
    FYT_ERROR("serial_driver", "transporter_->write() failed");
    transporter_->close();
    transporter_->open();
    return false;
  }
}

template <int capacity>
void FixedPacketTool<capacity>::enbaleRealtimeSend(bool enable) {
  if (enable == use_realtime_send_) {
    return;
  }
  if (enable) {
    use_realtime_send_ = true;
    realtime_send_thread_ = std::make_unique<std::thread>([&]() {
      FixedPacket<capacity> packet;
      while (use_realtime_send_) {
        bool empty = true;
        {
          std::lock_guard<std::mutex> lock(realtime_send_mut_);
          empty = realtime_packets_.empty();
          if (!empty) {
            packet = realtime_packets_.front();
            realtime_packets_.pop();
          }
        }
        if (!empty) {
          simpleSendPacket(packet);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      }
    });
  } else {
    use_realtime_send_ = false;
    realtime_send_thread_->join();
    realtime_send_thread_.reset();
  }
}

template <int capacity>
bool FixedPacketTool<capacity>::sendPacket(const FixedPacket<capacity> &packet) {
  if (use_realtime_send_) {
    // 實時發送模式：將數據包推入隊列，由後台線程發送
    std::lock_guard<std::mutex> lock(realtime_send_mut_);
    realtime_packets_.push(packet);
    return true;
  }
  // 非實時模式：直接通過串口發送
  return simpleSendPacket(packet);
}

template <int capacity>
bool FixedPacketTool<capacity>::recvPacket(FixedPacket<capacity> &packet) {
  // 1. 读取新数据，尝试填满我们的剩余缓冲空间（recv_buffer_ 大小是 capacity * 2）
  //    最多读取 capacity 长度，防止溢出。
  int recv_len = transporter_->read(tmp_buffer_, capacity);
  
  if (recv_len > 0) {
    if (use_data_print_) {
      for (int i = 0; i < recv_len; i++) {
        std::cout << std::hex << static_cast<int>(tmp_buffer_[i]) << " ";
      }
      std::cout << "\n";
    }

    // 2. 防溢出保护：如果 recv_buffer_ 装不下了，说明里面全是垃圾数据，清空它。
    if (recv_buf_len_ + recv_len > capacity * 2) {
      FYT_WARN("serial_driver", "Buffer overflow, flushing buffer. Possible heavy noise.");
      recv_buf_len_ = 0; // 丢弃所有旧缓存
    }

    // 3. 将新读取的数据拼接到缓冲区尾部
    memcpy(recv_buffer_ + recv_buf_len_, tmp_buffer_, recv_len);
    recv_buf_len_ += recv_len;

    // 4. 滑动窗口寻找合法帧（遍历整个已有缓冲区）
    for (int i = 0; i <= recv_buf_len_ - capacity; i++) {
      // 检查：是否符合帧头 0xFF，帧尾 0xFE，以及特定的协议约束 [2]==0x00, [3]==0x00
      if (recv_buffer_[i] == 0xFF && 
          recv_buffer_[i + 2] == 0x00 && 
          recv_buffer_[i + 3] == 0x00 && 
          recv_buffer_[i + capacity - 1] == 0xFE) {
        
        // 找到了完整合法的一帧！拷贝出数据
        packet.copyFrom(recv_buffer_ + i);

        // 5. 寻帧成功后，将已处理的数据（包括当前帧之前的错位垃圾）从缓冲区丢弃
        int processed_len = i + capacity;
        int remain_len = recv_buf_len_ - processed_len;
        
        // 将后面剩余的数据（半包）挪到缓冲区最前面
        if (remain_len > 0) {
          memmove(recv_buffer_, recv_buffer_ + processed_len, remain_len);
        }
        recv_buf_len_ = remain_len;

        return true; // 成功解析出一帧
      }
    }

    // 5. 遍历完都没有找到合法的包
    // 可能是因为剩下的数据不够一帧长（半包），留到下一次 read 凑齐即可。
    // 为了防止陷入死锁，如果缓冲区马上满了却还没找到合法帧头，丢弃开头的一个字节来强行滑动。
    if (recv_buf_len_ > capacity * 1.5) {
      FYT_INFO("serial_driver", "Sliding window stuck, dropping garbage bytes...");
      int drop_len = recv_buf_len_ - capacity; // 强制丢弃旧数据，只保留最新的 capacity 长度
      memmove(recv_buffer_, recv_buffer_ + drop_len, capacity);
      recv_buf_len_ = capacity;
    }

    return false; // 当前尚未凑齐/找到合法帧，返回false等待下次调度

  } else if (recv_len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    // 6. 如果系统 read 发生真实错误（排除了非阻塞模式下的空读），说明硬件确实断开连接
    FYT_ERROR("serial_driver", "Hardware disconnected, read() failed. Rebooting port...");
    transporter_->close();
    transporter_->open();
    recv_buf_len_ = 0; // 硬件重启，清空缓存
    return false;
  }
  
  return false;
}

using FixedPacketTool16 = FixedPacketTool<16>;
using FixedPacketTool32 = FixedPacketTool<32>;
using FixedPacketTool64 = FixedPacketTool<64>;

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_FIXED_PACKET_TOOL_HPP_
