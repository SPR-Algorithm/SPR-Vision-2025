// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.
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

#ifndef SERIAL_DRIVER_TEST_PROTOCOL_HPP_
#define SERIAL_DRIVER_TEST_PROTOCOL_HPP_

#include "rm_interfaces/msg/chassis_cmd.hpp"
#include "rm_serial_driver/protocol.hpp"

namespace fyt::serial_driver::protocol {
// 默认
class TestProtocol : public Protocol {
public:
  explicit TestProtocol(std::string_view port_name, bool enable_data_print);

  ~TestProtocol() = default;

  void send(const rm_interfaces::msg::GimbalCmd &data) override;

  bool receive(rm_interfaces::msg::SerialReceiveData &data) override;

  std::vector<rclcpp::SubscriptionBase::SharedPtr> getSubscriptions(rclcpp::Node::SharedPtr node) override;

  std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> getClients(
    rclcpp::Node::SharedPtr node) const override;

  std::string getErrorMessage() override { return packet_tool16->getErrorMessage(); }

  //bool reset();

private:
  // FixedPacketTool<16>::SharedPtr packet_tool_;
  enum GameStatus { NOT_START = 0x00, ENEMY_RED = 0x01, ENEMY_BLUE = 0x02 };
  // FixedPacketTool<33>::SharedPtr packet_tool33;
  FixedPacketTool<16>::SharedPtr packet_tool16;
  // FixedPacket<33> packet33;
  FixedPacket<16> packet16;
  rm_interfaces::msg::ChassisCmd chassis_cmd_;
};
}  // namespace fyt::serial_driver::protocol

#endif  // SERIAL_DRIVER_TEST_PROTOCOL_HPP_
