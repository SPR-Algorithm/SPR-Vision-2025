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

#include "rm_serial_driver/protocol/test_protocol.hpp"
// ros2
#include <geometry_msgs/msg/twist.hpp>

namespace fyt::serial_driver::protocol {

TestProtocol::TestProtocol(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  // packet_tool33 = std::make_shared<FixedPacketTool<33>>(uart_transporter);
  // packet_tool33->enbaleDataPrint(enable_data_print);
  packet_tool16 = std::make_shared<FixedPacketTool<16>>(uart_transporter);
  packet_tool16->enbaleDataPrint(enable_data_print);
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> TestProtocol::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  return {node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); })
  };
}

//下面没用
//用来订阅tracker的id消息，填充gimbal里面的id数据，对应我们电控的逻辑
// std::vector<rclcpp::SubscriptionBase::SharedPtr> TestProtocol::getidSubscription(
//   rclcpp::Node::SharedPtr node) {
//   return {node->create_subscription<rm_interfaces::msg::Target>(
//     "armor_solver/target",
//     rclcpp::SensorDataQoS(),
//     [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); })

//   };
// }


std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> TestProtocol::getClients(
  rclcpp::Node::SharedPtr node) const {
  return {node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                           rmw_qos_profile_services_default)};
}

void TestProtocol::send(const rm_interfaces::msg::GimbalCmd &data) {
  // FixedPacket<16> packet16;
  packet16.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  packet16.loadData<float>(static_cast<float>(data.yaw), 2);
  //交换了pitch和distance的位置
  packet16.loadData<float>(static_cast<float>(data.pitch), 6);
  packet16.loadData<float>(static_cast<float>(data.distance), 10);
  //添加我们的enemy_id，暂时不做
  // packet.loadData<float>(static_cast<float>(data.enemyid), 14);
  packet_tool16->sendPacket(packet16);
}

bool TestProtocol::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<16> packet;
  if (packet_tool16->recvPacket(packet)) {
     // game status
    uint8_t enemy_color;
    packet.unloadData(enemy_color, 1);
    // data.mode = (enemy_color == ENEMY_BLUE ? 1 : 0);
    data.mode = enemy_color;
    packet.unloadData(data.pitch, 4);
    packet.unloadData(data.yaw, 8);
    //std::cout<<"data.pitch:"<<data.pitch<<std::endl;
    //std::cout<<"data.yaw:"<<data.yaw<<std::endl;

    // 实际上是底盘角度
    // packet.unloadData(data.chassis_yaw, 10);
    // blood
    // packet.unloadData(data.judge_system_data.blood, 14);
    // remaining time
    // packet.unloadData(data.judge_system_data.remaining_time, 16);
    // outpost hp
    // packet.unloadData(data.judge_system_data.outpost_hp, 20);
    // operator control message
    // packet.unloadData(data.judge_system_data.operator_command.is_outpost_attacking, 22);
    // packet.unloadData(data.judge_system_data.operator_command.is_retreating, 23);
    // packet.unloadData(data.judge_system_data.operator_command.is_drone_avoiding, 24);

    // packet.unloadData(data.judge_system_data.game_status, 25);

    data.bullet_speed = 25;
    return true;
  } else {
    return false;
  }
}

// bool TestProtocol::reset() {
//   packet_tool16->reset();
//   return true;
// }

}  // namespace fyt::serial_driver::protocol
