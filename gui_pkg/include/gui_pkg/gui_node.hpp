/******************************************************************************
 *
 *  $Id$
 *
 *  Copyright (C) 2021 Veysi ADIN, UST KIST
 *
 *  This file is part of the IgH EtherCAT master userspace program in the ROS2 environment.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is free software; you can
 *  redistribute it and/or modify it under the terms of the GNU General
 *  Public License as published by the Free Software Foundation; version 2
 *  of the License.
 *
 *  The IgH EtherCAT master userspace program in the ROS2 environment is distributed in the hope that
 *  it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 *  warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with the IgH EtherCAT master userspace program in the ROS environment. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 *  ---
 *
 *  The license mentioned above concerns the source code only. Using the
 *  EtherCAT technology and brand is only permitted in compliance with the
 *  industrial property and similar rights of Beckhoff Automation GmbH.
 *
 *  Contact information: veysi.adin@kist.re.kr
 *****************************************************************************/
/*****************************************************************************
 * \file  gui_node.hpp
 * \brief GUI node implementation to show slave status and controller commands in ROS2
 *        GUI node is a ROS2 node which subscribes EthercatLifecycle Node topics and 
 *        controller topics and shows those values via GUI.
 *******************************************************************************/

#pragma once
//ROS2
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "ecat_msgs/msg/data_received.hpp"
#include "ecat_msgs/msg/data_sent.hpp"

//CPP
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>

// Actually this should be number of servo drives
#define NUM_OF_SLAVES 2
// QT
//#include <QMainWindow>
//#include <QApplication>
namespace GUI {

/**
 * @brief This structure will be all data that'll be received by this node from other nodes.
 * 
 */
typedef struct
{
    int32_t   target_pos ;
    int32_t   target_vel ;
    int16_t   target_tor ;
    int16_t   max_tor ;
    uint16_t  control_word ;
    int32_t   vel_offset ;
    int16_t   tor_offset ;

    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    int8_t   op_mode_display ;
    uint8_t  left_limit_switch_val ;
    uint8_t  right_limit_switch_val ;
    int32_t  right_x_axis;
    int32_t  left_x_axis;
}ReceivedData;

 class GuiNode : public rclcpp::Node
  {
     //Q_OBJECT
  public:
      GuiNode();
      virtual ~GuiNode();
  public:
    /**
     * @brief Function will be used for subscribtion callbacks from controller node
     *        for Controller topic.
     * 
     * @param msg controller command structure published by controller node.
     */
      void HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg);
      /**
       * @brief Function will be used for subscribtion callbacks from EthercatLifecycle node 
       *        for Master_Commands topic.
       * 
       * @param msg Master commands structure published by EthercatLifecycle node
       */
      void HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg);
      /**
       * @brief Function will be used for subscribtion callbacks from EthercatLifecycle node 
       *        for Master_Commands topic.
       * 
       * @param msg Slave feedback structure published by EthercatLifecycle node
       */
      void HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg);
      ReceivedData received_data_[NUM_OF_SLAVES];
  //signals:
 //    void UpdateParameters(int s);
  private:
      // ROS2 subscribtions.
      rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr slave_feedback_;
      rclcpp::Subscription<ecat_msgs::msg::DataSent>::SharedPtr master_commands_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_commands_;

  };// class GuiNode

 } // namespace GUI
