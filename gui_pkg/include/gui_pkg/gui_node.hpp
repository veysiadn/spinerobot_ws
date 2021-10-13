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

// Message file headers, -custom and built-in-
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "ecat_msgs/msg/data_received.hpp"
#include "ecat_msgs/msg/data_sent.hpp"

#include <rclcpp/rclcpp.hpp>    // Standard ROS2 header API
#include <rclcpp/strategies/message_pool_memory_strategy.hpp>   // /// Completely static memory allocation strategy for messages.
#include <rclcpp/strategies/allocator_memory_strategy.hpp>
/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */

#include <rttest/rttest.h>   // To get number of allocation, statistics related memory allocation

#include <tlsf_cpp/tlsf.hpp>   // C++ wrapper for Miguel Masmano Tello's implementation of the TLSF memory allocator
// Implements the allocator_traits template
using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

template<typename T = void>
using TLSFAllocator = tlsf_heap_allocator<T>;
//CPP
#include <vector>
#include <chrono>
#include <memory>
#include <functional>
#include <string>
#include <cstdint>
#include <ctime>
#include <ratio>
#include <fstream>
#include <string>
#include "timing.hpp"
using namespace std::chrono_literals;

#define NUM_OF_SERVO_DRIVES 1

#define TEST_BIT(NUM,N)    ((NUM &  (1 << N))>>N)  // Check specific bit in the data. 0 or 1.
#define SET_BIT(NUM,N)      (NUM |  (1 << N))  // Set(1) specific bit in the data.
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))  // Reset(0) specific bit in the data

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
    uint8_t  p_emergency_switch_val;
    uint8_t  com_status;
}ReceivedData;

 class GuiNode : public rclcpp::Node
  {
     //Q_OBJECT
  public:
      GuiNode();
      virtual ~GuiNode();
  public:
      // Received data structure to store all subscribed data.
      ReceivedData received_data_[NUM_OF_SERVO_DRIVES];
      // GUI button value to publish emergency button state.
      uint8_t emergency_button_val_ = 1;
      Timing time_info_;
  private:  
      // ROS2 subscriptions.
      rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr slave_feedback_;
      rclcpp::Subscription<ecat_msgs::msg::DataSent>::SharedPtr master_commands_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_commands_;
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gui_publisher_;
      /**
       * @brief Publishes gui button value in specified interval.
       */
      void timer_callback();
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
  };// class GuiNode

 } // namespace GUI
