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

#define NUM_OF_SLAVES 3
// QT
//#include <QMainWindow>
//#include <QApplication>
namespace GUI {


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
      void HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg);
      void HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg);
      void HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg);
      ReceivedData received_data_[NUM_OF_SLAVES];
  //signals:
 //    void UpdateParameters(int s);
  private:
      rclcpp::Subscription<ecat_msgs::msg::DataReceived>::SharedPtr slave_feedback_;
      rclcpp::Subscription<ecat_msgs::msg::DataSent>::SharedPtr master_commands_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr  controller_commands_;

  };// class GuiNode

 } // namespace GUI
