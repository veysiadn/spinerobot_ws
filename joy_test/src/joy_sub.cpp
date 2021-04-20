/**
 * @file joy_sub.cpp
 * @author Veysi ADIN (veysi.adin@kist.re.kr)
 * @brief This file contains minimal joystick subscriber under topic named "joy"
 *         Gets data from joy_node, data contains axes and buttons values in array.
 *         to run it in ROS source your environment first.Then run command below.
 *         $ ros2 run controller joy_sub
 * @version 0.1
 * @date 2021-04-20
 * @note Don't forget to change CMakeList.txt and packages.xml files after changes.
 * @copyright Copyright (c) 2021
 * 
 */


#include <cstdio>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using std::placeholders::_1;

class JoySubscriber : public rclcpp::Node
{
    public  :
    /**
     * @brief Joysubscriber create a subscription with specific topic name "joy"
     * 
     */
    JoySubscriber() : Node("joy_sub")

    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoySubscriber::topic_callback, this, _1));
    }

  private:
  /**
   * @brief Joystick parameters callback function to print values in terminal.
   *        Printing is just for demonstration purposes.
   * 
   * @param msg joy_node published data variable.Note that msg has axes and buttons field.
   */
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
      
      RCLCPP_INFO(this->get_logger(), "----------------------------------------");
      RCLCPP_INFO(this->get_logger(), "Left X   : %f\n", msg->axes[0]);
      RCLCPP_INFO(this->get_logger(), "Left Y   : %f\n", msg->axes[1]);
      RCLCPP_INFO(this->get_logger(), "Right X  : %f\n", msg->axes[3]);
      RCLCPP_INFO(this->get_logger(), "Right Y  : %f\n", msg->axes[4]);
      RCLCPP_INFO(this->get_logger(), "----------------------------------------");
      /* 
      RCLCPP_INFO(this->get_logger(), "Left B   : %f\n", msg->axes[2]);
      RCLCPP_INFO(this->get_logger(), "Right B  : %f\n", msg->axes[5]);
      RCLCPP_INFO(this->get_logger(), "GButton  : %d\n", msg->buttons[0]);
      RCLCPP_INFO(this->get_logger(), "RButton  : %d\n", msg->buttons[1]);
      RCLCPP_INFO(this->get_logger(), "BButton  : %d\n", msg->buttons[2]);
      RCLCPP_INFO(this->get_logger(), "YButton  : %d\n", msg->buttons[3]);
      RCLCPP_INFO(this->get_logger(), "LfButton : %d\n", msg->buttons[4]);
      RCLCPP_INFO(this->get_logger(), "RgButton : %d\n", msg->buttons[5]);
      RCLCPP_INFO(this->get_logger(), "GButton  : %d\n", msg->buttons[6]);
      */

    }
    /// Subsriction definition for joystick msg
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  /// spinning is required for subscribers to check available data that's published to subscribed topic.
  rclcpp::spin(std::make_shared<JoySubscriber>());
  rclcpp::shutdown();
  return 0;
}
