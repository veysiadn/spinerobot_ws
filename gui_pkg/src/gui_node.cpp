#include "../include/gui_pkg/gui_node.hpp"

  using namespace std::chrono_literals;
  using namespace GUI;
  GuiNode::GuiNode() : Node("gui_node")
  {
      controller_commands_= this->create_subscription<sensor_msgs::msg::Joy>("Controller", 10,
                                          std::bind(&GuiNode::HandleControllerCallbacks, this, std::placeholders::_1));
      slave_feedback_ = this->create_subscription<ecat_msgs::msg::DataReceived>("Slave_Feedback", 10,
                                           std::bind(&GuiNode::HandleSlaveFeedbackCallbacks, this, std::placeholders::_1));
      master_commands_ = this->create_subscription<ecat_msgs::msg::DataSent>("Master_Commands", 10,
                                           std::bind(&GuiNode::HandleMasterCommandCallbacks, this, std::placeholders::_1));
  }

  GuiNode::~GuiNode()
  {
    rclcpp::shutdown();
  }

  void GuiNode::HandleControllerCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
     for(int i=0; i < NUM_OF_SLAVES ; i++){
        received_data_[i].right_x_axis = msg->axes[3];
        received_data_[i].left_x_axis =  msg->axes[0];
     }
    // emit UpdateParameters(0);
  }

  void GuiNode::HandleMasterCommandCallbacks(const ecat_msgs::msg::DataSent::SharedPtr msg)
  {
      for(int i=0; i < NUM_OF_SLAVES ; i++){
         received_data_[i].target_pos   =  msg->target_pos[i];
         received_data_[i].target_vel   =  msg->target_vel[i];
         received_data_[i].control_word =  msg->control_word[i];
      }
     // emit UpdateParameters(0);
  }

  void GuiNode::HandleSlaveFeedbackCallbacks(const ecat_msgs::msg::DataReceived::SharedPtr msg)
  {
      for(int i=0; i < NUM_OF_SLAVES ; i++){
        received_data_[i].actual_pos             =  msg->actual_pos[i];
        received_data_[i].actual_vel             =  msg->actual_vel[i];
        received_data_[i].status_word            =  msg->status_word[i];
        received_data_[i].left_limit_switch_val  =  msg->left_limit_switch_val;
        received_data_[i].right_limit_switch_val =  msg->right_limit_switch_val;
    }
     // emit UpdateParameters(0);
  }

