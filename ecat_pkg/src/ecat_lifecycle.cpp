#include <ecat_lifecycle.hpp>

using namespace EthercatLifeCycleNode ; 
EthercatLifeCycle::EthercatLifeCycle(): LifecycleNode("ecat_life_cycle_node")
{
    
    ecat_node_= std::make_unique<EthercatNode>();
    /// @todo for fault injection probably you'll have to declare parameters here.
    /// this->declare_parameter("")
}

EthercatLifeCycle::~EthercatLifeCycle()
{
    ecat_node_.reset();
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_configure(const State &)
{
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuring EtherCAT device...");

    if(ecat_node_->InitEthercatCommunication())
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuration phase failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }else{

        received_data_publisher_ = this->create_publisher<ecat_msgs::msg::DataReceived>("Slave_Feedback", 10);
        sent_data_publisher_     = this->create_publisher<ecat_msgs::msg::DataSent>("Master_Commands", 10);
        joystick_subscriber_     = this->create_subscription<sensor_msgs::msg::Joy>("Controller", 10, 
                                    std::bind(&EthercatLifeCycle::HandleCallbacks, this, std::placeholders::_1));
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_activate(const State &)
{
    if(ecat_node_->StartEthercatCommunication()){

        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Activation phase failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }else{
        RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Activation complete, real-time communication started.");
        received_data_publisher_->on_activate();
        sent_data_publisher_->on_activate();
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_deactivate(const State &)
{
    received_data_publisher_->on_deactivate();
    sent_data_publisher_->on_deactivate();
    ecat_node_->DeactivateCommunication();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_cleanup(const State &)
{
    ecat_node_.reset();
    received_data_publisher_.reset();
    sent_data_publisher_.reset();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_shutdown(const State &)
{
    ecat_node_->ReleaseMaster();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_error(const State &)
{
    ecat_node_.reset();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EthercatLifeCycle::HandleCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    ecat_node_->left_x_axis_  = msg->axes[0];
    ecat_node_->left_y_axis_  = msg->axes[1];
    ecat_node_->right_x_axis_ = msg->axes[2];
    ecat_node_->right_y_axis_ = msg->axes[3];
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