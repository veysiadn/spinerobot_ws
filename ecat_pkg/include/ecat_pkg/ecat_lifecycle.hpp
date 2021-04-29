#include "ecat_node.hpp"
/******************************************************************************/
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
/******************************************************************************/
/// Interface header files.Contains custom msg files.
#include "ecat_msgs/msg/data_received.hpp"
#include "ecat_msgs/msg/data_sent.hpp"
/******************************************************************************/
using namespace rclcpp_lifecycle ;
using namespace EthercatCommunication ; 
namespace EthercatLifeCycleNode
{
class EthercatLifeCycle : public LifecycleNode
{

        public:
        EthercatLifeCycle();
        ~EthercatLifeCycle();
    private:
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State &);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const State &);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State &);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State &);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State &);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State &);

        rclcpp::TimerBase::SharedPtr timer_;
        LifecyclePublisher<ecat_msgs::msg::DataReceived>::SharedPtr received_data_publisher_;
        LifecyclePublisher<ecat_msgs::msg::DataSent>::SharedPtr     sent_data_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      joystick_subscriber_;
        std::unique_ptr<EthercatNode>                               ecat_node_;
        
        
        /**
        * @brief This function handles callbacks from subscribtions.
        *        It will update controller subscription values .
        */
        void HandleCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg);
};
}