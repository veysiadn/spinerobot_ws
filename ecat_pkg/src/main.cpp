/****************************************************************************/
#include "rclcpp/rclcpp.hpp"
/****************************************************************************/
#include "ecat_node.hpp"
/****************************************************************************/
#include "ecat_lifecycle.hpp"
std::unique_ptr<EthercatLifeCycleNode::EthercatLifeCycle> ecat_lifecycle_node;

void signalHandler(int /*signum*/)
{
    /// @todo Add shutdown command here.
}

int main(int argc, char **argv)
{
   
    signal(SIGINT,signalHandler);

    rclcpp::init(argc, argv);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Mlockall failed, check if you have sudo authority.");
        return -1;
    }
    ecat_lifecycle_node = std::make_unique<EthercatLifeCycleNode::EthercatLifeCycle>();
    rclcpp::spin(ecat_lifecycle_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

