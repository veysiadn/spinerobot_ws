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

        ecat_msgs::msg::DataReceived received_data_[NUM_OF_SLAVES];
        ecat_msgs::msg::DataSent    sent_data_[NUM_OF_SLAVES];

        std::unique_ptr<EthercatNode>                               ecat_node_;
        
        
        /**
        * @brief This function handles callbacks from subscribtions.
        *        It will update controller subscription values .
        */
        void HandleCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg);

        /**
         * @brief Sets Ethercat communication thread's properties 
         *        After this function called user must call StartEthercatCommunication() function]
         * @return 0 if succesfull, otherwise -1.
         */
        int SetComThreadPriorities();
        /**
         * @brief Encapsulates all configuration steps for the EtherCAT communication with default slaves.
         *        And waits for connected slaves to become operational.
         * @return 0 if succesful otherwise -1. 
         */
        int InitEthercatCommunication() ;

        /**
         * @brief Helper function to enter pthread_create, since pthread's are C function it doesn't
         *        accept class member function, to pass class member function this helper function is 
         *        created.
         * 
         * @param arg Pointer to current class instance.
         * @return void* 
         */
        static void *PassCycylicExchange(void *arg);

        /**
         * @brief Starts EtherCAT communcation
         * 
         * @return 0 if succesfull, otherwise -1.
         */
        int  StartEthercatCommunication(); 

        /**
         * @brief Realtime cyclic Pdo exchange function which will constantly read/write values from/to slaves
         * 
         * @param arg Used during pthread_create function to pass variables to realtime task. 
         * @return NULL
         */
        void StartPdoExchange(void *instance); 
        /**
         * @brief Gets  master's communication state.
         *  \see ec_al_state_t
         * 
         * @return Application layer state for master.
         */
        int GetComState();
        /**
         * @brief Reads data from slaves and updates received data structure to be published
         */
        void UpdateReceivedData();
        /**
         * @brief Publishes all data that master received and will be sent
         * 
         * @return 0 if succesfull otherwise -1. 
         */
        int PublishAllData();
    private : 
        pthread_t ethercat_thread_;
        struct sched_param ethercat_sched_param_ = {};
        pthread_attr_t ethercat_thread_attr_;
        int32_t err_;
        //Variable for opening EtherCAT master from CLI via code.
        uint8_t al_state_ = 0; 
        float left_x_axis_;
        float left_y_axis_;
        float right_x_axis_;
        float right_y_axis_;     
};
}