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
 * \file  ecat_lifecycle.hpp
 * \brief Ethercat lifecycle node implementation and real-time communication
 *        loop implemented in this file.
 *******************************************************************************/
#include "ecat_node.hpp"
/******************************************************************************/
/// ROS2 lifecycle node header files.
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
    /**
     * @brief Ethercat lifecycle node configuration function, node will start with this function
     *        For more information about Lifecyclenode and it's interfaces check below link :
     *         https://design.ros2.org/articles/node_lifecycle.html
     *         \see node_interfaces::LifecycleNodeInterface::CallbackReturn
     * @return Success if configuration succesfull,otherwise FAILURE 
     */
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State &);
    /**
     * @brief Activates Ethercat lifecycle node and starts real-time Ethercat communication.
     *        All publishing is done in real-time loop in this active state.
     * 
     * @return Success if activation succesfull,otherwise FAILURE
     */
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const State &);
    /**
     * @brief Deactivates Ethercat lifecycle node, turns of real-time communication.
     * 
     * @return Success if deactivation succesfull,otherwise FAILURE
     */
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State &);
    /**
     * @brief Cleans up all variables and datas assigned by Ethercat lifecycle node. 
     * 
     * @return Success if cleanup succesfull,otherwise FAILURE 
     */
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State &);
    /**
     * @brief Shuts down EtherCAT lifecycle node, releases Ethercat master.
     * 
     * @return Success if shut down succesfull,otherwise FAILURE 
     */
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State &);
    /**
     * @brief There isn't any error recovery functionality for this node, just resets nodes.
     *         Reconfiguration is needed for restarting communication.
     * 
     * @return Success 
     */
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State &);

        rclcpp::TimerBase::SharedPtr timer_;
        /// This lifecycle publisher will be used to publish received feedback data from slaves.
        LifecyclePublisher<ecat_msgs::msg::DataReceived>::SharedPtr received_data_publisher_;
        /// This lifecycle publisher will be used to publish sent data from master to slaves.
        LifecyclePublisher<ecat_msgs::msg::DataSent>::SharedPtr     sent_data_publisher_;
        /// This subscriber  will be used to receive data from controller node.
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      joystick_subscriber_;

        ecat_msgs::msg::DataReceived    received_data_;
        ecat_msgs::msg::DataSent        sent_data_;

        std::unique_ptr<EthercatNode>    ecat_node_;
        
        
        /**
        * @brief This function handles callbacks from subscribtions.
        *        It will update received values from controller node.
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
        void ReadFromSlaves();
        /**
         * @brief Publishes all data that master received and will be sent
         * 
         * @return 0 if succesfull otherwise -1. 
         */
        int PublishAllData();
        /**
         * @brief Enables connected motor drives based on CIA402
         * 
         */
        void EnableMotors();
        /**
         * @brief Updates data that will be sent to slaves.
         *        This updated data will be published as well.
         */
        void WriteToSlaves();
        /**
         * @brief Acquired data from subscribed controller topic will be assigned as 
         *        motor speed parameter.
         */
        void UpdateControlParameters();

        int CheckMotorState();
    private : 
        /// pthread create required parameters.
        pthread_t ethercat_thread_;
        struct sched_param ethercat_sched_param_ = {};
        pthread_attr_t ethercat_thread_attr_;
        int32_t err_;
        /// Application layer of slaves seen by master.(INIT/PREOP/SAFEOP/OP)
        uint8_t al_state_ = 0; 
        uint32_t motor_state_[NUM_OF_SLAVES];
        uint32_t command = 0x004F;
        /// Values will be sent by controller node and will ne assigned to variables below.
        float left_x_axis_;
        float left_y_axis_;
        float right_x_axis_;
        float right_y_axis_;     
};
}