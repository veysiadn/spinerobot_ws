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
/******************************************************************************
 *  \file   ecat_node.hpp
 *  \brief  ROS2 EtherCAT lifecycle node implementatiton header file 
 * 
 *   This header file contains blueprint of EthercatNode class which will be 
 *   responsible for EtherCAT communication.
 *    @todo After implementation explain functionality of this class and methods.
 *******************************************************************************/
#pragma once
/******************************************************************************/
#include "ecat_globals.hpp"
/******************************************************************************/
class EthercatSlave ;
#include "ecat_slave.hpp"
/******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joy.hpp"
/*****************************************************************************/
namespace EthercatCommunication
{
class EthercatNode
{
    public:
        /**
         *  EtherCAT node constructor.
         *  @todo extend constructors functionality.
         **/
        EthercatNode();
        ~EthercatNode();
    #ifdef LOGGING
    static void create_new_statistics_sample(StatisticsStruct *ss, unsigned int * sampling_counter);
    static void create_statistics(StatisticsStruct * ss, struct timespec * wakeup_time_p);
    static void log_statistics_to_file(StatisticsStruct *ss);
    #endif
    EthercatSlave slaves_[NUM_OF_SLAVES];
/**
 * @brief Requests master instance and creates a domain for a master.
 * @note  Keep in mind that created master and domain are global variables.
 * @return 0 if succesful otherwise -1.
 */
    int  ConfigureMaster();
/**
 * @brief Defines default connected slaves based on number of slaves.
 *        Specifies its position, vendor id , product code etc.
 *  Default connected slaves considered implementation specific. In our case it will be 3 motors and 
 *  one EasyCAT slave.
 */
    void DefineDefaultSlaves();
/**
 * @brief Passes your defined slave to EthercatNode class.
 * 
 * @param c_slave first create your own EthercatSlave instance and modify it then pass it to configuration.
 * @param position specify the physical connection position for your custom configured slave.
 */
    void SetCustomSlave(EthercatSlave c_slave, int position);
/**
 * @brief Obtains slave configuration for all slaves w.r.t master.
 * @return 0 if succesfull, otherwise -1. 
 */
    int  ConfigureSlaves();
/**
 * @brief Set mode to ProfilePositionMode with specified parameters for servo drive on that position.
 *       
 * @param P Profile position parameter structure specified by user.
 * @param position Slave position
 * @return 0 if succesfull, otherwise -1.
 */
    int  SetProfilePositionParameters(ProfilePosParam& P , int position);
/**
 * @brief Set the mode to ProfilePositionMode with specified Parameters for all servo drives on the bus.
 * 
 * @param P Profile position parameter structure specified by user.
 * @return 0 if succesful, otherwise -1.
 */
    int SetProfilePositionParametersAll(ProfilePosParam& P);
/**
 * @brief Set mode to ProfileVelocityMode with specified parameters for servo drive on that position.
 * 
 * @param P Profile velocity parameter structure specified by user.
 * @param position Slave position
 * @return 0 if succesful, -1 otherwise.
 */
    int SetProfileVelocityParameters(ProfileVelocityParam& P,int position);
/**
 * @brief Set mode to ProfileVelocityMode with specified parameters for all servo drives on the bus
 * 
 * @param P Profile velocity parameter structure specified by user.
 * @return 0 if succesfull, -1 otherwise.
 * @todo Add error code to all functions.Instead of returning -1. 
 */
    int SetProfileVelocityParametersAll(ProfileVelocityParam& P);
/**
 * @brief Maps default PDOs for our spine surgery robot implementation.
 * @note This method is specific for our spinerobot implementation.
 * If you have different topology or different servo drives use 
 * \see MapCustomPdos() function.
 * @return 0 if succesfull, otherwise -1.
 */
    int MapDefaultPdos();
/**
 * @brief Map Custom PDO based on your PDO mappings
 * @note  You have to specify slave syncs and slave pdo registers before using function
 * @param S EthercatSlave instance
 * @param position Physical position of your slave w.r.t master
 * @return 0 if succesfull, -1 otherwise.
 */
    int MapCustomPdos(ec_sync_info_t *syncs, ec_pdo_entry_reg_t *pdo_entry_reg, int position);
/**
 * @brief Configures DC sync for our default configuration
 * 
 */
    void ConfigDcSyncDefault();
/**
 * @brief Configures DC synchronization for specified slave position
 * 
 * @param assign_activate Activating DC synchronization for slave.
 * 0x300 for Elmo | and same for EasyCAT
 * @note Assign activate parameters specified in slaves ESI file 
 * 
 * @param position
 */
    void ConfigDcSync(uint16_t assign_activate, int position);

#if SDO_COMM        
    int  ConfigSdoRequests(SdoRequest& e_sdo);
    int  ReadSdo(ec_sdo_request_t *req, uint32_t& target);
    void WriteSdo(ec_sdo_request_t *req, uint32_t data);
#endif
/**
 * @brief This function will check slave's application layer states. (INIT/PREOP/SAFEOP/OP)
 */
    void CheckSlaveConfigurationState();
/**
 * @brief This function will check master's state, in terms of number of responding slaves and their application layer states
 * 
 * @return 0 if succesful, otherwise -1 
 * \see ec_master_state_t structure.
 **/
    int  CheckMasterState();
/**
 * @brief  Reads the state of a domain.
 * Stores the domain state in the given state structure.
 * Using this method, the process data exchange can be monitored in realtime.
 * */
    void CheckMasterDomainState();
/**
 * @brief Activates master, after this function call realtime operation can start.
 * \warning Before activating master all configuration should be done
 * \warning After calling this function you have to register domain(s) and start realtime task.
 * @return 0 if succesful, otherwise -1. 
 */
    int  ActivateMaster();
/**
 * @brief Registers domain for each slave.
 *  This method has to be called after ecrt_master_activate() to get the mapped domain process data memory. 
 * @return 0 if succeful , otherwise -1 
 */
    int  RegisterDomain();
/**
 * @brief Puts all slave to operational mode.User must call this before entering real-time operation.
 *        Reason for this function is that, master and slave has to do several exchange before becoming operational.
 *        So this function does exchange between master and slaves for up to 10 sec, could finish earlier.
 *        If timeout occurs it will return -1.
 * @return 0 if succesfull, otherwise -1.
 */
    int  WaitForOperationalMode();
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
 * @brief Realtime cyclic Pdo exchange function which will constantly read/write values from/to slaves
 * 
 * @param arg Used during pthread_create function to pass variables to realtime task. 
 * @return NULL
 */
    void *StartPdoExchange(void *arg);
/**
 * @brief Starts EtherCAT communcation
 * 
 * @return 0 if succesfull, otherwise -1.
 */
    int  StartEthercatCommunication();
/**
 * @brief Opens EtherCAT master via command line tool if it's not already on.
 * 
 * @return 0 if succesfull, otherwise -1.
 */
    int OpenEthercatMaster();


    int  IsOperational();
    void EnableDevice();


    void *MotorCyclicTask(void *arg);
    
    int  KeepInOPmode();
    void ResetMaster();
    public :
        float left_x_axis_;
        float left_y_axis_;
        float right_x_axis_;
        float right_y_axis_;
    private:
        pthread_t ethercat_thread_;
        struct sched_param ethercat_sched_param_ = {};
        pthread_attr_t ethercat_thread_attr_;
        int32_t err_;
        //Variable for opening EtherCAT master from CLI via code.
        int fd ; 
};
}