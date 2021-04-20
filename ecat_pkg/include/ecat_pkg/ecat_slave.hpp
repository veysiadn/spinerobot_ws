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
 *  \file   ecat_slave.hpp
 *  \brief  Ethercat slave class implementation header file.
 * 
 *   This header file contains blueprint of EtherCAT slave class which will be 
 *   used to  communicate with EtherCAT master.
 *    @todo After implementation explain functionality of this class and methods.
 *******************************************************************************/
#pragma once

#include "ecat_globals.hpp"


/******************************************************************************
 *  \class   EthercatSlave
 *  \brief   Contains EtherCAT slave parameters for configuration.
 * 
 *       This class is slave class for EtherCAT communication.User should specify all 
 *       variables in this section to be able to use specific slave.
 *    @todo After implementation explain functionality of this class and methods.
 *******************************************************************************/
class EthercatSlave
{
    public:
        EthercatSlave();
        ~EthercatSlave();
    /**
     * @brief This function will check slave's application layer states.
     *        (INIT/PREOP/SAFEOP/OP) 
     * @note This function shouldn't be called in real time context.For diagnosis 
     *       you can use CheckDomainState() encapsulation in ecat_node.
     * @return 0 if succesful. 
     */
    int CheckSlaveConfigState();
    /**
     * @brief Each slave have it's own vendor_id_ specified in it's ESI file.
     *        This variable will be used to define and configure slave.
     * for Elmo Gold Solo Twitter ; vendor_id_ = 0x0000009a
     * @todo add parameter for each slave information so that all slaves can be configured.
     */
    uint32_t   vendor_id_ ; // Each slave have it's own vendor_id_ specified in it's ESI file.

    /**
     * @brief Each slave have it's own product_code_ specified in it's ESI file.
     *        This information will be used to define and configure slave.
     * for Elmo Gold Solo Twitter ; product_code_ = 0x00030924 ;
     */
    uint32_t   product_code_ ;
    /// Alias will be always zero since we have only one master, without redundancy.
    const static uint16_t   kAlias_ = 0 ;
    /**
     * @brief Cycle time for slave
     * @todo  Cycle time should be parameterized and should be same for all slaves.
     */
    uint32_t   cycle_time_ ;
    /// DC sync shift setting, zero will give best synchronization.
    const static uint32_t   kSync0_shift_ = 0;
    /**
     * Slave's physical position w.r.t to master.
     * @note position should start from zero.
     **/ 
    uint16_t                position_ ; 
    /// Slave configuration parameters, assinged to each slave.
    ec_slave_config_t       *slave_config_ ;
    /// Slave state handle to check if slave is online and it's state machine(INIT/PREOP/SAFEOP/0P)
    ec_slave_config_state_t  slave_config_state_ ;

    /// PDO domain for data exchange
    uint8_t                *slave_pdo_domain_ ;

    /// Variable for checking motor state 
    int32_t                  motor_state_ ;

    /// Offset for PDO entries to assign pdo registers.
    OffsetPDO                 offset_ ;
    /// Received data from servo drivers.
    ReceivedData              data_ ;
    /// Slave velocity parameters.
    ProfileVelocityParam    velocity_param_ ;
    /// Slave position parameters.
    ProfilePosParam         position_param_ ;
    // Slave homing parameters. 
    HomingParam             homing_param_ ;

    /**
     * @brief To register all PDOs at once user can create an array of PDO to register to master.
     */
    /// 
    ec_pdo_entry_reg_t   * slave_pdo_regs_ ;
    /// PDO entry configuration information required to register PDOs
    ec_pdo_entry_info_t  * slave_pdo_entries_ ;
    /// PDO configuration information required for synchronization
    ec_pdo_info_t        * slave_pdo_indexes_ ;
    /// Sync manager configuration information for synchronization
    ec_sync_info_t       * slave_syncs_ ; 


/**
 * @brief This part is written for specific slave Elmo Gold Solo Twitter.
 *        PDO information of Elmo driver that'll be registered to EtherCAT master.
 */ 
    ec_pdo_entry_reg_t elmo_pdo_regs[20] = {
        // Input PDO mapping ; 
        {kAlias_, position_, vendor_id_,product_code_, OD_POSITION_ACTUAL_VAL ,       &this->offset_.actual_pos},
        {kAlias_, position_, vendor_id_,product_code_, OD_STATUS_WORD ,               &this->offset_.status_word},
        {kAlias_, position_, vendor_id_,product_code_, OD_VELOCITY_ACTUAL_VALUE ,     &this->offset_.actual_vel},
        ////////////////
        {kAlias_, position_, vendor_id_,product_code_, OD_OPERATION_MODE_DISPLAY ,     &this->offset_.op_mode_display},
        {kAlias_, position_, vendor_id_,product_code_, OD_POSITON_FOLLOWING_ERROR ,    &this->offset_.pos_fol_err},
        {kAlias_, position_, vendor_id_,product_code_, OD_TORQUE_ACTUAL_VALUE ,        &this->offset_.actual_tor},
        {kAlias_, position_, vendor_id_,product_code_, OD_CURRENT_ACTUAL_VALUE ,       &this->offset_.actual_cur},
        {kAlias_, position_, vendor_id_,product_code_, OD_ERROR_CODE ,                 &this->offset_.error_code},
        {kAlias_, position_, vendor_id_,product_code_, OD_EXTRA_STATUS_REGISTER ,      &this->offset_.extra_status_reg},
        //////////////
        // Output PDO Mapping ; 
        {kAlias_, position_, vendor_id_,product_code_, OD_TARGET_POSITION,            &this->offset_.target_pos},
        {kAlias_, position_, vendor_id_,product_code_, OD_TARGET_VELOCITY ,           &this->offset_.target_vel},
        {kAlias_, position_, vendor_id_,product_code_, OD_CONTROL_WORD ,              &this->offset_.control_word},
        ////////
        {kAlias_, position_, vendor_id_,product_code_, OD_TARGET_TORQUE,              &this->offset_.target_tor},
        {kAlias_, position_, vendor_id_,product_code_, OD_TORQUE_MAX ,                &this->offset_.max_tor},
        {kAlias_, position_, vendor_id_,product_code_, OD_OPERATION_MODE ,            &this->offset_.op_mode},
        {kAlias_, position_, vendor_id_,product_code_, OD_PROFILE_VELOCITY ,          &this->offset_.profile_vel},
        {kAlias_, position_, vendor_id_,product_code_, OD_PROFILE_ACCELERATION ,      &this->offset_.profile_acc},
        {kAlias_, position_, vendor_id_,product_code_, OD_PROFILE_DECELERATION ,      &this->offset_.profile_dec},
        {kAlias_, position_, vendor_id_,product_code_, OD_QUICK_STOP_DECELERATION ,   &this->offset_.quick_stop_dec},
        ////////////
        {}
    };
    /*******************************************************************************/
    ec_pdo_entry_info_t elmo_pdo_entries[8] = {
     // Outputs from master to slave , WRITE to this indexes
/////
/*
    {OD_TARGET_POSITION, 32},    // TARGET_POSITION
    {OD_TARGET_VELOCITY, 32},    // TARGET_VELOCITY
    {OD_TARGET_TORQUE,   16},    // TARGET_TORQUE
    {OD_TORQUE_MAX,      16},    // MAX_TORQUE
    {OD_CONTROL_WORD,    16},    // CONTROL_WORD
    {OD_OPERATION_MODE,   8},     // MODE_OF_OPERATION
    {0x0000, 0x00,    8},       //Gap 
    {OD_PROFILE_VELOCITY,        32},     // PROFILE_VELOCITY
    {OD_PROFILE_ACCELERATION,    32},     // PROFILE_ACCELERATION
    {OD_PROFILE_DECELERATION,    32},     // PROFILE_DECELERATION
    {OD_QUICK_STOP_DECELERATION,  32},     // QUICK_STOP_DECELERATION
    //{0x60b1, 0x00, 32},                   // VELOCITY_OFFSET

    // Inputs from slave to the master READ from this indexes
    {OD_POSITION_ACTUAL_VAL,     32},     // POSITION_ACTUAL_VALUE
    {OD_POSITON_FOLLOWING_ERROR, 32},     // POSITION_FOLLOWING_ERROR_ACTUAL_VALUE
    {OD_TORQUE_ACTUAL_VALUE, 16},        // TORQUE_ACTUAL_VALUE
    {OD_STATUS_WORD,        16},        // STATUS_WORD
    {OD_OPERATION_MODE_DISPLAY, 8},      // CUR_MODE_OF_OPERATION
    {0x0000, 0x00, 8},                  // Gap 
    {OD_VELOCITY_ACTUAL_VALUE,    32},      // VELOCITY_SENSOR_ACTUAL_VALUE (COUTNS/SEC)
    {OD_DC_CIRCUIT_LINK_VOLTAGE, 32},             //DC Link Circuit Voltage
    {OD_CURRENT_ACTUAL_VALUE, 16},        // CURRENT_ACTUAL_VALUE
    {OD_EXTRA_STATUS_REGISTER, 16},             // EXTRA_STATUS_REGISTER
    {OD_ERROR_CODE,    16},             // ERROR_CODE
*/
//////

    {OD_TARGET_POSITION, 32},
    {OD_DIGITAL_OUTPUTS, 32},
    {OD_CONTROL_WORD, 16},
    {OD_TARGET_VELOCITY,32},
    {OD_POSITION_ACTUAL_VAL, 32},
    {OD_DIGITAL_INPUTS, 32},
    {OD_STATUS_WORD,    16},
    {OD_VELOCITY_ACTUAL_VALUE,32}
};

    ec_pdo_info_t elmo_pdo_indexes[4] = {
    ///
    ///16XX From master to slave outputs e.g Target Position RxPDO
    /*! @note for more information check Elmo CIA402 PDO Index/subindex list.
    {0x1605, 7, elmo_pdo_entries + 0},   
    {0x1611, 1, elmo_pdo_entries + 7}, 
    {0x1613, 1, elmo_pdo_entries + 8},
    {0x1614, 1, elmo_pdo_entries + 9},
    {0x161f, 1, elmo_pdo_entries + 10},

    ///1AXX From slave to the master inputs e.g Actual Position TxPDO
    {0x1a04, 6, elmo_pdo_entries + 11},   
    {0x1a0f, 1, elmo_pdo_entries + 17},
    {0x1a18, 1, elmo_pdo_entries + 18},
    {0x1a1f, 1, elmo_pdo_entries + 19},
    {0x1a21, 1, elmo_pdo_entries + 20},
    {0x1a26, 1, elmo_pdo_entries + 21}, */
    {0x1600, 3, elmo_pdo_entries + 0},
    {0x1607, 1, elmo_pdo_entries + 3},

    {0x1a00, 3, elmo_pdo_entries + 4},
    {0x1a07, 1, elmo_pdo_entries + 7}
};

    ec_sync_info_t elmo_syncs[5] = {
        /*
        {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
        {1, EC_DIR_INPUT,  0, NULL, EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 5, elmo_pdo_indexes + 0, EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  6, elmo_pdo_indexes + 5, EC_WD_DISABLE},
        {0xff}
        */
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 2, elmo_pdo_indexes + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 2, elmo_pdo_indexes + 2, EC_WD_DISABLE},
    {0xff}
    };
    /*******************************************************************************/
/* Master 0, Slave 0, "EasyCAT_Slave"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa002
 * Revision number: 0x00000001
 */
ec_pdo_entry_reg_t easycat_pdo_regs[]{{}};
ec_pdo_entry_info_t easycat_slave_pdo_entries[] = {
    {0x0005, 0x01, 8},  /* Segments */
    {0x0006, 0x01, 16}, /* Potentiometer */
    {0x0006, 0x02, 8},  /* Switches */
};

ec_pdo_info_t easycat_slave_pdos[] = {
    {0x1600, 1, easycat_slave_pdo_entries + 0}, /* Outputs */
    {0x1a00, 2, easycat_slave_pdo_entries + 1}, /* Inputs */
};

ec_sync_info_t easycat_slave_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, easycat_slave_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, easycat_slave_pdos + 1, EC_WD_DISABLE},
    {0xff}
};


};