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

 
    /// DC sync shift setting, zero will give best synchronization.
    const static uint32_t   kSync0_shift_ = 0;

    /// Slave configuration parameters, assinged to each slave.
    ec_slave_config_t       * slave_config_ ;
    /// Slave state handle to check if slave is online and it's state machine(INIT/PREOP/SAFEOP/0P)
    ec_slave_config_state_t  slave_config_state_ ;

    /// PDO domain for data exchange
    uint8_t                * slave_pdo_domain_ ;

    /// Variable for checking motor state 
    int32_t                  motor_state_ ;
    /// Holds alias,position,vendor_id,product_code,al_state information for the slave.
    ec_slave_info_t         slave_info_;
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

};// EthercatSlave class