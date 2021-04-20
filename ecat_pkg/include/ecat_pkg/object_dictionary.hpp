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
 *******************************************************************************/
/*******************************************************************************
 * \file  object_dictionary.hpp
 * \brief Contains object dictionary and state machine definitions based on CIA402 for Elmo Gold Solo Twitter.
 * 
 *  @todo Seperete state machine from object dictionary as state machine header file.
 *******************************************************************************/

#pragma once

// Object Dictionary of Elmo GOLD Solo Twitter Starts.
#define OD_CONTROL_WORD            0x6040,0x00
#define OD_STATUS_WORD             0x6041,0x00

#define OD_OPERATION_MODE           0x6060,0x00
#define OD_OPERATION_MODE_DISPLAY   0x6061,0x00

#define OD_TARGET_POSITION        	0x607A,0x00
#define OD_POSITION_ACTUAL_VAL    	0x6064,0x00
#define OD_POSITION_DEMAND        	0x6062,0x00
#define OD_POSITON_FOLLOWING_ERROR 	0x60F4,0X00
#define OD_POSITONCOUNTS		 	0x6063,0x00
#define OD_MAX_FOLLOWING_ERROR     	0x6065,0x00

#define OD_VELOCITY_ACTUAL_VALUE   0x606C,0x00
#define OD_VELOCITY_ADDRESS        0x6069,0x00
#define OD_TARGET_VELOCITY         0x60FF,0x00
#define	OD_VELOCITY_OFFSET		   0x60B1,0x00

#define OD_PROFILE_VELOCITY                     0x6081,0x00
#define OD_MAX_PROFILE_VELOCITY                 0x6082,0x00
#define OD_PROFILE_ACCELERATION                 0x6083,0x00
#define OD_PROFILE_DECELERATION                 0x6084,0x00
#define OD_QUICK_STOP_DECELERATION              0x6085,0x00
#define OD_MOTION_PROFILE_TYPE                  0x6086,0x00
#define OD_LINEAR_RAMP_TRAPEZOIDAL              0x00,0x00
#define OD_VELOCITY_ENCODER_RESOLUTION_NUM      0x6094,0x01
#define OD_VELOCITY_ENCODER_RESOLUTION_DEN      0x6094,0x02

#define OD_DIGITAL_INPUTS			  0x60FD,0x00
#define OD_DIGITAL_OUTPUTS 			  0x60FE,0x01

#define OD_DC_CIRCUIT_LINK_VOLTAGE	   0x6079,0x00
#define OD_TARGET_TORQUE               0x6071,0x00 
#define OD_TORQUE_MAX                  0x6072,0x00
#define OD_TORQUE_ACTUAL_VALUE		   0x6077,0x00
#define OD_TORQUE_OFFSET               0x60b2,0x00

#define OD_MAX_CURRENT				   0x6073, 0x00
#define OD_CURRENT_ACTUAL_VALUE		   0x6078, 0x00
#define OD_ERROR_CODE				   0x603F, 0x00   // 2 bit

#define OD_EXTRA_STATUS_REGISTER	    0x2085, 0x00
#define OD_CHECK_ERROR               	0x1002,0x00
#define OD_QUICK_STOP_MODE           	0x605A,0x00
#define OD_STOP_OPTION_CODE         	0x605D,0x00

// Object Dictionary of Elmo GOLD Solo Twitter Ends.
/*****************************************************************/
// CIA 402 State machine definitions start. 

#define SM_COMM_RESET              	0x81
#define SM_FULL_RESET              	0x82
#define SM_START                  	0x01
#define SM_GO_READY_TO_SWITCH_ON    0x06
#define SM_GO_SWITCH_ON            	0x07
#define SM_GO_ENABLE               	0X0F
#define SM_GO_SWITCH_ON_DISABLE    	0x00
#define SM_RUN                    	0x1F
#define SM_EXPEDITE               	0x3F       //like run, but dont finish actual position profile
#define SM_QUICKSTOP              	0x02 


/* From CiA402,  - State coding
	Statusword      |      PDS FSA state
xxxx xxxx x0xx 0000 | Not ready to switch on
xxxx xxxx x1xx 0000 | Switch on disabled
xxxx xxxx x01x 0001 | Ready to switch on
xxxx xxxx x01x 0011 | Switched on
xxxx xxxx x01x 0111 | Operation enabled
xxxx xxxx x00x 0111 | Quick stop active
xxxx xxxx x0xx 1111 | Fault reaction active
xxxx xxxx x0xx 1000 | Fault
*/
#define SM_FSAFROMSTATUSWORD(SW) 		(SW & 0x006f)
#define SM_NOT_READY_TO_SWITCH_ON   	0b00000000
#define SM_NOT_READY_TO_SWITCH_ON_2  	0b00100000
#define SM_SWITCH_ON_DISABLED     		0b01000000
#define SM_SWITCH_ON_DISABLED_2    		0b01100000
#define SM_READY_TO_SWITCH_ON      		0b00100001
#define SM_SWITCHED_ON           		0b00100011
#define SM_OPERATION_ENABLED     		0b00100111
#define SM_QUICK_STOP_ACTIVE      		0b00000111
#define SM_FAULT_REACTION_ACTIVE  		0b00001111
#define SM_FAULTREACTIONACTIVE2 		0b00101111
#define SM_FAULT                		0b00001000
#define SM_FAULT2               		0b00101000

// SatusWord bits :
#define SM_SW_READY_TO_SWITCH_ON    	0x0001
#define SM_SW_SWITCHED_ON          		0x0002
#define SM_SW_OPERATION_ENABLED    		0x0004
#define SM_SW_FAULT               		0x0008
#define SM_SW_VOLTAGE_ENABLED     		0x0010
#define SM_SW_QUICK_STOP           		0x0020
#define SM_SW_SWITCH_ON_DISABLED    	0x0040
#define SM_SW_WARNING             		0x0080
#define SM_SW_REMOTE              		0x0200
#define SM_SW_TARGET_REACHED       		0x0400
#define SM_SW_INTERNAL_LIMIT_ACTIVE 	0x0800

// ControlWord bits :
#define SM_CW_SWITCH_ON        	0x0001
#define SM_CW_ENABLE_VOLTAGE   	0x0002
#define SM_CW_QUICK_STOP       	0x0004
#define SM_CW_ENABLE_OPERATION 	0x0008
#define SM_CW_FAULT_RESET      	0x0080
#define SM_CW_OD_HALT         	0x0100

/* CiA402 statemachine definition end */
/*****************************************************************/
