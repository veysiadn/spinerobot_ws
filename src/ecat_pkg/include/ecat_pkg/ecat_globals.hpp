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
 * \file  ecat_globals.hpp
 * \brief Header file for all include statements and global variables for EtherCAT
 *        communication.
 * 
 * This header file contains required include statements for IgH EtherCAT library,
 * global variables (e.g. ethercat master,master_state, domain,domain_state), 
 * structs for PDO offset and recieved data from slaves,
 * Communication period and number of slaves can be specified in here.
 *  @todo Converting period and number of slaves variable as a ros parameter.
 *  ******************************************************************************/
#pragma once

#include <iostream>
#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h> /* sched_setscheduler() */
#include <chrono>
#include <memory>
/****************************************************************************/
// IgH EtherCAT library header file the user-space real-time interface library.
// IgH, EtherCAT related functions and data types.
#include "ecrt.h"  
     
// Object dictionary paramaters PDO index and default values in here.
#include "object_dictionary.hpp"  

/*****************************************************************************/
#define PERIOD_NS        1e6  // EtherCAT communication period.
#define NUM_OF_SLAVES     4   // Number of connected slave to the bus.
const unsigned int g_kNumberOfServoDrivers = 3 ;
const unsigned int g_kNsPerSec = 1e9;  // Nanoseconds per second.
/****************************************************************************/
static ec_master_t        * g_master = NULL ;  // EtherCAT master
static ec_master_state_t    g_master_state = {}; // EtherCAT master state

static ec_domain_t       * g_master_domain = NULL; // Ethercat data passing master domain
static ec_domain_state_t   g_master_domainState = {};   // EtherCAT master domain state

static char                   g_slaves_up = 0 ;  // Number of slaves in op mode.
static struct timespec        g_sync_timer ;     // timer for DC sync .
/****************************************************************************/
#define TEST_BIT(NUM,N)     (NUM &  (1 << N))  // Check specific bit in the data. 0 or 1.
#define SET_BIT(NUM,N)      (NUM |  (1 << N))  // Set(1) specific bit in the data.
#define RESET_BIT(NUM,N)    (NUM & ~(1 << N))  // Reset(0) specific bit in the data

/* Convert timespec struct to nanoseconds */ 
#define TIMESPEC2NS(T)      ((uint64_t) (T).tv_sec * kNsPerSec + (T).tv_nsec) 

/* Using Monotonic system-wide clock.  */
#define CLOCK_TO_USE        CLOCK_MONOTONIC  

#ifdef LOGGING
#ifdef LOGGING_SAMPLING
// Statistics struct for analyzing performance.
typedef struct StatisticsStruct
{
  int       statistics_id;
  uint32_t  period_ns;
  uint32_t  exec_ns;
  uint32_t  latency_ns;
  uint32_t * latency_min_ns;
  uint32_t * latency_max_ns;
  uint32_t * period_min_ns;
  uint32_t * period_max_ns;
  uint32_t * exec_min_ns;
  uint32_t * exec_max_ns;
  struct timespec start_time;
  struct timespec end_time;
  struct timespec last_start_time;
} StatisticsStruct;
#endif
#ifdef LOGGING_NO_SAMPLING

typedef struct StatisticsStruct
{
  int statistics_id;
  uint32_t * period_ns;
  uint32_t * exec_ns;
  uint32_t * latency_ns;
  struct timespec start_time;
  struct timespec end_time;
  struct timespec last_start_time;
} StatisticsStruct;
#endif
#endif
 // Motor operation modes
typedef enum
{
    kProfilePosition = 1,
    kProfileVelocity = 3,
    kProfileTorque   = 4,
    kHoming = 6,
    kInterpolatedPosition = 7,
    kCSP = 8,
    kCSV = 9,
    kCST = 10,
} OpMode ;

// CIA 402 state machine motor states
enum MotorStates{
	kReadyToSwitchOn = 1,
	kSwitchedOn,
	kOperationEnabled,
	kFault,
	kVoltageEnabled,
	kQuickStop,
	kSwitchOnDisabled,
	kWarning,
	kRemote,
	kTargetReached,
	kInternalLimitActivate
};

//offset for PDO entries to register PDOs.
typedef struct
{
    unsigned int target_pos ;
    unsigned int target_vel ;
    unsigned int target_tor ;
    unsigned int max_tor  ;
    unsigned int control_word ;
    unsigned int op_mode ;
    unsigned int profile_acc ;
    unsigned int profile_dec ;
    unsigned int quick_stop_dec ;
    unsigned int profile_vel ;

    unsigned int actual_pos ;
    unsigned int pos_fol_err ;
    unsigned int actual_vel ;
    unsigned int actual_cur ;
    unsigned int actual_tor ;
    unsigned int status_word ;
    unsigned int op_mode_display ;
    unsigned int error_code ;
    unsigned int extra_status_reg ;
} OffsetPDO ;


// Received feedback data from slaves
typedef struct
{
    int32_t   target_pos ;
    int32_t   target_vel ;
    int16_t   target_tor ;
    int16_t   max_tor ;
    uint16_t  control_word ;
    OpMode    op_mode ;
    int32_t   vel_offset ;
    int16_t   tor_offset ;

    int32_t  actual_pos ;
    int32_t  actual_vel ;
    int16_t  actual_cur ;
    int16_t  actual_tor ;
    uint16_t status_word ;
    int8_t   mode_display ;
    /// @todo find better name for limit swithces.
    uint8_t  left_limit_switch_val ;
    uint8_t  right_limit_switch_val ;
}ReceivedData;

// EtherCAT SDO request structure for configuration phase.
typedef struct
{
    ec_sdo_request * profile_acc ;    
    ec_sdo_request * profile_dec ;      
    ec_sdo_request * profile_vel ;  
    ec_sdo_request * quick_stop_dec ;
    ec_sdo_request * motion_profile_type ;
    ec_sdo_request * max_profile_vel ;
    ec_sdo_request * max_fol_err ;
    ec_sdo_request * speed_for_switch_search;
    ec_sdo_request * speed_for_zero_search;
    ec_sdo_request * homing_acc;
    ec_sdo_request * curr_threshold_homing;
    ec_sdo_request * home_offset;
    ec_sdo_request * homing_method;		
} SdoRequest ;


// Parameters that should be specified in position mode.
typedef struct 
{
    uint32_t profile_vel ;
    uint32_t profile_acc ;
    uint32_t profile_dec ;
    uint32_t max_fol_err ;
    uint32_t max_profile_vel ; 
    uint32_t quick_stop_dec ;
    uint16_t motion_profile_type ; 

} ProfilePosParam ;

// Parameters that should be specified in homing mode.
typedef struct
{
	uint32_t	max_fol_err;
	uint32_t	max_profile_vel;
	uint32_t	quick_stop_dec;
	uint32_t	speed_for_switch_search;
	uint32_t	speed_for_zero_search;
	uint32_t	homing_acc;
    // Used when homing by touching mechanical limit and sensing current
	uint16_t	curr_threshold_homing;
    // Amount to move away from the sensed limit	
	int32_t		home_offset;
	int8_t		homing_method;
} HomingParam;

// Parameters that should be specified in velocity mode.
typedef struct
{
    uint32_t	max_profile_vel;
    uint32_t	quick_stop_dec;
    uint32_t	profile_acc;
    uint32_t	profile_dec;
    uint16_t    motion_profile_type;
} ProfileVelocityParam ;


