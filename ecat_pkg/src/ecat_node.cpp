#include "ecat_node.hpp"
using namespace EthercatCommunication ; 

EthercatNode::EthercatNode()
{
    /// @todo for fault injection probably you'll have to declare parameters here.
    /// this->declare_parameter("")
}

EthercatNode::~EthercatNode()
{

}

int  EthercatNode::ConfigureMaster()
{
    g_master = ecrt_request_master(0);    
    if (!g_master) {
        
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Requesting master instance failed ! ");
        return -1 ;
    }

    g_master_domain = ecrt_master_create_domain(g_master);
    if(!g_master_domain) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to create master domain ! ");
        return -1 ;
    }
    return 0 ;
}

void EthercatNode::DefineDefaultSlaves()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++)
    {
        // In this way first servo connected to master pc will be slave[0], second will be slave[1] and so on.
        this->slaves_[i].position_      = i ; 
        this->slaves_[i].vendor_id_     = 0x0000009a ; // Elmo vendor id;
        this->slaves_[i].product_code_  = 0x00030924 ; // Elmo product code.
    }
        /// This means that this slave will be connected last.It will be at the end of topology.
        /// Connection will be ; master->slave[0]->slave[1]->slave[2]->slave[3]->.....->slave[NUM_OF_SLAVES-1]
        this->slaves_[NUM_OF_SLAVES-1].position_      = NUM_OF_SLAVES-1 ; 
        this->slaves_[NUM_OF_SLAVES-1].vendor_id_     = 0x00830518 ; // Custom EasyCAT vendor id;
        this->slaves_[NUM_OF_SLAVES-1].product_code_  = 0x02021053 ; // Custom EasyCAT product code.  
}

void EthercatNode::SetCustomSlave(EthercatSlave c_slave, int position)
{
    this->slaves_[position] = c_slave ; 
}

int  EthercatNode::ConfigureSlaves()
{
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++ ){
        this->slaves_[i].slave_config_ = ecrt_master_slave_config(g_master,this->slaves_[i].kAlias_,
                                        this->slaves_[i].position_,this->slaves_[i].vendor_id_,
                                        this->slaves_[i].product_code_); 

        if(!this->slaves_[i].slave_config_) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to  configure slave ! ");
            return -1;
        }
    }

    return 0 ;
}

int EthercatNode::SetProfilePositionParameters(ProfilePosParam& P, int position)
{   
  // Operation mode to ProfilePositionMode for slave on that position.
    if( ecrt_slave_config_sdo8(this->slaves_[position].slave_config_,OD_OPERATION_MODE, kProfilePosition) ){
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
        return  -1 ;
    }
    //profile velocity
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_PROFILE_VELOCITY, P.profile_vel) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
        return -1;
    }
    //max profile velocity
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity config error !");
        return -1;
    }
    //profile acceleration
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_PROFILE_ACCELERATION, P.profile_acc) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
        return -1;
    }
    //profile deceleration
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_PROFILE_DECELERATION,P.profile_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
        return -1;
    }
    // quick stop deceleration 
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_QUICK_STOP_DECELERATION,P.quick_stop_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
        return -1;
    }
    // max following error 
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_MAX_FOLLOWING_ERROR,P.max_fol_err) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max following error failed ! ");
        return -1;
    }   
    return 0;
}

int EthercatNode::SetProfilePositionParametersAll(ProfilePosParam& P)
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++)
    {
        // Set operation mode to ProfilePositionMode for all motors.
        if( ecrt_slave_config_sdo8(this->slaves_[i].slave_config_,OD_OPERATION_MODE, kProfilePosition) ){
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
            return  -1 ;
        }
        //profile velocity
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_PROFILE_VELOCITY, P.profile_vel) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity failed ! ");
            return -1;
        }
        //max profile velocity
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity failed ! ");
            return -1;
        }
        //profile acceleration
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_PROFILE_ACCELERATION, P.profile_acc) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
            return -1;
        }
        //profile deceleration
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_PROFILE_DECELERATION,P.profile_dec) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
            return -1;
        }
        // quick stop deceleration 
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_QUICK_STOP_DECELERATION,P.quick_stop_dec) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
            return -1;
        }
        // max following error 
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_MAX_FOLLOWING_ERROR,P.max_fol_err) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max foloowing error failed ! ");
            return -1;
        }   
    }
    return 0; 
}

int EthercatNode::SetProfileVelocityParameters(ProfileVelocityParam& P, int position)
{
    // Set operation mode to ProfileVelocityMode for slave on that position.
    if( ecrt_slave_config_sdo8(this->slaves_[position].slave_config_,OD_OPERATION_MODE, kProfileVelocity) ){
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
        return  -1 ;
    }
    // motionProfileType
    if(ecrt_slave_config_sdo16(this->slaves_[position].slave_config_,OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
        return -1;
    }
    //max profile velocity
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
        std::cout << "Set max profile  velocity config error ! " << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile  velocity config error ! ");
        return -1;
    }
    //profile acceleration
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_PROFILE_DECELERATION, P.profile_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed !");
        return -1;
    }
    //profile deceleration
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_PROFILE_ACCELERATION,P.profile_acc) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
        return -1;
    }
    // quick stop deceleration 
    if(ecrt_slave_config_sdo32(this->slaves_[position].slave_config_,OD_QUICK_STOP_DECELERATION,P.quick_stop_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed ! ");
        return -1;
    }
    return 0 ;
}

int EthercatNode::SetProfileVelocityParametersAll(ProfileVelocityParam& P)
{
    for(int i = 0; i < g_kNumberOfServoDrivers ; i++){
        // Set operation mode to ProfileVelocityMode for all motors.
        if( ecrt_slave_config_sdo8(this->slaves_[i].slave_config_,OD_OPERATION_MODE, kProfileVelocity) ){
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
            return  -1 ;
        }
        // motionProfileType
        if(ecrt_slave_config_sdo16(this->slaves_[i].slave_config_,OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
            return -1;
        }
        //max profile velocity
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
            std::cout << "Set max profile  velocity config error ! " << std::endl;
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile  velocity config error ! ");
            return -1;
        }
        //profile acceleration
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_PROFILE_DECELERATION, P.profile_dec) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed !");
            return -1;
        }
        //profile deceleration
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_PROFILE_ACCELERATION,P.profile_acc) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
            return -1;
        }
        // quick stop deceleration 
        if(ecrt_slave_config_sdo32(this->slaves_[i].slave_config_,OD_QUICK_STOP_DECELERATION,P.quick_stop_dec) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed ! ");
            return -1;
        }        
    }
    return 0;
}

int EthercatNode::MapDefaultPdos()
{
    int err ;
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){    
        err = ecrt_slave_config_pdos(this->slaves_[i].slave_config_,EC_END,this->slaves_[i].elmo_syncs);
        if ( err ) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to configure  PDOs for motor!");
            return -1;
        } 
        err = ecrt_domain_reg_pdo_entry_list(g_master_domain, this->slaves_[i].elmo_pdo_regs);
        if ( err ){
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to register PDO entries for motor!");
            return -1;
        }
    }
    err = ecrt_slave_config_pdos(this->slaves_[NUM_OF_SLAVES-1].slave_config_,EC_END,this->slaves_[NUM_OF_SLAVES-1].easycat_slave_syncs);
    if ( err ) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to configure  PDOs for EasyCAT!");
        return -1;
    }
    err = ecrt_domain_reg_pdo_entry_list(g_master_domain, this->slaves_[NUM_OF_SLAVES-1].easycat_pdo_regs);
    if ( err ){
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to register PDO entries for motor!");
        return -1;
        } 
    return 0;
}

int EthercatNode::MapCustomPdos(ec_sync_info_t *syncs, ec_pdo_entry_reg_t *pdo_entry_reg, int position)
{
        int err = ecrt_slave_config_pdos(this->slaves_[position].slave_config_,EC_END,syncs);
        if ( err ) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to configure  PDOs!  ");
            return -1;
        } 
        err = ecrt_domain_reg_pdo_entry_list(g_master_domain, pdo_entry_reg);
        if ( err ){
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Failed to register PDO entries ");
            return -1;
        }
        return 0;
}


void EthercatNode::ConfigDcSyncDefault()
{
    for(int i=0; i < NUM_OF_SLAVES ; i++){
        ecrt_slave_config_dc(this->slaves_[i].slave_config_, 0X0300, PERIOD_NS, this->slaves_[i].kSync0_shift_, 0, 0);
    }
}

void EthercatNode::ConfigDcSync(uint16_t assign_activate, int position)
{
    return ecrt_slave_config_dc(this->slaves_[position].slave_config_, assign_activate, PERIOD_NS, this->slaves_[position].kSync0_shift_, 0, 0);
}

void EthercatNode::CheckSlaveConfigurationState()
{
    for(int i = 0 ; i < NUM_OF_SLAVES -1 ;i++)
    {
        this->slaves_[i].CheckSlaveConfigState();
    }
}

int EthercatNode::CheckMasterState()
{
    ec_master_state_t ms;
    ecrt_master_state(g_master, &ms);

    if (ms.slaves_responding != g_master_state.slaves_responding){
        printf("%u slave(s).\n", ms.slaves_responding);
        if (ms.slaves_responding < 1) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Connection error,no response from slaves.");
            return -1;
        }
    }
    if (ms.al_states != g_master_state.al_states){
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != g_master_state.link_up){
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
        if(!ms.link_up){ 
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Master state link down");
            return -1;
        }
    }
    g_master_state = ms;
    return 0;
}

void EthercatNode::CheckMasterDomainState()
{
    ec_domain_state_t ds;                     //Domain instance
    ecrt_domain_state(g_master_domain, &ds);

    if (ds.working_counter != g_master_domain_state.working_counter)
        printf("masterDomain: WC %u.\n", ds.working_counter);
    if (ds.wc_state != g_master_domain_state.wc_state)
        printf("masterDomain: State %u.\n", ds.wc_state);
    if(g_master_domain_state.wc_state == EC_WC_COMPLETE){
        printf("All slaves configured...\n");
        g_master_domain_state = ds;
    }
    g_master_domain_state = ds;
}

int EthercatNode::ActivateMaster()
{   
    if ( ecrt_master_activate(g_master) ) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Master activation error ! ");
        return -1 ;
    }
    return 0 ; 
}

int EthercatNode::RegisterDomain()
{
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        if(!(this->slaves_[i].slave_pdo_domain_ = ecrt_domain_data(g_master_domain)))
        {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Domain PDO registration error");
            return -1;
        }
    }
    return 0;
}

int EthercatNode::WaitForOperationalMode()
{
    int try_counter=0;
    int check_state_count=0;
    int time_out = 1e4*PERIOD_MS;
    while (g_slaves_up != NUM_OF_SLAVES ){
        if(try_counter < time_out){
            ecrt_master_receive(g_master);
            ecrt_domain_process(g_master_domain);
            usleep(PERIOD_US);
            if(!check_state_count){
                CheckMasterState();
                CheckMasterDomainState();
                this->CheckSlaveConfigurationState();
                check_state_count = PERIOD_US ;
            }
            clock_gettime(CLOCK_MONOTONIC, &g_sync_timer);
            ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(g_sync_timer));
            ecrt_master_sync_slave_clocks(g_master);
            ecrt_master_application_time(g_master, TIMESPEC2NS(g_sync_timer));

            ecrt_domain_queue(g_master_domain);                
            ecrt_master_send(g_master);

            try_counter++;
            check_state_count--;
        }else {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Time out occurred while waiting for OP mode.!  ");
            return -1;
        }
    return 0;
    }
}

int EthercatNode::SetComThreadPriorities()
{
    ethercat_sched_param_.sched_priority = 98;
    printf("Using priority %i\n.", ethercat_sched_param_.sched_priority);

    if (sched_setscheduler(0, SCHED_FIFO, &ethercat_sched_param_) == -1){
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set scheduler failed. ! ");
        return -1 ; 
    }    
    err_ = pthread_attr_init(&ethercat_thread_attr_);
    if (err_) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error initializing thread attribute  ! ");
        return -1;
    }
    /* Set a specific stack size  */
    err_ = pthread_attr_setstacksize(&ethercat_thread_attr_, PTHREAD_STACK_MIN);
    if (err_) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error setting thread stack size  ! ");
        return -1 ;
    }

    err_ = pthread_attr_setschedpolicy(&ethercat_thread_attr_, SCHED_FIFO);
    if (err_) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setschedpolicy failed ! ");
        return -1 ;
    }
    err_ = pthread_attr_setschedparam(&ethercat_thread_attr_, &ethercat_sched_param_);
    if (err_) {
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setschedparam failed ! ");
            return -1 ;
    }
    /* Use scheduling parameters of attr */
    err_ = pthread_attr_setinheritsched(&ethercat_thread_attr_, PTHREAD_EXPLICIT_SCHED);
    if (err_) 
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Pthread setinheritsched failed ! ");
        return -1 ;
    }
}

void *EthercatNode::StartPdoExchange(void *arg)
{
int counter = 1000;
struct timespec wake_up_time, time;
        #if MEASURE_TIMING
            struct timespec startTime, endTime, lastStartTime = {};
            uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
            latency_min_ns = 0, latency_max_ns = 0,
            period_min_ns = 0, period_max_ns = 0,
            exec_min_ns = 0, exec_max_ns = 0,
            max_period=0, max_latency=0,max_exec=0;

        #endif

    // get current time
clock_gettime(CLOCK_TO_USE, &wake_up_time);
int begin=10;
uint8_t r_limit_sw_val = 0 ;
uint8_t l_limit_sw_val = 0 ;
    while(1)
 {

    wake_up_time = timespec_add(wake_up_time, g_cycle_time);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);

    // Write application time to master
    //
    // It is a good idea to use the target time (not the measured time) as
    // application time, because it is more stable.
    //
    ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));

        #if MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &startTime);
            latency_ns = DIFF_NS(wake_up_time, startTime);
            period_ns = DIFF_NS(lastStartTime, startTime);
            exec_ns = DIFF_NS(lastStartTime, endTime);
            lastStartTime = startTime;
            if(!begin)
            {
            if(latency_ns > max_latency)        max_latency = latency_ns;
            if(period_ns > max_period)          max_period  = period_ns;
            if(exec_ns > max_exec)              max_exec    = exec_ns;
            }

            if (latency_ns > latency_max_ns)  {
                latency_max_ns = latency_ns;
            }
            if (latency_ns < latency_min_ns) {
                latency_min_ns = latency_ns;
            }
            if (period_ns > period_max_ns) {
                period_max_ns = period_ns;
            }
            if (period_ns < period_min_ns) {
                period_min_ns = period_ns;
            }
            if (exec_ns > exec_max_ns) {
                exec_max_ns = exec_ns;
            }
            if (exec_ns < exec_min_ns) {
                exec_min_ns = exec_ns;
            }
        #endif

            // receive process data
    ecrt_master_receive(g_master);
    ecrt_domain_process(g_master_domain);

    if (counter)
    {
        counter--;
    }
    else
    {
        // do this at 1 Hz
        counter = 100;
        #if MEASURE_TIMING
                // output timing stats
              /* printf("-----------------------------------------------\n");
                printf("Tperiod   min   : %10u ns  | max : %10u ns\n",
                        period_min_ns, period_max_ns);
                printf("Texec     min   : %10u ns  | max : %10u ns\n",
                        exec_min_ns, exec_max_ns);
                printf("Tlatency  min   : %10u ns  | max : %10u ns\n",
                        latency_min_ns, latency_max_ns);
                printf("Tjitter max     : %10u ns  \n",
                        latency_max_ns-latency_min_ns);*/
                printf("Right switch val     = %d\n"
                       "Left switch val      = %d\n",
                        r_limit_sw_val,l_limit_sw_val);
                printf("----------------------------------------");
                printf( "Left X   : %f\n", left_x_axis_);
                printf("Left Y    : %f\n", left_y_axis_);
                printf( "Right X  : %f\n", right_x_axis_);
                printf("Right Y   : %f\n", right_y_axis_);
                printf("----------------------------------------");
               /* printf("Tperiod min     : %10u ns  | max : %10u ns\n",
                        period_min_ns, max_period);
                 printf("Texec  min      : %10u ns  | max : %10u ns\n",
                        exec_min_ns, max_exec);
                 printf("Tjitter min     : %10u ns  | max : %10u ns\n",
                        max_latency-latency_min_ns, max_latency);
                printf("-----------------------------------------------\n");*/
                period_max_ns = 0;
                period_min_ns = 0xffffffff;
                exec_max_ns = 0;
                exec_min_ns = 0xffffffff;
                latency_max_ns = 0;
                latency_min_ns = 0xffffffff;
        #endif

                // calculate new process data
                r_limit_sw_val = EC_READ_U8(slaves_[NUM_OF_SLAVES-1].slave_pdo_domain_ +slaves_[NUM_OF_SLAVES-1].offset_.r_limit_switch);
                l_limit_sw_val = EC_READ_U8(slaves_[NUM_OF_SLAVES-1].slave_pdo_domain_ +slaves_[NUM_OF_SLAVES-1].offset_.r_limit_switch);
    }


            if (g_sync_ref_counter) {
                g_sync_ref_counter--;
            } else {
                g_sync_ref_counter = 1; // sync every cycle

                clock_gettime(CLOCK_TO_USE, &time);
                ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(time));
            }
            ecrt_master_sync_slave_clocks(g_master);

            // send process data
            ecrt_domain_queue(g_master_domain);
            ecrt_master_send(g_master);
            if(begin) begin--;
    #if MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &endTime);
    #endif
 }
    return NULL;
}   

int EthercatNode::OpenEthercatMaster()
{
    this->fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    if(this->fd){
        RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Opening EtherCAT master...");
        std::system("cd ~; sudo ethercatctl start");
        usleep(1e6);
        this->fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
        if(this->fd){
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : EtherCAT device not found.");
            return -1;
            }else {
                return 0 ;
            }
    }
}

int EthercatNode::InitEthercatCommunication()
{
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Opening EtherCAT device...");
    if (this->OpenEthercatMaster())
    {
        return -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuring EtherCAT master...");
    if (this->ConfigureMaster())
    {
        return -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Defining default slaves...");
    this->DefineDefaultSlaves();

    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuring  slaves...");
    if(this->ConfigureSlaves())
    {
        return -1 ;
    }
/*
    ProfileVelocityParam P ;
    
    P.profile_acc=50000 ;
    P.profile_dec=50000 ;
    P.max_profile_vel = 90000 ;
    P.quick_stop_dec = 50000 ;
    P.motion_profile_type = 0 ;
    SetProfileVelocityParametersAll(P);
*/
RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuring DC synchronization...");

    ConfigDcSyncDefault();

RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Activating master...");

    if(ActivateMaster())
    {
        return  -1 ;
    }
RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Registering master domain...");

    if (RegisterDomain())
    {
        return  -1 ;
    }
    if (WaitForOperationalMode())
    {
        return -1 ;
    }
    if (SetComThreadPriorities())
    {
        return -1 ;
    }

    return 0 ; 
}

int  EthercatNode::StartEthercatCommunication()
{

    err_= pthread_create(&ethercat_thread_,&ethercat_thread_attr_, (THREADFUNCPTR)&EthercatNode::StartPdoExchange,NULL);
    if(err_)
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Couldn't start communication thread.!");
        return -1 ; 
    }
    return 0 ;
}

