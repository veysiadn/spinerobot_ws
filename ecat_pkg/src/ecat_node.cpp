#include "ecat_node.hpp"

EthercatNode::EthercatNode() : LifecycleNode("ecat_node")
{
    /// @todo for fault injection probably you'll have to declare parameters here.
    /// this->declare_parameter("")
}

EthercatNode::~EthercatNode()
{

}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatNode::on_configure(const State &)
{
    /// @todo add configuration phase w.r.t lifecycle node requirements.
}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatNode::on_activate(const State &){}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatNode::on_deactivate(const State &){}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatNode::on_cleanup(const State &){}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatNode::on_shutdown(const State &){}
node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatNode::on_error(const State &){}

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
    for(int i = 0 ; i < g_kNumberOfServoDrivers-1 ; i++)
    {
        // In this way first servo connected to master pc will be slave[0], second will be slave[1] and so on.
        this->slaves_[i].position_      = i ; 
        this->slaves_[i].vendor_id_     = 0x0000009a ; // Elmo vendor id;
        this->slaves_[i].product_code_  = 0x00030924 ; // Elmo product code.
    }
        /// This means that this slave will be connected last.It will be at the end of topology.
        /// Connection will be ; master->slave[0]->slave[1]->slave[2]->slave[3]->.....->slave[NUM_OF_SLAVES-1]
        this->slaves_[NUM_OF_SLAVES-1].position_      = NUM_OF_SLAVES-1 ; 
        this->slaves_[NUM_OF_SLAVES-1].vendor_id_     = 0x0000079a ; // EasyCAT vendor id;
        this->slaves_[NUM_OF_SLAVES-1].product_code_  = 0xababa001 ; // EasyCAT product code.  
}

void EthercatNode::SetCustomSlave(EthercatSlave c_slave, int position)
{
    this->slaves_[position] = c_slave ; 
}

int  EthercatNode::ConfigureSlaves()
{
    for(int i = 0 ; i < NUM_OF_SLAVES-1 ; i++ ){
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
    for(int i = 0 ; i < g_kNumberOfServoDrivers - 1 ; i++)
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
    for(int i = 0; i < g_kNumberOfServoDrivers - 1 ; i++){
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
    for(int i = 0 ; i < g_kNumberOfServoDrivers -1 ; i++){    
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
    for(int i=0; i < NUM_OF_SLAVES-1 ; i++)
        if (i==(NUM_OF_SLAVES-1)){
            ecrt_slave_config_dc(this->slaves_[i].slave_config_, 0x0006, this->slaves_[i].cycle_time_, this->slaves_[i].kSync0_shift_, 0, 0);
        }else {
            ecrt_slave_config_dc(this->slaves_[i].slave_config_, 0x0300, this->slaves_[i].cycle_time_, this->slaves_[i].kSync0_shift_, 0, 0);
        }
}

void EthercatNode::ConfigDcSync(uint16_t assign_activate, int position)
{
    return ecrt_slave_config_dc(this->slaves_[position].slave_config_, assign_activate, this->slaves_[position].cycle_time_, this->slaves_[position].kSync0_shift_, 0, 0);
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
    int tryCount=0;
    int printSpeedSet=0;
    int timeOut = 1e4*PERIOD_MS;
    while (g_slaves_up != NUM_OF_SLAVES ){
        if(tryCount < timeOut){
            ecrt_master_receive(g_master);
            ecrt_domain_process(g_master_domain);
            usleep(PERIOD_US);
            if(!printSpeedSet){
                CheckMasterState();
                CheckMasterDomainState();
                this->CheckSlaveConfigurationState();
                printSpeedSet = PERIOD_US ;
            }
            clock_gettime(CLOCK_MONOTONIC, &g_sync_timer);
            ecrt_master_sync_reference_clock_to(g_master, TIMESPEC2NS(g_sync_timer));
            ecrt_master_sync_slave_clocks(g_master);
            ecrt_master_application_time(g_master, TIMESPEC2NS(g_sync_timer));

            ecrt_domain_queue(g_master_domain);                
            ecrt_master_send(g_master);

            tryCount++;
            printSpeedSet--;
        }else {
            std::cout << "Error : Timeout occurred while waiting for OP mode.! " << std::endl;
            return -1;
        }
    return 0;
}