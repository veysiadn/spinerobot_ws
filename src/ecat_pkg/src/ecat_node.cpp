#include "ecat_node.hpp"

EthercatNode::EthercatNode() : LifecycleNode("EtherCAT Node")
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

void EthercatNode::SetCustomSlave(EthercatSlave c_slave, int index)
{
    this->slaves_[index] = c_slave ; 
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

int EthercatNode::SetOperationMode(uint8_t om, int index)
{
    if( ecrt_slave_config_sdo8(this->slaves_[index].slave_config_,OD_OPERATION_MODE, om) ){
        
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
        return  -1 ;
    }
    return 0; 
}

int EthercatNode::SetOperationModeAll(uint8_t om)
{
    for(int i = 0; i < g_kNumberOfServoDrivers -1 ; i++)
    {
        if( ecrt_slave_config_sdo8(this->slaves_[i].slave_config_,OD_OPERATION_MODE, om) ){
            RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set operation mode config error ! ");
            return  -1 ;
        }
    }
    return 0 ; 
}

int EthercatNode::SetProfilePositionParameters(ProfilePosParam& P, int index)
{
    //profile velocity
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_PROFILE_VELOCITY, P.profile_vel) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
        return -1;
    }
    //max profile velocity
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile velocity config error !");
        return -1;
    }
    //profile acceleration
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_PROFILE_ACCELERATION, P.profile_acc) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
        return -1;
    }
    //profile deceleration
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_PROFILE_DECELERATION,P.profile_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed ! ");
        return -1;
    }
    // quick stop deceleration 
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_QUICK_STOP_DECELERATION,P.quick_stop_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed !");
        return -1;
    }
    // max following error 
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_MAX_FOLLOWING_ERROR,P.max_fol_err) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max following error failed ! ");
        return -1;
    }   
    return 0;
}

int EthercatNode::SetProfilePositionParametersAll(ProfilePosParam& P)
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++)
    {
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

int EthercatNode::SetProfileVelocityParameters(ProfileVelocityParam& P, int index)
{
    // motionProfileType
    if(ecrt_slave_config_sdo16(this->slaves_[index].slave_config_,OD_MOTION_PROFILE_TYPE, P.motion_profile_type) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile velocity config error ! ");
        return -1;
    }
    //max profile velocity
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_MAX_PROFILE_VELOCITY,P.max_profile_vel) < 0) {
        std::cout << "Set max profile  velocity config error ! " << std::endl;
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set max profile  velocity config error ! ");
        return -1;
    }
    //profile acceleration
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_PROFILE_DECELERATION, P.profile_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile deceleration failed !");
        return -1;
    }
    //profile deceleration
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_PROFILE_ACCELERATION,P.profile_acc) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set profile acceleration failed ! ");
        return -1;
    }
    // quick stop deceleration 
    if(ecrt_slave_config_sdo32(this->slaves_[index].slave_config_,OD_QUICK_STOP_DECELERATION,P.quick_stop_dec) < 0) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set quick stop deceleration failed ! ");
        return -1;
    }
    return 0 ;
}

int EthercatNode::SetProfileVelocityParametersAll(ProfileVelocityParam& P)
{
    for(int i = 0; i < g_kNumberOfServoDrivers ; i++){
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