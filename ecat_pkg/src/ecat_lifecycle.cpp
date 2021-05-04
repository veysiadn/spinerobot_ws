#include <ecat_lifecycle.hpp>

using namespace EthercatLifeCycleNode ; 

EthercatLifeCycle::EthercatLifeCycle(): LifecycleNode("ecat_life_cycle_node")
{
    
    ecat_node_= std::make_unique<EthercatNode>();
    /// @todo for fault injection probably you'll have to declare parameters here.
    /// this->declare_parameter("")
}

EthercatLifeCycle::~EthercatLifeCycle()
{
    ecat_node_.reset();
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_configure(const State &)
{
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuring EtherCAT device...");

    if(InitEthercatCommunication())
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuration phase failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }else{

        received_data_publisher_ = this->create_publisher<ecat_msgs::msg::DataReceived>("Slave_Feedback", 10);
        sent_data_publisher_     = this->create_publisher<ecat_msgs::msg::DataSent>("Master_Commands", 10);
        joystick_subscriber_     = this->create_subscription<sensor_msgs::msg::Joy>("Controller", 10, 
                                    std::bind(&EthercatLifeCycle::HandleCallbacks, this, std::placeholders::_1));
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_activate(const State &)
{
    if(StartEthercatCommunication()){

        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Activation phase failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }else{
        RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Activation complete, real-time communication started.");
        received_data_publisher_->on_activate();
        sent_data_publisher_->on_activate();
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_deactivate(const State &)
{
    received_data_publisher_->on_deactivate();
    sent_data_publisher_->on_deactivate();
    ecat_node_->DeactivateCommunication();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_cleanup(const State &)
{
    ecat_node_.reset();
    received_data_publisher_.reset();
    sent_data_publisher_.reset();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_shutdown(const State &)
{
    ecat_node_->ReleaseMaster();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_error(const State &)
{
    ecat_node_.reset();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EthercatLifeCycle::HandleCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    left_x_axis_  = msg->axes[0];
    left_y_axis_  = msg->axes[1];
    right_x_axis_ = msg->axes[2];
    right_y_axis_ = msg->axes[3];
 /* 
      RCLCPP_INFO(this->get_logger(), "Left B   : %f\n", msg->axes[2]);
      RCLCPP_INFO(this->get_logger(), "Right B  : %f\n", msg->axes[5]);
      RCLCPP_INFO(this->get_logger(), "GButton  : %d\n", msg->buttons[0]);
      RCLCPP_INFO(this->get_logger(), "RButton  : %d\n", msg->buttons[1]);
      RCLCPP_INFO(this->get_logger(), "BButton  : %d\n", msg->buttons[2]);
      RCLCPP_INFO(this->get_logger(), "YButton  : %d\n", msg->buttons[3]);
      RCLCPP_INFO(this->get_logger(), "LfButton : %d\n", msg->buttons[4]);
      RCLCPP_INFO(this->get_logger(), "RgButton : %d\n", msg->buttons[5]);
      RCLCPP_INFO(this->get_logger(), "GButton  : %d\n", msg->buttons[6]);
*/

}

int EthercatLifeCycle::SetComThreadPriorities()
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

int EthercatLifeCycle::InitEthercatCommunication()
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Opening EtherCAT device...");
    if (ecat_node_->OpenEthercatMaster())
    {
        return -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring EtherCAT master...");
    if (ecat_node_->ConfigureMaster())
    {
        return -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Getting connected slave informations...");
    if(ecat_node_->GetNumberOfConnectedSlaves()){
        return -1 ;
    }
    ecat_node_->GetAllSlaveInformation();
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        printf("--------------------Slave Info -------------------------\n"
               " Slave alias         = %d\n "
               "Slave position      = %d\n "
               "Slave vendor_id     = 0x%08x\n "
               "Slave product_code  = 0x%08x\n "
               "Slave name          = %s\n "
               "--------------------EOF %d'th Slave Info ----------------\n ",
                ecat_node_->slaves_[FINAL_SLAVE].slave_info_.alias,
                ecat_node_->slaves_[FINAL_SLAVE].slave_info_.position,
                ecat_node_->slaves_[FINAL_SLAVE].slave_info_.vendor_id,
                ecat_node_->slaves_[FINAL_SLAVE].slave_info_.product_code,
                ecat_node_->slaves_[FINAL_SLAVE].slave_info_.name,i);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring  slaves...");
    if(ecat_node_->ConfigureSlaves())
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Mapping default PDOs...");
    if(ecat_node_->MapDefaultPdos())
    {
        return  -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring DC synchronization...");

    ecat_node_->ConfigDcSyncDefault();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activating master...");

    if(ecat_node_->ActivateMaster())
    {
        return  -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Registering master domain...");

    if (ecat_node_->RegisterDomain())
    {
        return  -1 ;
    }
    if (ecat_node_->WaitForOperationalMode())
    {
        return -1 ;
    }
    if (SetComThreadPriorities())
    {
        return -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initialization succesfull...");
    return 0 ; 
}

int  EthercatLifeCycle::StartEthercatCommunication()
{

    err_= pthread_create(&ethercat_thread_,&ethercat_thread_attr_, &EthercatLifeCycle::PassCycylicExchange,this);
    if(err_)
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error : Couldn't start communication thread.!");
        return -1 ; 
    }
    return 0 ;
}

void EthercatLifeCycle::StartPdoExchange(void *instance)
{
    EthercatNode *my_node = reinterpret_cast<EthercatNode*>(instance);

    int counter = 100;
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
    int status_check_counter = 1000;
    uint8_t r_limit_sw_val = 0 ;
    uint8_t l_limit_sw_val = 0 ;
    while(1){
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

        if (status_check_counter){
            status_check_counter--;
        }else { 
            ecat_node_->CheckMasterState();
            ecat_node_->CheckMasterDomainState();
            ecat_node_->CheckSlaveConfigurationState();
            al_state_ = g_master_state.al_states ; 
            status_check_counter = 1000;
        }

        if (counter){
            counter--;
        }
        else
        {
            // do this at 1 Hz
            counter = 100;
            #if MEASURE_TIMING
                    // output timing stats
                 printf("-----------------------------------------------\n\n");
                    printf("Tperiod   min   : %10u ns  | max : %10u ns\n",
                            period_min_ns, period_max_ns);
                    printf("Texec     min   : %10u ns  | max : %10u ns\n",
                            exec_min_ns, exec_max_ns);
                    printf("Tlatency  min   : %10u ns  | max : %10u ns\n",
                            latency_min_ns, latency_max_ns);
                    printf("Tjitter max     : %10u ns  \n",
                            latency_max_ns-latency_min_ns);
            std::cout <<    "Left Switch   : " << unsigned(received_data_[FINAL_SLAVE].left_limit_switch_val) << std::endl << 
                            "Right Switch  : " << unsigned(received_data_[FINAL_SLAVE].right_limit_switch_val) << std::endl;
                     printf("Tperiod min     : %10u ns  | max : %10u ns\n",
                            period_min_ns, max_period);
                    printf("Texec  min      : %10u ns  | max : %10u ns\n",
                            exec_min_ns, max_exec);
                    printf("Tjitter min     : %10u ns  | max : %10u ns\n",
                            max_latency-latency_min_ns, max_latency);
                    printf("-----------------------------------------------\n\n");
                    period_max_ns = 0;
                    period_min_ns = 0xffffffff;
                    exec_max_ns = 0;
                    exec_min_ns = 0xffffffff;
                    latency_max_ns = 0;
                    latency_min_ns = 0xffffffff;
            #endif

                    // calculate new process data
        }
        UpdateReceivedData();
        
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
    }//while(1)
    return;
}// StartPdoExchange   

void EthercatLifeCycle::UpdateReceivedData()
{
    for(int i = 0; i < g_kNumberOfServoDrivers ; i++){
        received_data_[i].actual_pos  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_pos);
        received_data_[i].actual_vel  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_vel);
        received_data_[i].status_word = EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.status_word);
    }
    received_data_[FINAL_SLAVE].right_limit_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.r_limit_switch);
    received_data_[FINAL_SLAVE].left_limit_switch_val  = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.l_limit_switch);
}

int EthercatLifeCycle::PublishAllData()
{   
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        received_data_publisher_->publish(received_data_[i]);
        sent_data_publisher_->publish(sent_data_[i]);
    }
}

void *EthercatLifeCycle::PassCycylicExchange(void *arg)
{
    static_cast<EthercatLifeCycle*>(arg)->StartPdoExchange(arg);
    
}

int EthercatLifeCycle::GetComState()
{
    return al_state_ ; 
}


