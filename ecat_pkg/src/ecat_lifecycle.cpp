#include <ecat_lifecycle.hpp>

using namespace EthercatLifeCycleNode ; 

EthercatLifeCycle::EthercatLifeCycle(): LifecycleNode("ecat_node")
{
    
    ecat_node_= std::make_unique<EthercatNode>();

    received_data_.status_word.resize(g_kNumberOfServoDrivers);
    received_data_.actual_pos.resize(g_kNumberOfServoDrivers);
    received_data_.actual_vel.resize(g_kNumberOfServoDrivers);
    received_data_.actual_tor.resize(g_kNumberOfServoDrivers);
    received_data_.op_mode_display.resize(g_kNumberOfServoDrivers);

    sent_data_.control_word.resize(g_kNumberOfServoDrivers);
    sent_data_.target_pos.resize(g_kNumberOfServoDrivers);
    sent_data_.target_vel.resize(g_kNumberOfServoDrivers);
    sent_data_.target_tor.resize(g_kNumberOfServoDrivers);
}

EthercatLifeCycle::~EthercatLifeCycle()
{
    ecat_node_.reset();
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_configure(const State &)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Configuring EtherCAT device...");

    if(InitEthercatCommunication())
    {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Configuration phase failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }else{
        received_data_publisher_ = this->create_publisher<ecat_msgs::msg::DataReceived>("Slave_Feedback", 10);
        sent_data_publisher_     = this->create_publisher<ecat_msgs::msg::DataSent>("Master_Commands", 10);
        joystick_subscriber_     = this->create_subscription<sensor_msgs::msg::Joy>("Controller", 10, 
                                    std::bind(&EthercatLifeCycle::HandleControlNodeCallbacks, this, std::placeholders::_1));
        gui_subscriber_          = this->create_subscription<std_msgs::msg::UInt8>("gui_buttons", 10, 
                                    std::bind(&EthercatLifeCycle::HandleGuiNodeCallbacks, this, std::placeholders::_1));

        return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_activate(const State &)
{
    if(StartEthercatCommunication()){

        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Activation phase failed");
        return node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }else{
        received_data_publisher_->on_activate();
        sent_data_publisher_->on_activate();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Activation complete, real-time communication started.");
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
    ecat_node_->ShutDownEthercatMaster();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn EthercatLifeCycle::on_error(const State &)
{
    ecat_node_.reset();
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void EthercatLifeCycle::HandleControlNodeCallbacks(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    left_x_axis_  = msg->axes[0];
    left_y_axis_  = msg->axes[1];
    right_x_axis_ = msg->axes[3];
    right_y_axis_ = msg->axes[4];
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

void EthercatLifeCycle::HandleGuiNodeCallbacks(const std_msgs::msg::UInt8::SharedPtr gui_sub)
{
    gui_node_data_ = gui_sub->data;
}

int EthercatLifeCycle::SetComThreadPriorities()
{
    ethercat_sched_param_.sched_priority = 98;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Using priority %i\n.", ethercat_sched_param_.sched_priority);

    if (sched_setscheduler(0, SCHED_FIFO, &ethercat_sched_param_) == -1){
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Set scheduler failed. ! ");
        return -1 ; 
    }    
    err_ = pthread_attr_init(&ethercat_thread_attr_);
    if (err_) {
        RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Error initializing thread attribute  ! ");
        return -1;
    }
    /**********************************************************************************************/
    // This part is for CPU isolation to dedicate one core for EtherCAT communication.
    // for this feature to be active fist you have to modify GRUB_CMDLINE_LINUX_DEFAULT in /etc/default/grub 
    // add isolcpus=3 so after editing it will be ; GRUB_CMDLINE_LINUX_DEFAULT = "quiet splash isolcpus=3" 
    // save and exit, and type sudo update-grub and reboot.
    cpu_set_t mask;
    CPU_ZERO(&mask);
    CPU_SET(3,&mask);

    int result = sched_setaffinity(0,sizeof(mask),&mask);
    /**********************************************************************************************/
    
    /* Set a specific stack size  */
    err_ = pthread_attr_setstacksize(&ethercat_thread_attr_, 4096*64);
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Opening EtherCAT device...\n");
    if (ecat_node_->OpenEthercatMaster())
    {
        return -1 ;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Configuring EtherCAT master...\n");
    if (ecat_node_->ConfigureMaster())
    {
        return -1 ;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Getting connected slave informations...\n");
    if(ecat_node_->GetNumberOfConnectedSlaves()){
        return -1 ;
    }

    ecat_node_->GetAllSlaveInformation();
    for(int i = 0 ; i < NUM_OF_SLAVES ; i++){
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"--------------------Slave Info -------------------------\n"
               "Slave alias         = %d\n "
               "Slave position      = %d\n "
               "Slave vendor_id     = 0x%08x\n "
               "Slave product_code  = 0x%08x\n "
               "Slave name          = %s\n "
               "--------------------EOF %d'th Slave Info ----------------\n ",
                ecat_node_->slaves_[i].slave_info_.alias,
                ecat_node_->slaves_[i].slave_info_.position,
                ecat_node_->slaves_[i].slave_info_.vendor_id,
                ecat_node_->slaves_[i].slave_info_.product_code,
                ecat_node_->slaves_[i].slave_info_.name,i);
    }


    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Configuring  slaves...\n");
    if(ecat_node_->ConfigureSlaves()){
        return -1 ;
    }
    
    ProfileVelocityParam P ;
    
    P.profile_acc=1e6 ;
    P.profile_dec=1e6 ;
    P.max_profile_vel = 1e5 ;
    P.quick_stop_dec = 1e6 ;
    P.motion_profile_type = 0 ;
    ecat_node_->SetProfileVelocityParametersAll(P);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Mapping default PDOs...\n");
    if(ecat_node_->MapDefaultPdos()){
        return  -1 ;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Configuring DC synchronization...\n");
    ecat_node_->ConfigDcSyncDefault();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Activating master...\n");
    if(ecat_node_->ActivateMaster()){
        return  -1 ;
    }

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Registering master domain...\n");
    if (ecat_node_->RegisterDomain()){
        return  -1 ;
    }

    if (ecat_node_->WaitForOperationalMode()){
        return -1 ;
    }

    if (SetComThreadPriorities()){
        return -1 ;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Initialization succesfull...\n");
    
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
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Communication thread called.\n");
    return 0 ;
}

void EthercatLifeCycle::StartPdoExchange(void *instance)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting PDO exchange....\n");
    // Measurement time in minutes, e.g.
    uint32_t measurement_time = 0;
    uint32_t print_max_min = measurement_time * 6000 + 500 ; 
    int counter = 10;
    struct timespec wake_up_time, time;
    #if MEASURE_TIMING
        struct timespec start_time, end_time, last_start_time = {};
        uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
        latency_min_ns = 0xffffffff, latency_max_ns = 0,
        period_min_ns = 0xffffffff, period_max_ns = 0,
        exec_min_ns = 0xffffffff, exec_max_ns = 0,
        max_period=0, max_latency=0,max_exec=0,min_period = 0xffffffff,
        exec_min = 0xffffffff , latency_min = 0xffffffff;
        int32_t jitter = 0 , jitter_min = 0xfffffff, jitter_max = 0, old_latency=0;

    #endif
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Phase 2  PDO exchange....\n");
    // get current time
    clock_gettime(CLOCK_TO_USE, &wake_up_time);
    int begin=5e4;
    int status_check_counter = 1000;
    while(sig){
        wake_up_time = timespec_add(wake_up_time, g_cycle_time);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wake_up_time, NULL);
        ecrt_master_application_time(g_master, TIMESPEC2NS(wake_up_time));
//    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Phase 3  PDO exchange....\n");

        #if MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &start_time);
            old_latency = latency_ns;
            latency_ns = DIFF_NS(wake_up_time, start_time);
            period_ns = DIFF_NS(last_start_time, start_time);
            exec_ns = DIFF_NS(last_start_time, end_time);
            last_start_time = start_time;
            if(!begin)
            {
                jitter = latency_ns - old_latency ;
                if(jitter < 0 ) jitter *=-1; 
                if(jitter > jitter_max)             jitter_max  = jitter ; 
                if(latency_ns > max_latency)        max_latency = latency_ns;
                if(period_ns > max_period)          max_period  = period_ns;
                if(exec_ns > max_exec)              max_exec    = exec_ns;
                if(period_ns < min_period)          min_period  = period_ns;
                if(exec_ns < exec_min)              exec_min    = exec_ns;
                if(latency_ns < latency_min)        latency_min = latency_ns;
                if(jitter < jitter_min)             jitter_min  = jitter;
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
            // Checking master/domain/slaves state every 1sec.
            //  ecat_node_->CheckMasterState();
            //  ecat_node_->CheckMasterDomainState();
           // ecat_node_->CheckSlaveConfigurationState();
            al_state_ = g_master_state.al_states ; 
            status_check_counter = 1000;
        }
  //  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Phase 4  PDO exchange....\n");

        if (counter){
            counter--;
        }
        else
        {
            counter = 10;
            PublishAllData();
            #if MEASURE_TIMING
            // output timing stats
            if(!print_max_min){
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-----------------------------------------------\n\n");
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tperiod   min   : %10u ns  | max : %10u ns\n",
                            period_min_ns, period_max_ns);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Texec     min   : %10u ns  | max : %10u ns\n",
                            exec_min_ns, exec_max_ns);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tlatency  min   : %10u ns  | max : %10u ns\n",
                            latency_min_ns, latency_max_ns);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tjitter max     : %10u ns  \n",
                            latency_max_ns-latency_min_ns);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-----------------------------------------------\n\n");       
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tperiod min     : %10u ns  | max : %10u ns\n",
                            min_period, max_period);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Texec  min      : %10u ns  | max : %10u ns\n",
                            exec_min, max_exec);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Tjitter min     : %10u ns  | max : %10u ns\n",
                            jitter_min, jitter_max);
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"-----------------------------------------------\n\n");
          /*  std::cout <<    "Left Switch   : " << unsigned(received_data_.left_limit_switch_val) << std::endl << 
                            "Right Switch  : " << unsigned(received_data_.right_limit_switch_val) << std::endl;
            std::cout << "Left X Axis    : " << left_x_axis_ << std::endl;
            std::cout << "Right X XAxis  : " << right_x_axis_ << std::endl;*/
            std::cout << "Emergency button  : " << unsigned(gui_node_data_) << std::endl;
                    print_max_min=200;
            }else {
                print_max_min--;
            }
                    period_max_ns = 0;
                    period_min_ns = 0xffffffff;

                    exec_max_ns = 0;
                    exec_min_ns = 0xffffffff;

                    latency_max_ns = 0;
                    latency_min_ns = 0xffffffff;
            #endif
        }

        ReadFromSlaves();
        UpdateMotorState();
        UpdateControlParameters();


        WriteToSlaves();
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
                clock_gettime(CLOCK_TO_USE, &end_time);
        #endif
    }//while(1/sig) //Ctrl+C signal
    ecat_node_->DeactivateCommunication();
    return;
}// StartPdoExchange end

void EthercatLifeCycle::ReadFromSlaves()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        received_data_.actual_pos[i]  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_pos);
        received_data_.actual_vel[i]  = EC_READ_S32(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.actual_vel);
        received_data_.status_word[i] = EC_READ_U16(ecat_node_->slaves_[i].slave_pdo_domain_ +ecat_node_->slaves_[i].offset_.status_word);
    }
    received_data_.com_status = al_state_ ; 
    received_data_.right_limit_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.r_limit_switch);
    received_data_.left_limit_switch_val  = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.l_limit_switch);
    received_data_.emergency_switch_val = EC_READ_U8(ecat_node_->slaves_[FINAL_SLAVE].slave_pdo_domain_ +ecat_node_->slaves_[FINAL_SLAVE].offset_.emergency_switch);
    emergency_status_  = received_data_.emergency_switch_val;
}// ReadFromSlaves end

void EthercatLifeCycle::WriteToSlaves()
{
  //  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing to slaves....\n");
  if(!emergency_status_ || !gui_node_data_){
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel,0);
    }
  }else{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        EC_WRITE_U16(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.control_word,sent_data_.control_word[i]);
        EC_WRITE_S32(ecat_node_->slaves_[i].slave_pdo_domain_ + ecat_node_->slaves_[i].offset_.target_vel,sent_data_.target_vel[i]);
    }
  }
}

int EthercatLifeCycle::PublishAllData()
{   
   // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing all data....\n");
    received_data_.header.stamp = this->now();
    received_data_publisher_->publish(received_data_);
    sent_data_.header.stamp  = this->now();
    sent_data_publisher_->publish(sent_data_);
}

void *EthercatLifeCycle::PassCycylicExchange(void *arg)
{
    static_cast<EthercatLifeCycle*>(arg)->StartPdoExchange(arg);
}

int EthercatLifeCycle::GetComState()
{
    return al_state_ ; 
}

void EthercatLifeCycle::EnableMotors()
{
    //DS402 CANOpen over EtherCAT state machine
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
        if ( (received_data_.status_word[i] & command_) == 0x0040){  
            // If status is "Switch on disabled", \
            change state to "Ready to switch on"
            sent_data_.control_word[i]  = SM_GO_READY_TO_SWITCH_ON;
            command_ = 0x006f;
            motor_state_[i] = kSwitchOnDisabled;
           // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Transiting to -Ready to switch on state...- \n");

        } else if ( (received_data_.status_word[i] & command_) == 0x0021){ // If status is "Ready to switch on", \
                                                        change state to "Switched on"
            sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
            command_ = 0x006f;
            motor_state_[i] = kReadyToSwitchOn;
         //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Transiting to -Switched on state...- \n");        

        } else if ( (received_data_.status_word[i] & command_) == 0x0023){         
            // If status is "Switched on", change state to "Operation enabled"

            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Operation enabled...\n");
            sent_data_.control_word[i]  = SM_GO_ENABLE;
            command_ = 0x006f;
            motor_state_[i] = kSwitchedOn;

        }else if ((received_data_.status_word[i] & command_) == 0X08){             
            //if status is fault, reset fault state.
            command_ = 0X04F;

            sent_data_.control_word[i] = SM_FULL_RESET;
            motor_state_[i] = kFault;

        }
    }
}

void EthercatLifeCycle::UpdateControlParameters() 
{
   // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Updating control parameters....\n");
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        if(motor_state_[i]==kOperationEnabled || motor_state_[i]==kTargetReached || motor_state_[i]==kSwitchedOn){
            if(left_x_axis_ > 1000 || left_x_axis_ < -1000 ){
                sent_data_.target_vel[0] = left_x_axis_  * 3 ;
            }else{
                sent_data_.target_vel[0] = 0;
            }
            if(right_x_axis_ < -1000 || right_x_axis_ > 1000){
                sent_data_.target_vel[1] = right_x_axis_  * 3 ;
            }else{
                sent_data_.target_vel[1] = 0 ;
            }
        }else{
            sent_data_.target_vel[i]=0;
        }
    }

}

void EthercatLifeCycle::UpdateMotorState()
{
    for(int i = 0 ; i < g_kNumberOfServoDrivers ; i++){
        
        if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04F;
                sent_data_.control_word[i] = SM_FULL_RESET;
                motor_state_[i] = kFault;
        }
        if(motor_state_[i]!=kSwitchedOn){
            sent_data_.control_word[i] = SM_GO_READY_TO_SWITCH_ON;
            if ( (received_data_.status_word[i] & command_) == 0x0040){  
                // If status is "Switch on disabled", \
                change state to "Ready to switch on"
                sent_data_.control_word[i]  = SM_GO_READY_TO_SWITCH_ON;
                command_ = 0x006f;
                motor_state_[i] = kSwitchOnDisabled;
            } else if ( (received_data_.status_word[i] & command_) == 0x0021){
                    // If status is "Ready to switch on", \
                        change state to "Switched on"
                sent_data_.control_word[i]  = SM_GO_SWITCH_ON;     
                command_ = 0x006f;
                motor_state_[i] = kReadyToSwitchOn;

            } else if ( (received_data_.status_word[i] & command_) == 0x0023){         
                // If status is "Switched on", change state to "Operation enabled"
                sent_data_.control_word[i]  = SM_GO_ENABLE;
                command_ = 0x006f;
                motor_state_[i] = kSwitchedOn;

            }else if ((received_data_.status_word[i] & command_) == 0X08){             
                //if status is fault, reset fault state.
                command_ = 0X04F;

                sent_data_.control_word[i]  = 0x0080;
                motor_state_[i] = kFault;
            }
        }
    }
    return ;
}