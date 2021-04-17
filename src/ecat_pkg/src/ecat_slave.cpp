#include "ecat_slave.hpp"

EthercatSlave::EthercatSlave() // : variable {assigned value} for initialization.
{

}

EthercatSlave::~EthercatSlave()
{
    
}

int EthercatSlave::CheckSlaveConfigState()
{
    ec_slave_config_state_t s;
    ecrt_slave_config_state(this->slave_config_, &this->slave_config_state_);
    
    if (this->slave_config_state_.al_state != 0x08) {
        printf(" Slave is not operational AL state is : State 0x%02X.\n", this->slave_config_state_.al_state);
    }
    if (this->slave_config_state_.al_state == 0x08) {
        printf(" Slave is operational : State 0x%02X.\n", this->slave_config_state_.al_state);
        g_slaves_up++ ;
    }
    printf("Slave %d is : %s.\n",this->position_ ,s.online ? "online" : "offline");
    return 0;
}