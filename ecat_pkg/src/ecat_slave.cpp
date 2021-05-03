#include "ecat_slave.hpp"

EthercatSlave::EthercatSlave() // : variable {assigned value} for initialization.
{

}

EthercatSlave::~EthercatSlave()
{
    
}

int EthercatSlave::CheckSlaveConfigState()
{
    ecrt_slave_config_state(this->slave_config_, &this->slave_config_state_);
    
    if (this->slave_config_state_.al_state != 0x08) {
        std::cout << " Slave is not operational AL state is : State 0x%02X.\n"
                 << this->slave_config_state_.al_state << std::endl;
    }
    return 0;
}