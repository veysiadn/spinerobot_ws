#include <iostream>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>       
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include <ecrt.h>           //// IgH EtherCAT library header file, includes all EtherCAT related functions and data types.


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
    int	fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    std::system("sudo chown veysiadn /dev/EtherCAT*");
    if(fd){
        std::cout << "EtherCAT master is not active, activating...." << std::endl;
        std::system("cd ~; ethercatctl start");
        usleep(1e6);
        fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
        if(fd){
            std::cout << "EtherCAT device not found...." << std::endl;
    
	}
    }
    rclcpp::init(argc, argv);
    static ec_master_t *master = NULL;          //master instance for accessing hardware.
    printf("Starting ecat test.\n");

    master = ecrt_request_master(0);            // Requesting master to access all resources and functions. Constructor.

    if (!master) {                                              // Checking if request succesfull
        printf("Failed to get a master instance!\n");
        return EXIT_FAILURE;
    }
/*    else {
        ecrt_release_master(master);            // Releasing master after to finish the communication. destructor
        printf("Master get/release ok.\n");
	}*/
    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;
    usleep(5e6);
  printf("hello world ecatpkg package\n");
  return 0;
}
