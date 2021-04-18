#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <alchemy/task.h>
#include <alchemy/sem.h>
#include <alchemy/mutex.h>
#include <alchemy/timer.h>
#include <pthread.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>       
#include "rclcpp/rclcpp.hpp"
#include <ecrt.h>  

RT_TASK my_task;

static int run = 1;
static volatile sig_atomic_t sig=0;
/****************************************************************************/

// Application parameters
#define FREQUENCY 1000
#define CLOCK_TO_USE CLOCK_MONOTONIC
#define MEASURE_TIMING 1
#define MAX_SAFE_STACK (4096 * 1024)
/****************************************************************************/

#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/
/****************************************************************************/

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

static unsigned int counter = 1;
static unsigned int blink = 0;
// process data
static uint8_t *domain1_pd2 = NULL;
static uint64_t *domainInput_pd = NULL;
static uint8_t *domain1_pd = NULL;
/****************************************************************************/
static uint32_t alarmStatus;
static uint32_t temperatureStatus;
static uint32_t segments;
static uint32_t potentiometer;
static uint32_t swithces;
const struct timespec cycletime = {0, PERIOD_NS};

// process data
#define LAB_1_SlavePos  0, 0
#define LAB_1 0x0000079a, 0xababa001

// offsets for PDO entries
static ec_pdo_entry_reg_t domain1_regs[] = {
    {LAB_1_SlavePos,  LAB_1, 0X0005, 0X01, &alarmStatus},
    {LAB_1_SlavePos,  LAB_1, 0X0006, 0X01, &temperatureStatus},
    {}
};


/* Master 0, Slave 1, "LAB_1"
 * Vendor ID:       0x0000079a
 * Product code:    0xababa001
 * Revision number: 0x00000001
 */

static ec_pdo_entry_info_t slave_1_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Alarm */
    {0x0006, 0x01, 32}, /* Temperature */
};

static ec_pdo_info_t slave_1_pdos[] = {
    {0x1600, 1, slave_1_pdo_entries + 0}, /* Outputs */
    {0x1a00, 1, slave_1_pdo_entries + 1}, /* Inputs */
};

static ec_sync_info_t slave_1_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_1_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_1_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/*****************************************************************************
 * Realtime task
 ****************************************************************************/

void rt_check_domain_state(void)
{
    ec_domain_state_t ds = {};

	ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter) {
        //rt_printf("Domain1: WC %u.\n", ds.working_counter);
    }

    if (ds.wc_state != domain1_state.wc_state) {
        //rt_printf("Domain1: State %u.\n", ds.wc_state);
    }

    domain1_state = ds;
}

/****************************************************************************/

void rt_check_master_state(void)
{
    ec_master_state_t ms;

	ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        //rt_printf("%u slave(s).\n", ms.slaves_responding);
    }

    if (ms.al_states != master_state.al_states) {
        //rt_printf("AL states: 0x%02X.\n", ms.al_states);
    }

    if (ms.link_up != master_state.link_up) {
        //rt_printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/****************************************************************************/
void signalHandler(int num)
{
    sig = 1 ;
    run = 0 ;
}
void my_task_proc(void *arg)
{	
    RTIME wakeupTime=0,time=0;
    int cycle_counter = 0;
    unsigned int blink = 0;
    int print_MaxMin = 3e4  + 100 ;
	int begin=5;
	float tempData=0;
	unsigned short potVal=0;
    #if MEASURE_TIMING
        RTIME startTime=0, endTime=0, lastStartTime=0;
        int32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
        latency_min_ns = 0xffffffff, latency_max_ns = 0,
        period_min_ns = 0xffffffff, period_max_ns = 0,
        exec_min_ns = 0xffffffff, exec_max_ns = 0,
        max_period=0, max_latency=0, old_latency = 0,
        max_exec=0, min_latency=0xffffffff,
        min_period=0xffffffff, min_exec=0xffffffff;
        int32_t    min_jitter = 0xffffffff, max_jitter = 0, jitter_ns = 0;
    #endif

	rt_task_set_periodic(NULL, TM_NOW, PERIOD_NS); // ns
    wakeupTime = rt_timer_read();

	while (run) 
    {
		rt_task_wait_period(NULL);

		cycle_counter++;
		
        #if MEASURE_TIMING
            startTime = rt_timer_read();
            latency_ns = (wakeupTime - startTime);
            period_ns  = (startTime - lastStartTime);
            exec_ns    = (endTime   - lastStartTime);
            lastStartTime = startTime;
            if(!begin)
            {
                if(old_latency != latency_ns)       jitter_ns = old_latency-latency_ns ;
                if(jitter_ns < 0 )                  jitter_ns*=-1;
                if(jitter_ns > max_jitter)          max_jitter  = jitter_ns ;
                if(jitter_ns < min_jitter)          min_jitter  = jitter_ns ;
                if(latency_ns > max_latency)        max_latency = latency_ns;
                if(period_ns > max_period)          max_period  = period_ns;
                if(exec_ns > max_exec)              max_exec    = exec_ns;
                if(latency_ns < min_latency)        min_latency = latency_ns;
                if(period_ns < min_period )         min_period  = period_ns;
                if(exec_ns < min_exec)              min_exec    = exec_ns;
                old_latency = latency_ns;
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
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
        #endif
		// receive EtherCAT frames
		ecrt_master_receive(master);
		ecrt_domain_process(domain1);

		 //rt_check_domain_state();
		 //if(!(cycle_counter % 1000)) 
         //   rt_check_master_state();
	
        #if MEASURE_TIMING
        if(!print_MaxMin | sig){
                printf("%10u\n",
                        max_period);
                printf("%10u\n",
                        max_exec);
                printf("%10u\n",
                        max_latency);
                printf("%10d\n",
                        max_jitter);

                printf("%10u\n",
                        min_period);
                printf("%10u\n",
                        min_exec);
                printf("%10u\n",
                        min_latency);
                printf("%10d\n",
                        min_jitter);
                run = 0 ;
                return ;
            }
            else{
                print_MaxMin--;
            }
        #endif

		if (!(cycle_counter % 200)) {
			blink = !blink;
		}

        tempData = EC_READ_REAL(domain1_pd2 + temperatureStatus);
		potVal   = EC_READ_U8(domain1_pd2 + alarmStatus);

		// send process data
		ecrt_domain_queue(domain1);
		ecrt_master_send(master);
        if(begin) begin--;
        #if MEASURE_TIMING
            endTime = rt_timer_read();
        #endif
	}
    return ;
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/



/****************************************************************************
 * Main function
 ***************************************************************************/

int main(int argc, char *argv[])
{
    
  (void) argc;
  (void) argv;
/*    int	fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
    if(fd){
        std::cout << "EtherCAT master is not active, activating...." << std::endl;
        std::system("cd ~; ethercatctl start");
        usleep(1e6);
        fd = std::system("ls /dev | grep EtherCAT* > /dev/null");
        if(fd){
            printf("EtherCAT device not found....");
        }
    }*/
    ec_slave_config_t *sc;
    int ret;

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    // rt_print_auto_init(1);
    signal(SIGINT, signalHandler);
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

    master = ecrt_request_master(0);
    if (!master)
        return -1;

    domain1 = ecrt_master_create_domain(master);
     if (!domain1)
        return -1;

    if (!(sc = ecrt_master_slave_config(master,
                    LAB_1_SlavePos, LAB_1))) {  
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }
	printf("1...\n");
    alarmStatus = ecrt_slave_config_reg_pdo_entry(sc,
            0x005, 0x01, domain1, NULL);
    if (alarmStatus < 0)
        return -1;
    
    temperatureStatus = ecrt_slave_config_reg_pdo_entry(sc,
            0x006, 0x01, domain1, NULL);
    if (temperatureStatus < 0)
        return -1;
	printf("2...\n");

    // configure SYNC signals for this slave
    ecrt_slave_config_dc(sc, 0x0006, PERIOD_NS, 1000, 0, 0);
    //ecrt_slave_config_dc(slave_config2, 0x0001, PERIOD_NS, 1000, 0, 0);


    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;
    printf("3...\n");
    if(!(domain1_pd2 = ecrt_domain_data(domain1))) 
    return -1;
    printf("4...\n");
    ret = rt_task_create(&my_task, "my_task", 4096*1024, 80, 0);
    printf("5...\n");
    if (ret < 0) {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }
    printf("Starting my_task...\n");
    ret = rt_task_start(&my_task, &my_task_proc, NULL);
    if (ret < 0) {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }
    printf("6...\n");
    rt_task_join(&my_task);
        printf("7...\n");
    while(run)
    	usleep(1e6);
    printf("Deleting realtime task...\n");
    rt_task_delete(&my_task);

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}

/****************************************************************************/
