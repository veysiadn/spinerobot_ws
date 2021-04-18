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
#include<iostream>
/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

// Application parameters
#define FREQUENCY  1000
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

// EtherCAT
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_ana_in = NULL;
static ec_slave_config_state_t sc_ana_in_state = {};

/****************************************************************************/

// process data
static unsigned int counter = 1;
static unsigned int blink = 0;
// process data
static uint8_t *domain1_pd2 = NULL;
static uint64_t *domainInput_pd = NULL;
static uint8_t *domain1_pd = NULL;

#define LAB_1_SlavePos  0, 0

#define LAB_1 0x0000079a, 0xababa001


// offsets for PDO entries
static uint32_t alarmStatus;
static uint32_t temperatureStatus;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

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

/*****************************************************************************/

struct timespec timespec_add(struct timespec time1, struct timespec time2)
{
    struct timespec result;

    if ((time1.tv_nsec + time2.tv_nsec) >= NSEC_PER_SEC) {
        result.tv_sec = time1.tv_sec + time2.tv_sec + 1;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec - NSEC_PER_SEC;
    } else {
        result.tv_sec = time1.tv_sec + time2.tv_sec;
        result.tv_nsec = time1.tv_nsec + time2.tv_nsec;
    }

    return result;
}

/*****************************************************************************/

void check_domain1_state(void)
{
    ec_domain_state_t ds;

    ecrt_domain_state(domain1, &ds);

    if (ds.working_counter != domain1_state.working_counter)
        printf("Domain1: WC %u.\n", ds.working_counter);
    if (ds.wc_state != domain1_state.wc_state)
        printf("Domain1: State %u.\n", ds.wc_state);

    domain1_state = ds;
}

/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding)
        printf("%u slave(s).\n", ms.slaves_responding);
    if (ms.al_states != master_state.al_states)
        printf("AL states: 0x%02X.\n", ms.al_states);
    if (ms.link_up != master_state.link_up)
        printf("Link is %s.\n", ms.link_up ? "up" : "down");

    master_state = ms;
}


void check_slave_config_states(void)
{
    ec_slave_config_state_t s;

    ecrt_slave_config_state(sc_ana_in, &s);

    if (s.al_state != sc_ana_in_state.al_state) {
        printf("AnaIn: State 0x%02X.\n", s.al_state);
    }
    if (s.online != sc_ana_in_state.online) {
        printf("AnaIn: %s.\n", s.online ? "online" : "offline");
    }
    if (s.operational != sc_ana_in_state.operational) {
        printf("AnaIn: %soperational.\n", s.operational ? "" : "Not ");
    }

    sc_ana_in_state = s;
}
/****************************************************************************/
static volatile sig_atomic_t sig = 0;
void signalHandler(int signum)
{
    std::cout << "Interrupt signal (" << signum << ") received" << std::endl;
    sig = 1 ;
}

void *cyclic_task(void *arg)
{
    struct timespec wakeupTime, time;
    int print_MaxMin = 18e5 + 100 ;
    #if MEASURE_TIMING
        struct timespec startTime={}, endTime={}, lastStartTime = {};
        uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
        latency_min_ns = 0, latency_max_ns = 0,
        period_min_ns = 0, period_max_ns = 0,
        exec_min_ns = 0, exec_max_ns = 0,
        max_period=0, max_latency=0, old_latency = 0,
        max_exec=0, min_latency=0xffffffff,
        min_period=0xffffffff, min_exec=0xffffffff;
        int32_t    min_jitter = 0, max_jitter = 0xffffffff, jitter_ns = 0;
    #endif

    // get current time
clock_gettime(CLOCK_TO_USE, &wakeupTime);
int begin=100;
float tempData=0;
unsigned short potVal=0;
    while(1) 
 {
            
    wakeupTime = timespec_add(wakeupTime, cycletime);
    clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

    // Write application time to master
    //
    // It is a good idea to use the target time (not the measured time) as
    // application time, because it is more stable.
    //
    ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

    #if MEASURE_TIMING
       clock_gettime(CLOCK_TO_USE, &startTime);
       latency_ns = DIFF_NS(wakeupTime, startTime);
       period_ns  = DIFF_NS(lastStartTime, startTime);
       exec_ns    = DIFF_NS(lastStartTime, endTime);
       lastStartTime = startTime;
       if(!begin){
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
       }
       old_latency = latency_ns;
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
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // check process data state (optional)
    //check_domain1_state();

        #if MEASURE_TIMING
        // output timing stats
        //  printf("-----------------------------------------------\n");
        /* printf("%10u %10u %10u %10u\n",
                period_ns, exec_ns,latency_ns,jitter_ns); */
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
            return NULL;
        }
        else{
            print_MaxMin--;
        }

        // printf("-----------------------------------------------\n");
        period_max_ns = 0;
        period_min_ns = 0xffffffff;
        exec_max_ns = 0;
        exec_min_ns = 0xffffffff;
        latency_max_ns = 0;
        latency_min_ns = 0xffffffff;
        #endif

                // calculate new process data
                tempData = EC_READ_REAL(domain1_pd2 + temperatureStatus);
    

            // write process data
            //EC_WRITE_U8(domain1_pd + segments, blink ? 0x0c : 0x03);


            if (sync_ref_counter) {
                sync_ref_counter--;
            } else {
                sync_ref_counter = 1; // sync every cycle

                clock_gettime(CLOCK_TO_USE, &time);
                ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
            }
            ecrt_master_sync_slave_clocks(master);

            // send process data
            ecrt_domain_queue(domain1);
            ecrt_master_send(master);
            if(begin) begin--;
    #ifdef MEASURE_TIMING
            clock_gettime(CLOCK_TO_USE, &endTime);
    #endif
 }
    return NULL;
}

/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

int main(int argc, char **argv)
{  
	signal(SIGINT,signalHandler);
    ec_slave_config_t *slave_config2;

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

    if (!(slave_config2 = ecrt_master_slave_config(master,
                    LAB_1_SlavePos, LAB_1))) {  
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    alarmStatus = ecrt_slave_config_reg_pdo_entry(slave_config2,
            0x005, 0x01, domain1, NULL);
    if (alarmStatus < 0)
        return -1;
    
    temperatureStatus = ecrt_slave_config_reg_pdo_entry(slave_config2,
            0x006, 0x01, domain1, NULL);
    if (temperatureStatus < 0)
        return -1;


    // configure SYNC signals for this slave
    ecrt_slave_config_dc(slave_config2, 0x0006, PERIOD_NS, 1000, 0, 0);
    //ecrt_slave_config_dc(slave_config2, 0x0001, PERIOD_NS, 1000, 0, 0);


    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        return -1;

    if(!(domain1_pd2 = ecrt_domain_data(domain1))) 
    return -1;
 

    struct sched_param param = {};
    pthread_t cyclicThread;
    pthread_attr_t attr;
    int err;

    printf("\nStarting cyclic function.\n");
    param.sched_priority = 98;
    printf("Using priority %i\n.", param.sched_priority);

    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
        perror("sched_setscheduler failed\n");

    err = pthread_attr_init(&attr);
    if (err) {
        printf("init pthread attributes failed\n");
        return -1;
    }

    /* Set a specific stack size  */
    err = pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    if (err) {
        printf("pthread setstacksize failed\n");
        return -1 ;
    }

    /* Set scheduler policy and priority of pthread */
    err = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    if (err) {
        printf("pthread setschedpolicy failed\n");
        return -1 ;
    }
    err = pthread_attr_setschedparam(&attr, &param);
    if (err) {
            printf("pthread setschedparam failed\n");
            return -1 ;
    }
    /* Use scheduling parameters of attr */
    err = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    if (err)
    {
        printf("pthread setinheritsched failed\n");
        return -1 ;
    }

    /* Create a pthread with specified attributes */
    err = pthread_create(&cyclicThread, &attr, &cyclic_task, NULL);
    if (err) {
        printf("create pthread failed\n");
        return -1 ;
    }
    err = pthread_join(cyclicThread,NULL);
    printf("Thread returned\n");
    if(err){
        printf("Thread Join failed.\n");
        return -1 ;
    }
    printf("Main returned\n");
        
    return 0;
}

/****************************************************************************/
