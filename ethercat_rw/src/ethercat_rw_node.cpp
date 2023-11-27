/*

	This program was provided by Altinay Robotics Technologies to use Maxon EPOS4 COMPACT 50/15 ETHERCAT servo drive with ethercat.
	As the Ethercat Master, EtherLab's Igh Ethercat Master is used on a Raspberry Pi 4 Model B 4GB RAM model with
	5.4.224-rt80 PREEMPT_RT real time kernel. The program was tested with a  220W EC MOTOR 607946 (90mm flat brushless) servo motor.

	Compile with: gcc servo_pos.c -l ethercat -lm -o servo_pos

	Then run servo file with: taskset -c 3,7 sudo chrt -f 90 ./servo_pos 3000

	Date: 27/02/2023

*/

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sched.h> /* sched_setscheduler() */
#include <linux/version.h>
#include <linux/module.h>
#include <limits.h>
#include "ecrt.h"
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <termio.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

// #define TASK_FREQUENCY 100
// #define TIMEOUT_CLEAR_ERROR (1*TASK_FREQUENCY)
#define OPERATION_MODE 9
#define PI 3.14159265358979323

/****************************************************************************/

// Application parameters
#define FREQUENCY 500
#define CLOCK_MONOTONIC 1
#define TIMER_ABSTIME 1
#define CLOCK_TO_USE CLOCK_MONOTONIC
//#define CLOCK_TO_USE CLOCK_REALTIME

//#define MEASURE_TIMING 1

/****************************************************************************/
#define NSEC_PER_SEC (1000000000L)
#define PERIOD_NS (NSEC_PER_SEC / FREQUENCY)

#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
        (B).tv_nsec - (A).tv_nsec)

#define TIMESPEC2NS(T) ((uint64_t) (T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)

/****************************************************************************/

/* -----------------------------------------------------------------------------

	Create master and the domain objects

--------------------------------------------------------------------------------*/
static ec_master_t *master = NULL;
static ec_master_state_t master_state = {};

static ec_domain_t *domain1 = NULL;
static ec_domain_state_t domain1_state = {};

static ec_slave_config_t *sc_SMB;
static ec_slave_config_state_t sc_SMB_state = {};

/* -----------------------------------------------------------------------------

	Create global counters.

--------------------------------------------------------------------------------*/

int counter1 = 0;
double counter2 = 0;
static int run1 = 1;

int32_t i_target_velocity = 0;
static uint8_t *domain1_pd = NULL;
static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int sync_ref_counter = 0;
const struct timespec cycletime = {0, PERIOD_NS};

ros::Publisher jointStatePublisher;

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

extern int clock_nanosleep(clockid_t __clock_id, int __flags,
			   __const struct timespec *__req,
			   struct timespec *__rem);

/* Latency trick
 * if the file /dev/cpu_dma_latency exists,
 * open it and write a zero into it. This will tell 
 * the power management system not to transition to 
 * a high cstate (in fact, the system acts like idle=poll)
 * When the fd to /dev/cpu_dma_latency is closed, the behavior
 * goes back to the system default.
 * 
 * Documentation/power/pm_qos_interface.txt
 */
static void set_latency_target(void)
{
	struct stat s;
	int ret;

	if (stat("/dev/cpu_dma_latency", &s) == 0) {
		latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
		if (latency_target_fd == -1)
			return;
		ret = write(latency_target_fd, &latency_target_value, 4);
		if (ret == 0) {
			printf("# error setting cpu_dma_latency to %d!: %s\n", latency_target_value, strerror(errno));
			close(latency_target_fd);
			return;
		}
		printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
	}
}


//double act_p, act_v;

/* -----------------------------------------------------------------------------

	Define servo drive features.

--------------------------------------------------------------------------------*/

/* Master 0, Slave 0, "EPOS4"
 * Vendor ID:       0x000000fb
 * Product code:    0x65520000
 * Revision number: 0x01500000
*/

#define VendorID 0x000000fb
#define ProductCode 0x65520000

#define SMBSlavePos 0,0
#define SMB VendorID,ProductCode

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

static struct{
	unsigned int ctrl_word;
	unsigned int target_velocity;
	unsigned int velocity_offset;
	unsigned int operation_mode;
	unsigned int digital_output;
	unsigned int status_word;
	unsigned int current_position;
	unsigned int current_velocity;
	unsigned int current_torque;
	unsigned int mode_display;
    unsigned int digital_input;
}offset;

/* -----------------------------------------------------------------------------

	Defining the addresses of the servo-drive

--------------------------------------------------------------------------------*/

const static ec_pdo_entry_reg_t domain1_regs[]={
	{SMBSlavePos, SMB, 0x6040, 0, &offset.ctrl_word},
    {SMBSlavePos, SMB, 0x60FF, 0, &offset.target_velocity},
    {SMBSlavePos, SMB, 0x60B1, 0, &offset.velocity_offset},
    {SMBSlavePos, SMB, 0x6060, 0, &offset.operation_mode},
    {SMBSlavePos, SMB, 0x60FE, 1, &offset.digital_output},
	{SMBSlavePos, SMB, 0x6041, 0, &offset.status_word},
    {SMBSlavePos, SMB, 0x6064, 0, &offset.current_position}, 
	{SMBSlavePos, SMB, 0x606C, 0, &offset.current_velocity},
	{SMBSlavePos, SMB, 0x6077, 0, &offset.current_torque},
    {SMBSlavePos, SMB, 0x6061, 0, &offset.mode_display},
    {SMBSlavePos, SMB, 0x60FD, 0, &offset.digital_input},
	{}
};
/* -----------------------------------------------------------------------------

	Values obtained by ethercat cstruct -p0 command 

--------------------------------------------------------------------------------*/

static ec_pdo_entry_info_t SMB_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x60ff, 0x00, 32},
    {0x60b1, 0x00, 32},
    {0x6060, 0x00, 8},
    {0x60fe, 0x01, 32},
    {0x6041, 0x00, 16},
    {0x6064, 0x00, 32},
    {0x606c, 0x00, 32},
    {0x6077, 0x00, 16},
    {0x6061, 0x00, 8},
    {0x60fd, 0x00, 32},
};

ec_pdo_info_t SMB_pdos[] = {
    {0x1600, 5, SMB_pdo_entries + 0}, /* 1st RxPDO Mapping */
    {0x1a00, 6, SMB_pdo_entries + 5}, /* 1st TxPDO Mapping */
};

static ec_sync_info_t SMB_syncs[]={
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, SMB_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, SMB_pdos + 1, EC_WD_DISABLE},
    {0xff}
};



/* -----------------------------------------------------------------------------

	Function that controls if domain1 is activated.

--------------------------------------------------------------------------------*/

void check_domain1_state(void){
	ec_domain_state_t ds;
	ecrt_domain_state(domain1, &ds);
	if (ds.working_counter != domain1_state.working_counter)
	{
		printf("Domain1: WC %u.\n ", ds.working_counter);
	}
	if (ds.wc_state != domain1_state.wc_state)
	{
		printf("Domain1: State %u.\n",ds.wc_state );
	}
	domain1_state = ds;
}

/* -----------------------------------------------------------------------------

	Function that controls if slave configuration is valid.

--------------------------------------------------------------------------------*/

void check_slave_config_states(void){
	ec_slave_config_state_t s;
	ecrt_slave_config_state(sc_SMB, &s);
	if (s.al_state != sc_SMB_state.al_state)
	{
		printf("SMB: State 0x%02X.\n", s.al_state);
	}
	if (s.online != sc_SMB_state.online)
	{
		printf("SMB: %s.\n", s.online ? "online" : "offline");
	}
	if (s.operational != sc_SMB_state.operational)
	{
		printf("SMB: %soperational.\n", s.operational ? "" : "Not");
	}
	sc_SMB_state=s;
}

/* -----------------------------------------------------------------------------

	Function that controls if the master is activated.

--------------------------------------------------------------------------------*/

void check_master_state(){
	ec_master_state_t ms;
	ecrt_master_state(master, &ms);
	if (ms.slaves_responding != master_state.slaves_responding)
	{
		printf("%u slaves(s).\n",ms.slaves_responding);
	}
	if (ms.al_states != master_state.al_states)
	{
		printf("AL states: 0x%02X.\n", ms.al_states);
	}
	if (ms.link_up != master_state.link_up)
	{
		printf("Link is %s.\n",ms.link_up ? "up" : "down" );
	}
	master_state=ms;
}

/* -----------------------------------------------------------------------------

	Signal handler to break the while cycle in the main function.

--------------------------------------------------------------------------------*/

void signal_handler(int sig)
{
    run1 = 0;
}

/* -----------------------------------------------------------------------------

	Main program

--------------------------------------------------------------------------------*/

	uint16_t command = 0x0000;
	uint16_t ctrl1;
	uint16_t status;
	int8_t opmode;
	int32_t current_velocity;
	int32_t current_position;

void cyclic_task()
{

	static int counter3 = 0;
	static bool flip;
	static bool running=false;
	struct timespec time;
	{
		/* data */
	};
	

	// Read master and domain

	ecrt_master_receive(master);
	ecrt_domain_process(domain1);

	// Control master, domain and slave conditions

	// check_domain1_state();
	// check_master_state();
	// check_slave_config_states();

	// Read the values from the servo-drive

	status=EC_READ_U16(domain1_pd+offset.status_word);
	opmode=EC_READ_U8(domain1_pd+offset.mode_display);
	current_velocity=EC_READ_S32(domain1_pd+offset.current_velocity);
	current_position=EC_READ_S32(domain1_pd+offset.current_position);
	ctrl1 = EC_READ_U16(domain1_pd+offset.ctrl_word);
	
	// Print the values

	
    EC_WRITE_S8(domain1_pd+offset.operation_mode, OPERATION_MODE);

/* -----------------------------------------------------------------------------

	Starting the configuration of the controlword(0x6040h). The values 0x006,
	0x0007 and 0x000f must be written to 0x6040h respectively in order to make 
	the drive work.

--------------------------------------------------------------------------------*/
if (sc_SMB_state.operational && (sc_SMB_state.al_state==0x08) && (domain1_state.working_counter>2) )
	{
	// Read the values from the servo-drive

	status=EC_READ_U16(domain1_pd+offset.status_word);

	if (command == 0x0000 && !(status & 0x0008))
	{
		if  (status & 0x0007) 
		{
		EC_WRITE_U16(domain1_pd+offset.ctrl_word, 0x000f);
		command = 0x0027;
		}
		else
		{
		EC_WRITE_U16(domain1_pd+offset.ctrl_word, 0x0006);
		command=0x0006;
		}
	}
	else if (command == 0x0006 && (status & 0x0001) )
	{
		EC_WRITE_U16(domain1_pd+offset.ctrl_word, 0x0007);
		command = 0x000f;
	}

	else if (command == 0x000f && (status & 0x0003) )
	{
		EC_WRITE_U16(domain1_pd + offset.ctrl_word, 0x000f);
		command = 0x0027;
		
	}

	else if ( command == 0x0027 && (status & 0x0007) )
	{
		EC_WRITE_U16(domain1_pd+offset.ctrl_word, 0x000f);
		EC_WRITE_S32(domain1_pd+offset.target_velocity,i_target_velocity);
		command = 0x0027;
		counter1 += 1;
 		// Create a JointState message.
	    sensor_msgs::JointState jointState;
		// Set the joint names.
		jointState.name = {"j1"};

		// Set the joint positions.
		jointState.position = {current_position};
		jointState.velocity = {current_velocity};

		// Publish the joint states.
		jointStatePublisher.publish(jointState);
/* 	// Control the motor as a sinusoidal wave which increases and decreases the speed between -500, +500 rpm		
		
		if(counter1 % 300 == 0)
		{
			target_velocity = sin(counter2*PI/180.0)*500.0/(sin(PI/2.0));
			counter2++;

		}  
*/
	}


/* -----------------------------------------------------------------------------

	Fault resetting the controlword(0x6040h) in case of any error.

--------------------------------------------------------------------------------*/


	else
	{
		if((status & 0x0008) || !(status & 0x0001)){
			if (flip)
				EC_WRITE_U16(domain1_pd+offset.ctrl_word, 0x0080);
			else
				{
				EC_WRITE_U16(domain1_pd+offset.ctrl_word, 0x0006);
				command=0x0000;
				}
		} 
		if(counter3 % 50 == 0)
		{
			flip = !flip;
		}
		counter3++;
	} 

	}
/* if DC sync 
    if (sync_ref_counter) {
            sync_ref_counter--;
        } else {
            sync_ref_counter = 1; // sync every cycle

            clock_gettime(CLOCK_TO_USE, &time);
            ecrt_master_sync_reference_clock_to(master, TIMESPEC2NS(time));
    }
        ecrt_master_sync_slave_clocks(master);


*/
	// send process data

	ecrt_domain_queue(domain1);
	ecrt_master_send(master);		
}

void setVelocityCallback(const geometry_msgs::Twist& msg){

	i_target_velocity=msg.linear.x*1000;
	ROS_INFO_STREAM("Subscriber velocities:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);

}


int main(int argc, char** argv)
{

	/* if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    } */

	/* use the /dev/cpu_dma_latency trick if it's there */
	set_latency_target();
	// Receive master

	master = ecrt_request_master(0);

	// Handlers for signal.h library to break the while loop at the end of the program.

	signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
	
	// Flag for signal.h library.

	run1 = 1;

	// Check if the master is OK

	if (!master)
	{

		exit(EXIT_FAILURE);
	}

	// Check if the domain is OK

	domain1 = ecrt_master_create_domain(master);
	if (!domain1){
		ecrt_release_master(master);
		exit(EXIT_FAILURE);
	}

	// Check the slave configurations

	if (!(sc_SMB = ecrt_master_slave_config(master,SMBSlavePos, SMB))){
		fprintf(stderr, "Failed to get slave configuration!\n");
		exit(EXIT_FAILURE);
	}
    

	// Check the PDO configuration

	printf("Configuring PDOs...\n");

	if(ecrt_slave_config_pdos(sc_SMB,EC_END, SMB_syncs))
	{
		fprintf(stderr, "Failed to configure slaves!\n");
		exit(EXIT_FAILURE);
	}

	else
	{
		printf("Success to configure slaves!\n");
	}

	// Check the PDO entry registration

	if(ecrt_domain_reg_pdo_entry_list(domain1, domain1_regs))
	{
		fprintf(stderr, "PDO entry registration failed!\n");
		exit(EXIT_FAILURE);
	}

	else
	{
		printf("Success to configuring SMB PDO entry\n");
		printf("operation_mode=%d, ctrl_word=%d, target_velocity=%d. status_word=%d, mode_display=%d, current_velocity=%d\n",offset.operation_mode, offset.ctrl_word, offset.target_velocity, offset.status_word, offset.mode_display, offset.current_velocity );
	}

	// Activate the ethercat master

	printf("Activating master...\n");
	if(ecrt_master_activate(master))
	{
		exit(EXIT_FAILURE);
	}

	else
	{
		printf("Master activated!\n");
	}

	if (!(domain1_pd = ecrt_domain_data(domain1))){
		exit(EXIT_FAILURE);
	}
	
	// Main loop
	struct timespec wakeupTime;
    struct timespec startTime, endTime, lastStartTime = {};
    uint32_t period_ns = 0, exec_ns = 0, latency_ns = 0,
             latency_min_ns = 0, latency_max_ns = 0,
             period_min_ns = 0, period_max_ns = 0,
             exec_min_ns = 0, exec_max_ns = 0;

    struct sched_param param = {};
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);

	ros::init(argc, argv, "set_velocity");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/cmd_vel",100,&setVelocityCallback);
    
	jointStatePublisher = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

	printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

 	printf("Starting cyclic function.\n");

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);


	while(run1  && ros::ok()){
		wakeupTime = timespec_add(wakeupTime, cycletime);
        clock_nanosleep(CLOCK_TO_USE, TIMER_ABSTIME, &wakeupTime, NULL);

        // Write application time to master
        //
        // It is a good idea to use the target time (not the measured time) as
        // application time, because it is more stable.
        //
        ecrt_master_application_time(master, TIMESPEC2NS(wakeupTime));

//#ifdef MEASURE_TIMING
        clock_gettime(CLOCK_TO_USE, &startTime);
        latency_ns = DIFF_NS(wakeupTime, startTime);
        period_ns = DIFF_NS(lastStartTime, startTime);
        exec_ns = DIFF_NS(lastStartTime, endTime);
        lastStartTime = startTime;

        if (latency_ns > latency_max_ns) {
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
//#endif
        double dt = period_ns/1000000000.0; //exec time to second
	    
		if (counter) {
            counter--;
        } else { // do this at 1 Hz
            counter = FREQUENCY;
            // check for master state (optional)
			check_domain1_state();
			check_master_state();
			check_slave_config_states();
            // output timing stats
            printf("period     %10u ... %10u\n",
                    period_min_ns, period_max_ns);
            printf("exec       %10u ... %10u\n",
                    exec_min_ns, exec_max_ns); 
            printf("latency    %10u ... %10u\n",
                    latency_min_ns, latency_max_ns);
			printf("SMB: act vel: %5d target vel: %5d \n",
				current_velocity, i_target_velocity);
			printf("cmd: 0x%04x status: 0x%04x opmode: 0x%02x  act command: 0x%04x \n",
				command, status, opmode, ctrl1);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
        }
		cyclic_task();
		clock_gettime(CLOCK_TO_USE, &endTime);
		ros::spinOnce();
	}
	

	EC_WRITE_S32(domain1_pd+offset.target_velocity,0);
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);
	
	/* close the latency_target_fd if it's open */
	if (latency_target_fd >= 0)
		close(latency_target_fd);

	// wait for last target speed send to slave
    usleep(500000);
	// Deactivating the master

	ecrt_release_master(master);

	printf("\n\n*********************************\n\n");

	printf("The motor has stopped!\n\n");

	printf("*********************************\n\n");

	return EXIT_SUCCESS;
	return 0;
}