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
#include <malloc.h>
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


typedef struct {
    int axis_number;
    double current_position;
    double current_velocity;
    double target_position;
    double target_velocity;
    double last_target_position;
    double Kp;
    double Ki;
    double Kff;
    double max_acceleration;
    double max_velocity;
    double profile_time;
	double run_time;
    double integral;
	double output;
    struct timespec previous_time;
    struct timespec start_time;
    int in_range;
} AxisController;

AxisController axis1;

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


double cubic_spline_interpolation(int d, double s, double e, double tt, double t) {

    double a0=s;
    double a1=0; // start vel
    double a2=0; // start acc / 2
    double a3=(20.0*e-20.0*s) / (2.0 * pow(tt,3));
    double a4=(30.0*s-30.0*e) / (2.0 * pow(tt,4));
    double a5=(12.0*e-12.0*s) / (2.0 * pow(tt,5));
    switch (d)
    {
    case 1: 
        return (a1+2.0*a2*t+3.0*a3*t*t+4.0*a4*t*t*t+5.0*a5*t*t*t*t); 
        break;

    case 2: 
        return (2.0*a2+6.0*a3*t+12.0*a4*t*t+20.0*a5*t*t*t); 
        break;

    default:
        return (a0+a1*t+a2*t*t+a3*t*t*t+a4*t*t*t*t+a5*t*t*t*t*t);  
        break;
    }
}

void axis_controller_init(AxisController* controller, int axis_number, double Kp, double Ki, double Kff, double max_acceleration, double max_velocity ,int32_t cur_pos, int32_t cur_vel) {
    controller->axis_number = axis_number;
    controller->current_position = cur_pos;
    controller->current_velocity = cur_vel;
    controller->target_position = 0;
    controller->target_velocity = 0;
    controller->last_target_position = 0;
    controller->Kp = Kp;
    controller->Ki = Ki;
    controller->Kff = Kff;
    controller->max_acceleration = max_acceleration;
    controller->max_velocity = max_velocity;
    controller->profile_time = 0;
    controller->integral = 0;
    clock_gettime(CLOCK_TO_USE, &controller->previous_time);
    clock_gettime(CLOCK_TO_USE, &controller->start_time);
    controller->in_range = 0;
}

void axis_controller_control_loop(AxisController* controller) {

        struct timespec current_time={0,0};
        clock_gettime(CLOCK_TO_USE,&current_time);
        double elapsed_time = DIFF_NS(controller->previous_time , current_time)/(NSEC_PER_SEC*1.0);
        double t = DIFF_NS(controller->start_time , current_time)/(NSEC_PER_SEC*1.0);
        double target_velocity, target_position;

        if (t < controller->profile_time) {
            target_velocity = cubic_spline_interpolation(1, controller->last_target_position, controller->target_position, controller->profile_time, t);
            target_position = cubic_spline_interpolation(0, controller->last_target_position, controller->target_position, controller->profile_time, t);
        }
        else {
            target_position  = controller->target_position;
            target_velocity  = 0;
            controller->last_target_position = controller->current_position;
            if (abs((target_position)  - (controller->current_position)) < 2) controller->in_range = 1;
        }
		controller->run_time =t;
        double position_error = (target_position)  - (controller->current_position);
        double velocity_error = (target_velocity)  - (controller->current_velocity);
        double output = controller->Kp * position_error + controller->Ki * controller->integral + controller->Kff * target_velocity;

        if (fabs(output) > controller->max_velocity) {
            output = (output > 0) ? controller->max_velocity : -controller->max_velocity;
        }
		controller->output = output;
		controller->target_velocity = output;
        controller->integral += position_error * elapsed_time;
        controller->previous_time = current_time;

        //printf("Time: %.2f, Axis: %d, Position: %.2f, Velocity: %.2f, Output: %.2f , Profile: %.2f\n", t, controller->axis_number, controller->current_position, controller->current_velocity, output, controller->profile_time);

        //usleep(100000);  // Sleep for 100 milliseconds between control cycles
}

void axis_controller_set_target_position(AxisController* controller, double target_position) {
    controller->target_position = target_position;
	controller->last_target_position = controller->current_position;
    double time_duration = fmax((15.0 * fabs(target_position - controller->current_position)) / (8.0 * controller->max_velocity), sqrt((10.0 * fabs(target_position - controller->current_position)) / (controller->max_acceleration * sqrt(3.0))));
    controller->profile_time = time_duration;
    clock_gettime(CLOCK_TO_USE, &controller->start_time);
    controller->previous_time = controller->start_time;
	controller->in_range = 0;
}

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
	int32_t target_pos;


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
		counter1 = counter1 + 5;
		if ((! running) && (counter1 > 50)) 
		{
			axis_controller_init(&axis1, 1, 1.2, 0.1, 0.9, 500.0, 1000.0, current_position, current_velocity);
    		axis_controller_set_target_position(&axis1,target_pos);
			running=true;
		}
		else
		{
			axis_controller_control_loop(&axis1);
			i_target_velocity = axis1.output;
			axis1.current_velocity = current_velocity;
			axis1.current_position = current_position;
			if (axis1.in_range==1 && running) run1=false;
		}
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


int main(int argc, char *argv[])
{

	if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall failed");
        return -1;
    }

	/* use the /dev/cpu_dma_latency trick if it's there */
	set_latency_target();
	// Receive master

	master = ecrt_request_master(0);

	// Handlers for signal.h library to break the while loop at the end of the program.

	signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
	
	// Flag for signal.h library.

	if (argc < 1) {

		target_pos=0;
	} 
	else {
		target_pos=atoi(argv[1]);
	}

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

    printf("Using priority %i.", param.sched_priority);
    if (sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
        perror("sched_setscheduler failed");
    }

 	printf("Starting cyclic function.\n");

    // get current time
    clock_gettime(CLOCK_TO_USE, &wakeupTime);


	while(run1){
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
			printf("SMB: act vel: %5d target vel: %5.2f act_pos: %7.2d target_pos: %7.2f \n",
				current_velocity, axis1.target_velocity, current_position, axis1.target_position);
			printf("cmd: 0x%04x status: 0x%04x opmode: 0x%02x  act command: 0x%04x \n",
				command, status, opmode, ctrl1);
			printf("tt: %f t:%f \n",
			   axis1.profile_time, axis1.run_time);
            period_max_ns = 0;
            period_min_ns = 0xffffffff;
            exec_max_ns = 0;
            exec_min_ns = 0xffffffff;
            latency_max_ns = 0;
            latency_min_ns = 0xffffffff;
        }
		cyclic_task();
		clock_gettime(CLOCK_TO_USE, &endTime);
	}
	

	EC_WRITE_S32(domain1_pd+offset.target_velocity,0);
	ecrt_domain_queue(domain1);
	ecrt_master_send(master);
	// wait for last target speed send to slave
    usleep(500000);
	// Deactivating the master

	ecrt_release_master(master);

	// Shell command to stop the motor.
	usleep(500000);
	
	system("ethercat download -t int32 -p 0 0x60ff 00 00");

	printf("\n\n*********************************\n\n");

	printf("The motor has stopped!\n\n");

	printf("*********************************\n\n");

	return EXIT_SUCCESS;
	return 0;
}