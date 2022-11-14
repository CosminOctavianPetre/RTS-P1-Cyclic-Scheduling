//-Uncomment to compile with arduino support
#define ARDUINO

//-------------------------------------
//-  Include files
//-------------------------------------
#include <termios.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include <sys/errno.h>
#include <sys/stat.h>

#include <rtems.h>
#include <rtems/termiostypes.h>
#include <bsp.h>

#include "displayC.h"


//-------------------------------------
//-  Useful functions from displayC.c
//-------------------------------------
void diffTime(struct timespec end, struct timespec start, struct timespec *diff);
void addTime(struct timespec end, struct timespec start, struct timespec *add);
int compTime(struct timespec t1, struct timespec t2);             


//-------------------------------------
//-  Constants
//-------------------------------------
#define MSG_LEN                         9
#define SLAVE_ADDR                      0x8
#define MIXER_STATE_CHANGE_PERIOD       30      // mixer's state should change every this much
#define LIGHT_SENSOR_DARK_THRESH        50      // threshold for light values considered as dark
#define SPEED_REDUCTION_MIN_DISTANCE    11000   // minimum distance required to be able to stop the wagon safely in braking mode

// Speed Control stuff
#define NORMAL_AVG_SPEED            55.0F   // average speed to maintain in normal mode
#define BRAKING_AVG_SPEED           2.5F    // average speed to maintain in braking mode
#define BRAKING_AVG_SPEED_EXP       4.0F    // average speed to maintain in braking mode

// Cyclic Scheduler stuff
// number of secondary cycles per main cycle
#define NORMAL_NUM_SC               2
#define BRAKING_NUM_SC              6
#define BRAKING_NUM_SC_EXP          6
#define STOP_NUM_SC                 3
// secondary cycle time length
#define NORMAL_SC_TIME              5
#define BRAKING_SC_TIME             5
#define BRAKING_SC_TIME_EXP         5
#define STOP_SC_TIME                5


// ------------------------------------
// Operation Modes
// ------------------------------------
typedef enum{normal_mode, braking_mode, stop_mode} op_mode_t;


//-------------------------------------
//-  Global Variables
//-------------------------------------
struct timespec time_msg = {0, 400000000};
int fd_serie = -1;
op_mode_t op_mode = normal_mode;

// Sensors and states
float speed = 0.0F;
char gas_is_active = 0;
char brake_is_active = 0;
char mixer_is_active = 0;
char lamps_is_active = 0;
int is_dark = 0;
int depo_distance = 90000;
char stopped = 0;

// times
struct timespec mixer_state_change_last_time;


// ------------------------------------
// Messages
// ------------------------------------
// Requests
#define REQ_SPEED           "SPD: REQ\n"
#define REQ_SLOPE           "SLP: REQ\n"
#define REQ_GAS_SET         "GAS: SET\n"
#define REQ_GAS_CLR         "GAS: CLR\n"
#define REQ_BRK_SET         "BRK: SET\n"
#define REQ_BRK_CLR         "BRK: CLR\n"
#define REQ_MIX_SET         "MIX: SET\n"
#define REQ_MIX_CLR         "MIX: CLR\n"
#define REQ_LIT             "LIT: REQ\n"
#define REQ_LAM_SET         "LAM: SET\n"
#define REQ_LAM_CLR         "LAM: CLR\n"
#define REQ_STOPPED         "STP: REQ\n"
#define REQ_DISTANCE        "DS:  REQ\n"

// Answers
#define ANS_ERROR           "MSG: ERR\n"
#define ANS_SPEED           "SPD:%f\n"
#define ANS_SLOPE_FLAT      "SLP:FLAT\n"
#define ANS_SLOPE_DOWN      "SLP:DOWN\n"
#define ANS_SLOPE_UP        "SLP:  UP\n"
#define ANS_GAS_OK          "GAS:  OK\n"
#define ANS_BRK_OK          "BRK:  OK\n"
#define ANS_MIX_OK          "MIX:  OK\n"
#define ANS_LIT             "LIT: %2d%%\n"
#define ANS_LAM_OK          "LAM:  OK\n"
#define ANS_STOPPED_GO      "STP:  GO\n"
#define ANS_STOPPED_STOP    "STP:STOP\n"
#define ANS_DISTANCE        "DS:%5d\n"


// ------------------------------------
// Macros
// ------------------------------------

// fill given buffers with 0's
// mostly used to clear "request" and "answer" buffers
#define CLEAR_BUFFERS(BUF1, BUF2) \
    memset(BUF1, 0, MSG_LEN+1); \
    memset(BUF2, 0, MSG_LEN+1);

// prepare a request buffer, send the request and get the response
#define MAKE_REQUEST(req_buf, ans_buf, REQ) \
    strcpy(req_buf, REQ); \
    send_req_b(req_buf, ans_buf);

// check whether we should change operation mode
#define CHECK_OP_MODE_CHANGE(REQUIRED_COND, TARGET_MODE) \
    if ((REQUIRED_COND)) { \
        op_mode = TARGET_MODE; \
        return; \
    }


// ------------------------------------
// Function Prototypes
// ------------------------------------

/*************** Tasks ***************/

// ------------------------------------
// Task: Read Speed
// ------------------------------------
void task_get_speed();
// ------------------------------------
// Task: Read Slope
// ------------------------------------
void task_get_slope();
// ------------------------------------
// Task: On/Off Accelerator
// ------------------------------------
void task_set_gas();
// ------------------------------------
// Task: On/Off Brake
// ------------------------------------
void task_set_brake();
// ------------------------------------
// Task: On/Off Mixer
// ------------------------------------
void task_set_mixer();
// ------------------------------------
// Task: Read Light Sensor
// ------------------------------------
void task_get_light();
// ------------------------------------
// Task: On/Off Lamps
// ------------------------------------
void task_set_lamps();
// ------------------------------------
// Task: Read Distance
// ------------------------------------
void task_get_distance();
// ------------------------------------
// Task: Read Loading Sensor
// ------------------------------------
void task_get_loading_sensor();

/********** Aux functions ************/

// reads a message from UART serial module
// blocking call
int read_msg(int fd, char *buffer, int max_size);
// sends a request and receives a response
// blocking call
void send_req_b(char *request, char *answer);

/************ Scheduler **************/
void normal_sched();
void braking_sched();
void stop_sched();
void *controller(void *arg);


/********** Aux functions ************/
int read_msg(int fd, char *buffer, int max_size)
{
    char aux_buf[MSG_LEN+1];
    int count = 0;
    char char_aux;

    CLEAR_BUFFERS(aux_buf, buffer);

    while (1) {
        char_aux = 0;
        read(fd_serie, &char_aux, 1);
        // skip if it is not valid character
        if ( (char_aux < 'A' || char_aux > 'Z') && (char_aux < '0' || char_aux > '9') &&
        char_aux != ':'  && char_aux != ' ' && char_aux != '\n' && char_aux != '.' && char_aux != '%' ) {
            continue;
        }
        // store the character
        aux_buf[count] = char_aux;

        // increment count in a circular way
        count = (count + 1) % MSG_LEN;

        // if character is new_line return answer
        if (char_aux == '\n') {
            int first_part_size = strlen(&(aux_buf[count]));
            memcpy(buffer, &(aux_buf[count]), first_part_size);
            memcpy(&(buffer[first_part_size]), aux_buf, count);
            return 0;
        }
    }

    // set error msg as answer
    strncpy(buffer, ANS_ERROR, MSG_LEN);
    return 0;
}


void send_req_b(char *request, char *answer)
{
#if defined(ARDUINO)
    // use UART serial module
    write(fd_serie, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read_msg(fd_serie, answer, MSG_LEN);
#else
    // use the simulator
    simulator(request, answer);
#endif
}


/************** Tasks ****************/
void task_get_speed()
{    
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);
    MAKE_REQUEST(request, answer, REQ_SPEED);

    if ( sscanf(answer, ANS_SPEED, &speed) == 1 )
        displaySpeed(speed);
}


void task_get_slope()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);
    MAKE_REQUEST(request, answer, REQ_SLOPE);

    // display slope
    if ( !strcmp(answer, ANS_SLOPE_DOWN) )
        displaySlope(-1);
    if ( !strcmp(answer, ANS_SLOPE_FLAT) )
        displaySlope(0);
    if ( !strcmp(answer, ANS_SLOPE_UP) )
        displaySlope(1);
}


void task_set_gas()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];
    // in normal mode we should maintain an average speed
    // in braking mode we should slow down until we reach the stopping speed
    float speed_target = (float) (op_mode == normal_mode)*NORMAL_AVG_SPEED + (float) (op_mode == braking_mode)*BRAKING_AVG_SPEED;

    CLEAR_BUFFERS(request, answer);

    if (speed > speed_target) {
        // at this point, the wagon should slow down
        if (gas_is_active) {
            MAKE_REQUEST(request, answer, REQ_GAS_CLR);
            gas_is_active = 0;
        }
    } else {
        // at this point, the wagon should speed up
        if (!gas_is_active) {
            MAKE_REQUEST(request, answer, REQ_GAS_SET);
            gas_is_active = 1;
        }
    }

    displayGas(gas_is_active);
}


void task_set_brake()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];
    // in normal mode we should maintain an average speed
    // in braking mode we should slow down until we reach the stopping speed
    float speed_target = (float) (op_mode == normal_mode)*NORMAL_AVG_SPEED + (float) (op_mode == braking_mode)*BRAKING_AVG_SPEED;

    CLEAR_BUFFERS(request, answer);

    if (speed > speed_target) {
        // at this point, the wagon should slow down
        if (!brake_is_active) {
            MAKE_REQUEST(request, answer, REQ_BRK_SET);
            brake_is_active = 1;
        }
    } else {
        // at this point, the wagon should speed up
        if (brake_is_active) {
            MAKE_REQUEST(request, answer, REQ_BRK_CLR);
            brake_is_active = 0;
        }
    }

    displayBrake(brake_is_active);
}


void task_set_mixer()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];
    struct timespec mixer_state_change_period, time_now, delta;

    mixer_state_change_period.tv_sec = MIXER_STATE_CHANGE_PERIOD;

    CLEAR_BUFFERS(request, answer);

    // get elapsed time since last state change
    clock_gettime(CLOCK_REALTIME, &time_now);
    diffTime(time_now, mixer_state_change_last_time, &delta);

    // check if 30 seconds have passed since the last state change
    if ( compTime(delta, mixer_state_change_period) < 0 )
        return;
    
    switch (mixer_is_active) {
        case 0:
            MAKE_REQUEST(request, answer, REQ_MIX_SET);
            mixer_is_active = 1;
            break;
        case 1:
            MAKE_REQUEST(request, answer, REQ_MIX_CLR);
            mixer_is_active = 0;
            break;
    }
    clock_gettime(CLOCK_REALTIME, &mixer_state_change_last_time);
    displayMix(mixer_is_active);
}


void task_get_light()
{    
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];
    unsigned char light_value;

    CLEAR_BUFFERS(request, answer);
    MAKE_REQUEST(request, answer, REQ_LIT);

    if ( sscanf(answer, ANS_LIT, &light_value) == 1 ) {
        is_dark = light_value < LIGHT_SENSOR_DARK_THRESH;
        displayLightSensor(is_dark);
    }
}


void task_set_lamps()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    // lamps state is already correct: nothing to do
    if (is_dark == lamps_is_active)
        return;

    // change lamps state
    switch (is_dark) {
        case 1:
            MAKE_REQUEST(request, answer, REQ_LAM_SET);
            break;
        case 0:
            MAKE_REQUEST(request, answer, REQ_LAM_CLR);
            break;
    }
    lamps_is_active = is_dark;
    displayLamps(lamps_is_active);
}


void task_get_distance()
{    
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);
    MAKE_REQUEST(request, answer, REQ_DISTANCE);

    if ( sscanf(answer, ANS_DISTANCE, &depo_distance) == 1 )
        displayDistance(depo_distance);
}


void task_get_loading_sensor()
{    
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);
    MAKE_REQUEST(request, answer, REQ_STOPPED);

    // display wagon movement state
    if ( !strcmp(answer, ANS_STOPPED_GO) )
        stopped = 0;
    if ( !strcmp(answer, ANS_STOPPED_STOP) )
        stopped = 1;

    displayStop(stopped);
}


/************ Scheduler **************/
void normal_sched()
{
    unsigned char sc = 0;   // secondary cycle
    struct timespec exe_start_time, exe_end_time, delta;
    struct timespec sc_time, wait_time;
    
    // setup times
    sc_time.tv_sec = NORMAL_SC_TIME;
    clock_gettime(CLOCK_REALTIME, &exe_start_time);
    
    while (1) {
        // execute tasks
        switch (sc) {
            case 0:
                task_get_light();
                task_set_lamps();
                task_get_speed();
                task_get_slope();
                task_get_distance();
                break;
            case 1:
                task_get_light();
                task_set_lamps();
                task_set_gas();
                task_set_brake();
                task_set_mixer();
                break;
        }

        sc = (sc + 1) % NORMAL_NUM_SC;
        clock_gettime(CLOCK_REALTIME, &exe_end_time);
        diffTime(exe_end_time, exe_start_time, &delta);

        // panic: execution took too long
        if (compTime(sc_time, delta) < 0)
            exit(-1);

        // wait till next secondary cycle
        diffTime(sc_time, delta, &wait_time);
        nanosleep(&wait_time, NULL);
        addTime(exe_start_time, sc_time, &exe_start_time);

        CHECK_OP_MODE_CHANGE(depo_distance < SPEED_REDUCTION_MIN_DISTANCE, braking_mode);
    }
}


void braking_sched()
{
    unsigned char sc = 0;   // secondary cycle
    struct timespec exe_start_time, exe_end_time, delta;
    struct timespec sc_time, wait_time;
    
    // setup times
    sc_time.tv_sec = BRAKING_SC_TIME;
    clock_gettime(CLOCK_REALTIME, &exe_start_time);

    // lamps always on
    is_dark = 1;
    
    while (1) {
        // execute tasks
        switch (sc) {
            case 0:
                task_get_speed();
                task_set_gas();
                task_set_brake();
                task_set_mixer();
                task_get_distance();
                break;
            case 1:
                task_get_speed();
                task_set_gas();
                task_set_brake();
                task_get_slope();
                task_set_lamps();
                break;
            case 2:
                task_get_speed();
                task_set_gas();
                task_set_brake();
                task_set_mixer();
                task_get_distance();
                break;
            case 3:
                task_get_speed();
                task_set_gas();
                task_set_brake();
                task_get_slope();
                break;
            case 4:
                task_get_speed();
                task_set_gas();
                task_set_brake();
                task_set_mixer();
                task_get_distance();
                break;
            case 5:
                task_get_speed();
                task_set_gas();
                task_set_brake();
                task_get_slope();
                break;
        }

        sc = (sc + 1) % BRAKING_NUM_SC;
        clock_gettime(CLOCK_REALTIME, &exe_end_time);
        diffTime(exe_end_time, exe_start_time, &delta);

        // panic: execution took too long
        if (compTime(sc_time, delta) < 0)
            exit(-1);

        // wait till next secondary cycle
        diffTime(sc_time, delta, &wait_time);
        nanosleep(&wait_time, NULL);
        addTime(exe_start_time, sc_time, &exe_start_time);

        CHECK_OP_MODE_CHANGE(depo_distance == 0 && speed < 10, stop_mode);
    }
}


void stop_sched()
{
    unsigned char sc = 0;   // secondary cycle
    struct timespec exe_start_time, exe_end_time, delta;
    struct timespec sc_time, wait_time;

    // setup times
    sc_time.tv_sec = STOP_SC_TIME;
    clock_gettime(CLOCK_REALTIME, &exe_start_time);
    
    // lamps always on
    is_dark = 1;

    while (1) {
    // fprintf(stderr, "\n\n\nSTOP MODE REACHED\tSTOP MODE REACHED\tSTOP MODE REACHED\n\n\n");
    // displayStop(1);
        // execute tasks
        switch (sc) {
            case 0:
                task_set_lamps();
                task_get_loading_sensor();
                task_set_mixer();
                break;
            case 1:
                task_set_lamps();
                task_get_loading_sensor();
                break;
            case 2:
                task_set_lamps();
                task_get_loading_sensor();
                break;
        }

        sc = (sc + 1) % STOP_NUM_SC;
        clock_gettime(CLOCK_REALTIME, &exe_end_time);
        diffTime(exe_end_time, exe_start_time, &delta);

        // panic: execution took too long
        if (compTime(sc_time, delta) < 0)
            exit(-1);

        // wait till next secondary cycle
        diffTime(sc_time, delta, &wait_time);
        nanosleep(&wait_time, NULL);
        addTime(exe_start_time, sc_time, &exe_start_time);

        CHECK_OP_MODE_CHANGE(stopped == 0, normal_mode);
    }
}


void *controller(void *arg)
{
#if defined(ARDUINO)
    // wait for comms to be set up
    // NOTE: QEMU can't talk to Arduino over serial device without this
    sleep(1);
#endif

    // setup initial times
    clock_gettime(CLOCK_REALTIME, &mixer_state_change_last_time);

    while (1) {
        switch (op_mode) {
            case normal_mode:
                normal_sched();
                break;
            case braking_mode:
                braking_sched();
                break;
            case stop_mode:
                stop_sched();
                break;
        }
    }

    return (0);
}


rtems_task Init (rtems_task_argument ignored)
{
    pthread_t thread_ctrl;
    sigset_t alarm_sig;
    int i;

    /* Block all real time signals so they can be used for the timers.
       Note: this has to be done in main() before any threads are created
       so they all inherit the same mask. Doing it later is subject to
       race conditions */
    sigemptyset(&alarm_sig);
    for (i = SIGRTMIN; i <= SIGRTMAX; i++) {
        sigaddset(&alarm_sig, i);
    }
    sigprocmask(SIG_BLOCK, &alarm_sig, NULL);

    // init display
    displayInit(SIGRTMAX);

#if defined(ARDUINO)
    /* Open serial port */
    char serial_dev[] = "/dev/com1";
    fd_serie = open(serial_dev, O_RDWR);
    if (fd_serie < 0) {
        printf("open: error opening serial %s\n", serial_dev);
        exit(-1);
    }

    struct termios portSettings;
    speed_t speed = B9600;

    tcgetattr(fd_serie, &portSettings);
    cfsetispeed(&portSettings, speed);
    cfsetospeed(&portSettings, speed);
    cfmakeraw(&portSettings);
    tcsetattr(fd_serie, TCSANOW, &portSettings);
#endif

    /* Create first thread */
    pthread_create(&thread_ctrl, NULL, controller, NULL);
    pthread_join (thread_ctrl, NULL);
    exit(0);
}

#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_MAXIMUM_TASKS 1
#define CONFIGURE_MAXIMUM_SEMAPHORES 10
#define CONFIGURE_MAXIMUM_FILE_DESCRIPTORS 30
#define CONFIGURE_MAXIMUM_DIRVER 10
#define CONFIGURE_MAXIMUM_POSIX_THREADS 2
#define CONFIGURE_MAXIMUM_POSIX_TIMERS 1

#define CONFIGURE_INIT
#include <rtems/confdefs.h>
