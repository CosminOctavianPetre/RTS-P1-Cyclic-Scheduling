//-Uncomment to compile with arduino support
//#define ARDUINO

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

#include "displayA.h"


//-------------------------------------
//-  Useful functions from displayA.c
//-------------------------------------
void diffTime(struct timespec end, struct timespec start, struct timespec *diff);
void addTime(struct timespec end, struct timespec start, struct timespec *add);
int compTime(struct timespec t1, struct timespec t2);             


//-------------------------------------
//-  Constants
//-------------------------------------
#define MSG_LEN                     9
#define SLAVE_ADDR                  0x8
#define MIXER_STATE_CHANGE_PERIOD   30      // mixer's state should change every this much

// Cyclic Scheduler stuff
#define NUM_SC                      1       // number of secondary cycles per main cycle
#define SC_TIME                     10      // secondary cycle time length


//-------------------------------------
//-  Global Variables
//-------------------------------------
struct timespec time_msg = {0, 400000000};
int fd_serie = -1;

// Sensors and states
float speed = 0.0F;
char gas_is_active = 0;
char brake_is_active = 0;
char mixer_is_active = 0;

// times
struct timespec mixer_state_change_last_time;


// ------------------------------------
// Messages
// ------------------------------------
// Requests
#define REQ_SPEED       "SPD: REQ\n"
#define REQ_SLOPE       "SLP: REQ\n"
#define REQ_GAS_SET     "GAS: SET\n"
#define REQ_GAS_CLR     "GAS: CLR\n"
#define REQ_BRK_SET     "BRK: SET\n"
#define REQ_BRK_CLR     "BRK: CLR\n"
#define REQ_MIX_SET     "MIX: SET\n"
#define REQ_MIX_CLR     "MIX: CLR\n"

// Answers
#define ANS_ERROR       "MSG: ERR\n"
#define ANS_SPEED       "SPD:%f\n"
#define ANS_SLOPE_FLAT  "SLP:FLAT\n"
#define ANS_SLOPE_DOWN  "SLP:DOWN\n"
#define ANS_SLOPE_UP    "SLP:  UP\n"
#define ANS_GAS_OK      "GAS:  OK\n"
#define ANS_BRK_OK      "BRK:  OK\n"
#define ANS_MIX_OK      "MIX:  OK\n"


// ------------------------------------
// Macros
// ------------------------------------

// fill given buffers with 0's
// mostly used to clear "request" and "answer" buffers
#define CLEAR_BUFFERS(BUF1, BUF2) \
    memset(BUF1, 0, MSG_LEN+1); \
    memset(BUF2, 0, MSG_LEN+1);


// ------------------------------------
// Function Prototypes
// ------------------------------------

/*************** Tasks ***************/

// ------------------------------------
// Task: Read Speed
// ------------------------------------
int task_speed();
// ------------------------------------
// Task: Read Slope
// ------------------------------------
int task_slope();
// ------------------------------------
// Task: On/Off Accelerator
// ------------------------------------
int task_gas();
// ------------------------------------
// Task: On/Off Brake
// ------------------------------------
int task_brake();
// ------------------------------------
// Task: On/Off Mixer
// ------------------------------------
int task_mixer();

/********** Aux functions ************/

// reads a message from UART serial module
// blocking call
int read_msg(int fd, char *buffer, int max_size);
// sends a request and receives a response
// blocking call
void send_req_b(char *request, char *answer);

/************ Scheduler **************/
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
        //TODO: refactor
        //count = (count + 1) % MSG_LEN;
        count = count + 1;
        if (count == MSG_LEN) count = 0;

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
int task_speed()
{    
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    // request speed
    strcpy(request, REQ_SPEED);

    send_req_b(request, answer);

    if (1 == sscanf(answer, ANS_SPEED, &speed)){
        displaySpeed(speed);
    }
    return 0;
}


int task_slope()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    // request slope
    strcpy(request, REQ_SLOPE);
    send_req_b(request, answer);

    // display slope
    if (0 == strcmp(answer, ANS_SLOPE_DOWN)) displaySlope(-1);
    if (0 == strcmp(answer, ANS_SLOPE_FLAT)) displaySlope(0);
    if (0 == strcmp(answer, ANS_SLOPE_UP)) displaySlope(1);

    return 0;
}


int task_gas()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    if (speed > 55)
        goto end;
    // at this point, the wagon should speed up
    /* NOTE: two calls might be needed to speed up completely
    *  one call to turn off the brake, another to turn on the gas */
    if (brake_is_active) {
        strcpy(request, REQ_BRK_CLR);
        brake_is_active = 0;
    } else {
        strcpy(request, REQ_GAS_SET);
        gas_is_active = 1;
    }

end:
    // send request, get answer and display gas
    send_req_b(request, answer);
    displayGas(gas_is_active);

    return 0;
}


int task_brake()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    if (speed <= 55)
        goto end;
    // at this point, the wagon should slow down
    /* NOTE: two calls might be needed to slow down completely
    *  one call to turn off the gas, another to turn on the brake */
    if (gas_is_active) {
        strcpy(request, REQ_GAS_CLR);
        gas_is_active = 0;
    } else {
        strcpy(request, REQ_BRK_SET);
        brake_is_active = 1;
    }

end:
    // send request, get answer and display brake
    send_req_b(request, answer);
    displayBrake(brake_is_active);

    return 0;
}


int task_mixer()
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
        goto end;
    
    switch (mixer_is_active) {
        case 0:
            mixer_is_active = 1;
            strcpy(request, REQ_MIX_SET);
            break;
        case 1:
            mixer_is_active = 0;
            strcpy(request, REQ_MIX_CLR);
            break;
    }
    clock_gettime(CLOCK_REALTIME, &mixer_state_change_last_time);

end:
    // send request, get answer and display mixer
    send_req_b(request, answer);
    displayMix(mixer_is_active);

    return 0;
}


/************ Scheduler **************/
void *controller(void *arg)
{
    struct timespec exe_start_time, exe_end_time, delta;
    struct timespec sc_time, wait_time;
    sc_time.tv_sec = SC_TIME;
    
    // setup initial times
    clock_gettime(CLOCK_REALTIME, &exe_start_time);
    clock_gettime(CLOCK_REALTIME, &mixer_state_change_last_time);

    while(1) {
        // execute tasks
        task_speed();
        task_slope();
        task_gas();
        task_brake();
        task_mixer();

        clock_gettime(CLOCK_REALTIME, &exe_end_time);
        diffTime(exe_end_time, exe_start_time, &delta);

        // panic: execution took too long
        if (compTime(sc_time, delta) < 0)
            exit(-1);

        // wait till next secondary cycle
        diffTime(sc_time, delta, &wait_time);
        nanosleep(&wait_time, NULL);
        addTime(exe_start_time, sc_time, &exe_start_time);
    }
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
