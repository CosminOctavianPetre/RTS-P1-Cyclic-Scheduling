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

#include "displayB.h"


//-------------------------------------
//-  Useful functions from displayB.c
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
#define LIGHT_SENSOR_DARK_THRESH    50      // threshold for light values considered as dark

// Speed Control stuff
#define AVG_SPEED                   55.0F   // average speed to maintain

// Cyclic Scheduler stuff
#define NUM_SC                      2       // number of secondary cycles per main cycle
#define SC_TIME                     5       // secondary cycle time length


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
char lamps_is_active = 0;
int is_dark = 0;

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
#define REQ_LIT         "LIT: REQ\n"
#define REQ_LAM_SET     "LAM: SET\n"
#define REQ_LAM_CLR     "LAM: CLR\n"

// Answers
#define ANS_ERROR       "MSG: ERR\n"
#define ANS_SPEED       "SPD:%f\n"
#define ANS_SLOPE_FLAT  "SLP:FLAT\n"
#define ANS_SLOPE_DOWN  "SLP:DOWN\n"
#define ANS_SLOPE_UP    "SLP:  UP\n"
#define ANS_GAS_OK      "GAS:  OK\n"
#define ANS_BRK_OK      "BRK:  OK\n"
#define ANS_MIX_OK      "MIX:  OK\n"
#define ANS_LIT         "LIT: %2d%%\n"
#define ANS_LAM_OK      "LAM:  OK\n"


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

    CLEAR_BUFFERS(request, answer);

    if (speed > AVG_SPEED) {
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

    CLEAR_BUFFERS(request, answer);

    if (speed > AVG_SPEED) {
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


/************ Scheduler **************/
void *controller(void *arg)
{
    unsigned char sc = 0;   // secondary cycle
    struct timespec exe_start_time, exe_end_time, delta;
    struct timespec sc_time, wait_time;
    sc_time.tv_sec = SC_TIME;
    
#if defined(ARDUINO)
    // wait for comms to be set up
    // NOTE: QEMU might not be able to talk to Arduino over serial device without this
    sleep(1);
#endif

    // setup initial times
    clock_gettime(CLOCK_REALTIME, &exe_start_time);
    clock_gettime(CLOCK_REALTIME, &mixer_state_change_last_time);

    while(1) {
        // execute tasks
        switch (sc) {
            case 0:
                task_get_light();
                task_set_lamps();
                task_get_speed();
                task_get_slope();
                task_set_mixer();
                break;
            case 1:
                task_get_light();
                task_set_lamps();
                task_set_gas();
                task_set_brake();
                break;
        }
        
        sc = (sc + 1) % NUM_SC;
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
