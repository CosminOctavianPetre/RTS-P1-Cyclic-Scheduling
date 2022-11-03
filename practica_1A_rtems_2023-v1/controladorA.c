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
double getClock();


//-------------------------------------
//-  Constants
//-------------------------------------
#define MSG_LEN     9
#define SLAVE_ADDR  0x8
#define nsec        1000000000L

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
double mixer_last_state_change_time;


// ------------------------------------
// Macros
// ------------------------------------

// fill given buffers with 0's
// mostly used to clear "request" and "answer" buffers
#define CLEAR_BUFFERS(BUF1, BUF2) \
    memset(BUF1, 0, MSG_LEN+1); \
    memset(BUF2, 0, MSG_LEN+1);


// Converts a timespec to a fractional number of seconds
double timespec_to_double(struct timespec ts)
{
	return (double) (ts.tv_sec + ts.tv_nsec/nsec);
}


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

    strncpy(buffer, "MSG: ERR\n", MSG_LEN);
    return 0;
}


void send_req_b(char * request, char * answer)
{
#if defined(ARDUINO)
    // use UART serial module
    write(fd_serie, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read_msg(fd_serie, answer, MSG_LEN);
#else
    //Use the simulator
    simulator(request, answer);
#endif
}


int task_speed()
{    
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    // request speed
    strcpy(request, "SPD: REQ\n");

    //TODO: refactor
    // send_req_b(request, answer);
#if defined(ARDUINO)
    // use UART serial module
    write(fd_serie, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read_msg(fd_serie, answer, MSG_LEN);
#else
    //Use the simulator
    simulator(request, answer);
#endif

    // display speed
    if (1 == sscanf(answer, "SPD:%f\n", &speed)){
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
    strcpy(request, "SLP: REQ\n");

    //TODO: refactor
    // send_req_b(request, answer);
#if defined(ARDUINO)
    // use UART serial module
    write(fd_serie, request, MSG_LEN);
    nanosleep(&time_msg, NULL);
    read_msg(fd_serie, answer, MSG_LEN);
#else
    //Use the simulator
    simulator(request, answer);
#endif

    // display slope
    if (0 == strcmp(answer, "SLP:DOWN\n")) displaySlope(-1);
    if (0 == strcmp(answer, "SLP:FLAT\n")) displaySlope(0);
    if (0 == strcmp(answer, "SLP:  UP\n")) displaySlope(1);

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
    if (brake_is_active) {
        strcpy(request, "BRK: CLR\n");
        brake_is_active = 0;
    } else {
        strcpy(request, "GAS: SET\n");
        gas_is_active = 1;
    }

end:
    // send request, get answer and display
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
    if (gas_is_active) {
        strcpy(request, "GAS: CLR\n");
        gas_is_active = 0;
    } else {
        strcpy(request, "BRK: SET\n");
        brake_is_active = 1;
    }

end:
    // send request, get answer and display
    send_req_b(request, answer);
    displayBrake(brake_is_active);

    return 0;
}


int task_mixer()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    CLEAR_BUFFERS(request, answer);

    //check mixing time
    if ((getClock() - mixer_last_state_change_time) >= 30.0) {
        switch (mixer_is_active) {
            case 0:
                mixer_is_active = 1;
                strcpy(request, "MIX: SET\n");
                break;
            case 1:
                mixer_is_active = 0;
                strcpy(request, "MIX: CLR\n");
                break;
        }
        mixer_last_state_change_time = getClock();
    }

    send_req_b(request, answer);
    displayMix(mixer_is_active);

    return 0;
}


void *controller(void *arg)
{
    mixer_last_state_change_time = getClock();

    while(1) {
        task_speed();
        task_slope();
        task_gas();
        task_brake();
        task_mixer();
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
    //TODO: check that this is the right device name
    char serial_dev[]="/dev/com1";
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
