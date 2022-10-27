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
#define NSEC_PER_SEC  1000000000
#include "displayA.h"


void diffTime(struct timespec end, 
              struct timespec start,
              struct timespec *diff);
void addTime(struct timespec end, 
             struct timespec start,
             struct timespec *add);
int compTime(struct timespec t1, 
             struct timespec t2);             
double getClock();

extern char debug[80];

//-------------------------------------
//-  Constants
//-------------------------------------
#define MSG_LEN 9
#define SLAVE_ADDR 0x8

//-------------------------------------
//-  Global Variables
//-------------------------------------
float speed = 0.0;
struct timespec time_msg = {0,400000000};
int fd_serie = -1;
int accelerator_active = 0;
int brake_active = 0;
struct timespec restingTime, mixingTime;
double idle_time, mixing_time;
int mixing = 0;

/** \fn double timespec_to_double(struct timespec ts)
 *  \brief Converts a timespec to a fractional number of seconds.
*/
double timespec_to_double(struct timespec ts)
{
	return ((double)(ts.tv_sec) + ((double)(ts.tv_nsec) / NSEC_PER_SEC));
}

//-------------------------------------
//-  Function: read_msg
//-------------------------------------
int read_msg(int fd, char *buffer, int max_size)
{
    char aux_buf[MSG_LEN+1];
    int count=0;
    char car_aux;

    //clear buffer and aux_buf
    memset(aux_buf, '\0', MSG_LEN+1);
    memset(buffer, '\0', MSG_LEN+1);

    while (1) {
        car_aux='\0';
        read(fd_serie, &car_aux, 1);
        // skip if it is not valid character
        if ( ( (car_aux < 'A') || (car_aux > 'Z') ) &&
             ( (car_aux < '0') || (car_aux > '9') ) &&
               (car_aux != ':')  && (car_aux != ' ') &&
               (car_aux != '\n') && (car_aux != '.') &&
               (car_aux != '%') ) {
            continue;
        }
        // store the character
        aux_buf[count] = car_aux;

        //increment count in a circular way
        count = count + 1;
        if (count == MSG_LEN) count = 0;

        // if character is new_line return answer
        if (car_aux == '\n') {
           int first_part_size = strlen(&(aux_buf[count]));
           memcpy(buffer,&(aux_buf[count]), first_part_size);
           memcpy(&(buffer[first_part_size]),aux_buf,count);
           return 0;
        }
    }
    strncpy(buffer,"MSG: ERR\n",MSG_LEN);
    return 0;
}

//-------------------------------------
//-  Function: task_speed
//-------------------------------------
int task_speed()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    //--------------------------------
    //  request speed and display it
    //--------------------------------

    //clear request and answer
    memset(request, '\0', MSG_LEN+1);
    memset(answer, '\0', MSG_LEN+1);

    // request speed
    strcpy(request, "SPD: REQ\n");

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
    if (1 == sscanf (answer, "SPD:%f\n", &speed)){
        displaySpeed(speed);
    }
    return 0;
}

//-------------------------------------
//-  Function: task_slope
//-------------------------------------
int task_slope()
{
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    //--------------------------------
    //  request slope and display it
    //--------------------------------

    //clear request and answer
    memset(request,'\0',MSG_LEN+1);
    memset(answer,'\0',MSG_LEN+1);

    // request slope
    strcpy(request, "SLP: REQ\n");

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

int communicate(char *request, char *answer) {
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

//-------------------------------------
//-  Function: accelerator
//-------------------------------------

int task_accelerator() {

    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    //--------------------------------
    //  request slope and send acceleration message
    //--------------------------------

    //clear request and answer
    memset(request,'\0',MSG_LEN+1);
    memset(answer,'\0',MSG_LEN+1);

    // request slope
    if (speed <= 55) {
        if (brake_active) {
            strcpy(request, "BRK: CLR\n");
            brake_active = 0;
        } else {
            strcpy(request, "GAS: SET\n");
            accelerator_active = 1;
        }
    }
    //send request, and recieve answer
    communicate(request, answer);

    //display acceleration
    displayGas(accelerator_active);

    return (0);
}

//-------------------------------------
//-  Function: brake
//-------------------------------------
int task_brake() {

    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];
    int activate;

    //--------------------------------
    //  request slope and send acceleration message
    //--------------------------------

    //clear request and answer
    memset(request,'\0',MSG_LEN+1);
    memset(answer,'\0',MSG_LEN+1);

    // request slope
    if (speed > 55) {
        if (accelerator_active) {
            strcpy(request, "GAS: CLR\n");
            accelerator_active = 0;
        } else {
            strcpy(request, "BRK: SET\n");
            brake_active = 1;
        }
    }

    //send request, and recieve answer
    communicate(request, answer);

    //display acceleration
    displayBrake(brake_active);

    return (0);
}


//-------------------------------------
//-  Function: mixer
//-------------------------------------
int task_mixer() {
    char request[MSG_LEN+1];
    char answer[MSG_LEN+1];

    //--------------------------------
    //  request slope and send acceleration message
    //--------------------------------

    //clear request and answer
    memset(request,'\0',MSG_LEN+1);
    memset(answer,'\0',MSG_LEN+1);



    //check mixing time
    if ((getClock() - idle_time) >= 60.0) {
        mixing = 1;
        mixing_time = getClock();
        // clock_gettime(CLOCK_REALTIME, &mixingTime);
        idle_time = getClock() + 100.0;
        // restingTime.tv_sec = INT_MAX;
        strcpy(request, "MIX: SET\n");
    }

    if ((getClock() - mixing_time) >= 30.0) {
        mixing = 0;
        idle_time = getClock();
        // clock_gettime(CLOCK_REALTIME, &restingTime);
        mixing_time = getClock() + 100.0;
        // mixingTime.tv_sec = INT_MAX;
        strcpy(request, "MIX: CLR\n");
    }

    communicate(request, answer);

    displayMix(mixing);

    return (0);
}

#define N 3
void *controller(void *arg)
{
    idle_time = getClock();
    mixing_time = getClock() + 100.0;
    double timeInit, timeEnd, timeDiff, timeWait, maxTime;
    // clock_gettime(CLOCK_REALTIME, &restingTime);
    // mixingTime.tv_sec = INT_MAX;
    // struct timespec timeInit, timeEnd, timeDiff, timeWait, maxTime;
    timeWait = 10.0;
    // maxTime.tv_sec = INT_MAX;
    // maxTime.tv_nsec = INT_MAX;
    int ts = 0;
    timeInit = getClock();
    // clock_gettime(CLOCK_REALTIME, &timeInit);
    // Endless loop
    while(1) {
        switch (ts)
        {
        case 0:
            task_slope();
            task_speed();
            task_accelerator();
            task_brake();
            task_mixer();
            break;

        case 1:
            task_slope();
            task_speed();
            task_accelerator();
            task_brake();
            break;

        case 2:
            task_slope();
            task_speed();
            task_accelerator();
            task_brake();
            task_mixer();
            break;    
        
        }
        ts = (ts + 1) % N;
        timeEnd = getClock();
        // clock_gettime(CLOCK_REALTIME, &timeEnd);
        if (timeEnd < timeInit) {
            timeDiff = INT_MAX - timeInit + timeEnd;
            // diffTime(maxTime, timeInit, &timeDiff);
            // diffTime(timeDiff, timeEnd, &timeDiff);
        } else {
            timeDiff = timeEnd - timeInit;
            // diffTime(timeEnd, timeInit, &timeDiff);
        }

        sprintf(debug, "timeDiff: %lf timeInit: %lf timeEnd: %lf", timeDiff, timeInit, timeEnd);
        if (timeWait < timeDiff) {
            //error
            exit(0);
        }
        
        // diffTime(timeWait, timeDiff, &timeDiff);
        sleep((int) (timeWait - timeDiff));
        // nanosleep(&timeDiff,NULL);

        timeInit += timeWait;
        // addTime(timeInit, timeWait, &timeInit);

    }
}

//-------------------------------------
//-  Function: Init
//-------------------------------------
rtems_task Init (rtems_task_argument ignored)
{
    pthread_t thread_ctrl;
    sigset_t alarm_sig;
    int i;

    strcpy(debug, "initial\0");

    /* Block all real time signals so they can be used for the timers.
       Note: this has to be done in main() before any threads are created
       so they all inherit the same mask. Doing it later is subject to
       race conditions */
    sigemptyset (&alarm_sig);
    for (i = SIGRTMIN; i <= SIGRTMAX; i++) {
        sigaddset (&alarm_sig, i);
    }
    sigprocmask (SIG_BLOCK, &alarm_sig, NULL);

    // init display
    displayInit(SIGRTMAX);

#if defined(ARDUINO)
    /* Open serial port */
    char serial_dev[]="/dev/com1";
    fd_serie = open (serial_dev, O_RDWR);
    if (fd_serie < 0) {
        printf("open: error opening serial %s\n", serial_dev);
        exit(-1);
    }

    struct termios portSettings;
    speed_t speed=B9600;

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