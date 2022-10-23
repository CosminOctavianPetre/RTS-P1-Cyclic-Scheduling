// --------------------------------------
// Include files
// --------------------------------------
#include <string.h>
#include <stdio.h>
#include <Wire.h>

// --------------------------------------
// Global Constants
// --------------------------------------
#define SLAVE_ADDR          0x8
#define MESSAGE_SIZE        8
#define MAX_UNSIGNED_LONG   0xffffffffUL
#define us                  1000000UL


// --------------------------------------
// PIN Numbers
// --------------------------------------
#define ACCELERATION    13
#define BRAKE           12
#define MIXER           11
#define SPEED           10
#define DOWN            8
#define UP              9


// --------------------------------------
// Speed Modifiers
// --------------------------------------
#define MOD_GAS         0.5
#define MOD_BRK         -0.5
#define MOD_SLOPE_DOWN  0.25
#define MOD_SLOPE_UP    -0.25


// --------------------------------------
// Messages
// --------------------------------------
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
#define ANS_SPEED       "SPD:%s\n"
#define ANS_SLOPE_FLAT  "SLP:FLAT\n"
#define ANS_SLOPE_DOWN  "SLP:DOWN\n"
#define ANS_SLOPE_UP    "SLP:  UP\n"
#define ANS_GAS_OK      "GAS:  OK\n"
#define ANS_BRK_OK      "BRK:  OK\n"
#define ANS_MIX_OK      "MIX:  OK\n"


// --------------------------------------
// Macros
// --------------------------------------
#define SET_ANSWER(...) sprintf(answer, __VA_ARGS__)


// --------------------------------------
// Global Variables
// --------------------------------------
// Comm server stuff
bool request_received = false;
bool requested_answered = false;
char request[MESSAGE_SIZE+1];
char answer[MESSAGE_SIZE+1];

// Sensors and states
bool gas_is_active = false;
bool brake_is_active = false;
bool mixer_is_active = false;
double speed = 55.5;
//0 = Flat, 1 = Down, 2 = Up
int railway_slope = 0; 

unsigned long speed_last_measure_time;

// Scheduler stuff
int sc = 0; // secondary cycle
unsigned long exe_start_time;
unsigned long sc_time = 50UL;


// --------------------------------------
// Function: comm_server
// Worst compute time: 212 μs
// --------------------------------------
void comm_server()
{
    static int count = 0;
    char char_aux;

    // If there were a received msg, send the processed answer or ERROR if none.
    // then reset for the next request.
    // NOTE: this requires that between two calls of com_server all possible 
    //       answers have been processed.
    if (request_received) {
        // if there is an answer send it, else error
        if (requested_answered) {
            Serial.print(answer);
        } else {
            Serial.print(ANS_ERROR);
        }
        // reset flags and buffers
        request_received = false;
        requested_answered = false;
        memset(request, 0, MESSAGE_SIZE+1);
        memset(answer, 0, MESSAGE_SIZE+1);
    }

    while (Serial.available()) {
        // read one character
        char_aux = Serial.read();

        // skip if it is not a valid character
        if ( (char_aux < 'A' || char_aux > 'Z') &&
        char_aux != ':' && char_aux != ' ' && char_aux != '\n' )
            continue;

        // Store the character
        request[count] = char_aux;

        // If the last character is an enter or
        // There are 9th characters set an enter and finish.
        if (request[count] == '\n' || count == 8) {
            request[count] = '\n';
            count = 0;
            request_received = true;
            break;
        }

        count++;
    }

    // handle requests
    speed_req();
    slope_req();
    gas_req();
    brake_req();
    mixer_req();
}


// --------------------------------------
// Function: speed_req
// --------------------------------------
void speed_req()
{
    // If there is a request not answered, check if this is the one
    if ( request_received && !requested_answered && !strcmp(REQ_SPEED, request) ) {
        // send answer for read speed request
        char num_str[5];
        dtostrf(speed, 4,1, num_str);
        sprintf(answer, ANS_SPEED, num_str);

        // set request as answered
        requested_answered = true;
    }
}


// --------------------------------------
// Function: slope_req
// --------------------------------------
void slope_req()
{
    // If there is a request not answered, check if this is the one
    if ( request_received && !requested_answered && !strcmp(REQ_SLOPE, request) ) {
        // send answer for read slope request
        switch (railway_slope) {
            case 0:
                sprintf(answer, ANS_SLOPE_FLAT);
                break;
            case 1:
                sprintf(answer, ANS_SLOPE_DOWN);
                break;
            case 2:
                sprintf(answer, ANS_SLOPE_UP);
                break;
            default:
                break;
        }
    
        // set request as answered
        requested_answered = true;
    }
}


// --------------------------------------
// Function: gas_req
// --------------------------------------
void gas_req()
{
    int set, clr;

    set = strcmp(REQ_GAS_SET, request);
    clr = strcmp(REQ_GAS_CLR, request);

    // If there is a request not answered, check if this is the one
    if ( request_received && !requested_answered && (!set || !clr) ) {
        //TODO: refactor
        // gas_is_active = !set + !clr;
        if (!set)
            gas_is_active = true;
        else
            gas_is_active = false;

        // send answer for "activating accelerator" request
        sprintf(answer, ANS_GAS_OK);
        // set request as answered
        requested_answered = true;
    }
}


// --------------------------------------
// Function: brake_req
// --------------------------------------
void brake_req()
{
    int set, clr;

    set = strcmp(REQ_BRK_SET, request);
    clr = strcmp(REQ_BRK_CLR, request);

    // If there is a request not answered, check if this is the one
    if ( request_received && !requested_answered && (!set || !clr) ) {
        //TODO: refactor
        // brake_is_active = !set + !clr;
        if (!set)
            brake_is_active = true;
        else
            brake_is_active = false;

        // send answer for "activating brake" request
        sprintf(answer, ANS_BRK_OK);
        // set request as answered
        requested_answered = true;
    }
}


// --------------------------------------
// Function: mixer_req
// --------------------------------------
void mixer_req()
{
    int set, clr;

    set = strcmp(REQ_MIX_SET, request);
    clr = strcmp(REQ_MIX_CLR, request);

    // If there is a request not answered, check if this is the one
    if ( request_received && !requested_answered && (!set || !clr) ) {
        //TODO: refactor
        // mixer_is_active = !set + !clr;
        if (!set)
            mixer_is_active = true;
        else
            mixer_is_active = false;

        // send answer for "activating mixer" request
        sprintf(answer, ANS_MIX_OK);
        // set request as answered
        requested_answered = true;
    }
}


// --------------------------------------
// Function: acceleration_system
// Worst compute time: 16 μs
// --------------------------------------
void acceleration_system()
{
    digitalWrite(ACCELERATION, gas_is_active);
}


// --------------------------------------
// Function: brake_system
// Worst compute time: 16 μs
// --------------------------------------
void brake_system()
{
    digitalWrite(BRAKE, brake_is_active);
}


// --------------------------------------
// Function: mixer_system
// Worst compute time: 16 μs
// --------------------------------------
void mixer_system()
{
    digitalWrite(MIXER, mixer_is_active);
}


// --------------------------------------
// Function: read_slope
// Worst compute time: 20 μs
// --------------------------------------
void read_slope()
{
    int up, down;

    up = digitalRead(UP);
    down = digitalRead(DOWN);

    //TODO: refactor
    if (up == 0 && down == 0) {
        railway_slope = 0;
    } else if (up == 1) {
        railway_slope = 2;
    } else {
        railway_slope = 1;      
    }
}


// --------------------------------------
// Function: update_speed
// --------------------------------------
void update_speed()
{
    unsigned long speed_new_measure_time, delta;
    double acceleration;

    speed_new_measure_time = micros();

    // get elapsed time between speed measurements
    if (speed_last_measure_time > speed_new_measure_time)
	    delta = MAX_UNSIGNED_LONG - speed_last_measure_time + speed_new_measure_time;
	else
	    delta = speed_new_measure_time - speed_last_measure_time;

    speed_last_measure_time = speed_new_measure_time;

    //TODO: refactor
    // acceleration = MOD_GAS*gas_is_active + MOD_BRK*brake_is_active MOD_SLOPE_DOWN*(railway_slope == 1) + MOD_SLOPE_UP*(railway_slope == 2);
    acceleration = MOD_GAS*gas_is_active + MOD_BRK*brake_is_active;
    if (railway_slope)
        acceleration += (railway_slope == 1) ? MOD_SLOPE_DOWN : MOD_SLOPE_UP;

    speed += (double) acceleration * (delta/us);
}


// --------------------------------------
// Function: show_current_speed
// Worst compute time: 108 μs
// --------------------------------------
void show_current_speed()
{
    int pwm_value;

    // compute new speed first
    update_speed();

    // change speed LED intensity
    //TODO: refactor
    //pwm_value = (unsigned long) map(speed, 40, 70, 0, 255);

    if (speed <= 40) {
        pwm_value = 0;
    } else if (speed >= 70) {
        pwm_value = 255;
    } else {
        pwm_value = (int) ((speed - 40)*255) / 30;
    }

    analogWrite(SPEED, pwm_value);

    /*
    char speed_str_tmp[5], speed_str[16];
    strcpy(speed_str, "speed: ");

    dtostrf(speed, 2,2, speed_str_tmp);
    sprintf(speed_str, "%s %s", speed_str, speed_str_tmp);
    Serial.println(speed_str);
    */
}


// --------------------------------------
// Function: setup
// --------------------------------------
void setup()
{
    // set pins
    pinMode(ACCELERATION, OUTPUT);
    pinMode(BRAKE, OUTPUT);
    pinMode(MIXER, OUTPUT);
    pinMode(SPEED, OUTPUT);

    pinMode(DOWN, INPUT);
    pinMode(UP, INPUT);

    // Set up Serial Monitor
    Serial.begin(9600);

    // set initial speed measure time
    speed_last_measure_time = micros();

    // set initial execution start time
    exe_start_time = millis();
}


// --------------------------------------
// Function: loop
// --------------------------------------
void loop()
{
    unsigned long exe_end_time, delta;

    // execute tasks
    switch (sc) {
        case 0:
            acceleration_system();
            brake_system();
            break;
        case 1:
            mixer_system();
            show_current_speed();
            break;
        case 2:
            read_slope();
            break;
        case 3:
            comm_server();
            break;
    }

    sc = (sc + 1) % 4;
    exe_end_time = millis();

    // get elapsed execution time
    if (exe_start_time > exe_end_time) {
        delta = MAX_UNSIGNED_LONG - exe_start_time + exe_end_time;
    } else {
        delta = exe_end_time - exe_start_time;
    }

    // check if execution took too long
    if (sc_time < delta) {
        exit(-1);
    }

    // wait until next secondary cycle
    delay(sc_time - delta);
    exe_start_time += sc_time;
}
