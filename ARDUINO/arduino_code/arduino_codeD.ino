// ------------------------------------
// Include files
// ------------------------------------
#include <string.h>
#include <stdio.h>
#include <Wire.h>

// ------------------------------------
// Global Constants
// ------------------------------------
#define SLAVE_ADDR          0x8
#define MESSAGE_SIZE        8+2             // useful message size + required overhead ("/n", "/0")
#define MAX_UNSIGNED_LONG   0xffffffffUL
#define us                  1000000.0       // number of microsenconds in a second

// Cyclic Scheduler stuff
#define SC_REQUIRED_WAIT    5UL             // required time to wait for next secondary cycle
// number of secondary cycles per main cycle
#define DIST_SEL_NUM_SC     2
#define APPROACH_NUM_SC     2
#define STOP_NUM_SC         2
// secondary cycle time length
#define DIST_SEL_SC_TIME    100UL
#define APPROACH_SC_TIME    100UL
#define STOP_SC_TIME        100UL



// ------------------------------------
// Digital PIN Numbers
// ------------------------------------
#define ACCELERATION    13
#define BRAKE           12
#define MIXER           11
#define SPEED           10
#define DOWN            9
#define UP              8
#define LAMPS           7
#define PUSH_BUTTON     6
#define DECO_D          5
#define DECO_C          4
#define DECO_B          3
#define DECO_A          2


// ------------------------------------
// Analog PIN Numbers
// ------------------------------------
#define LDR             0
#define POTENTIOMETER   1


// ------------------------------------
// Slope Legal Values
// ------------------------------------
typedef enum{flat, down, up} slope_t;


// ------------------------------------
// Operation Mode Values
// ------------------------------------
typedef enum{selection_mode, approach_mode, stop_mode, emergency_mode} mode_t;


// ------------------------------------
// Limits
// ------------------------------------
// Speed
#define SPEED_MIN       40.0
#define SPEED_MAX       70.0

// LDR
#define LDR_VAL_MIN     0
#define LDR_VAL_MAX     99

// Distance to Deposit
#define DEPO_DIST_MIN   10000
#define DEPO_DIST_MAX   90000


// ------------------------------------
// Speed Modifiers
// ------------------------------------
#define MOD_GAS         0.5
#define MOD_BRK         -0.5
#define MOD_SLOPE_DOWN  0.25
#define MOD_SLOPE_UP    -0.25


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
#define REQ_STP             "STP: REQ\n"
#define REQ_DIS             "DS:  REQ\n" 
#define REQ_EMERGENCY_SET   "ERR: SET\n"


// Answers
#define ANS_ERROR           "MSG: ERR\n"
#define ANS_SPEED           "SPD:%s\n"
#define ANS_SLOPE_FLAT      "SLP:FLAT\n"
#define ANS_SLOPE_DOWN      "SLP:DOWN\n"
#define ANS_SLOPE_UP        "SLP:  UP\n"
#define ANS_GAS_OK          "GAS:  OK\n"
#define ANS_BRK_OK          "BRK:  OK\n"
#define ANS_MIX_OK          "MIX:  OK\n"
#define ANS_LIT             "LIT: %.2d%%\n"
#define ANS_LAM_OK          "LAM:  OK\n"
#define ANS_STP_GO          "STP:  GO\n"
#define ANS_STP_STOP        "STP:STOP\n"
#define ANS_DIS             "DS:%.5ld\n"
#define ANS_EMERGENCY_OK    "ERR:  OK\n"


// ------------------------------------
// Macros
// ------------------------------------

// write to "answer" buffer and set flag
#define SET_ANSWER(...) \
    sprintf(answer, __VA_ARGS__); \
    requested_answered = true;

// compare "request" buffer with given SET/CLR request strings
#define CHECK_SET_CLR_REQ_TYPE(SET_REQ, CLR_REQ) \
    int set, clr; \
    set = strcmp((SET_REQ), (const char *) request); \
    clr = strcmp((CLR_REQ), (const char *) request);

// used to get task compute time
#define TIME_TASK(TASK) \
    time_exec_begin = micros(); \
    (TASK); \
    time_exec_end = micros(); \
    elapsed = time_exec_end - time_exec_begin;


// ------------------------------------
// Global Variables
// ------------------------------------

// Comm server stuff
bool request_received = false;
bool requested_answered = false;
char request[MESSAGE_SIZE];
char answer[MESSAGE_SIZE];

// Sensors and states
bool gas_is_active = false;
bool brake_is_active = false;
bool mixer_is_active = false;
bool lamps_is_active = false;
double speed = 55.5;
slope_t railway_slope = flat;
unsigned char ldr_value = 0;
unsigned long depo_distance = 10000;
long actual_distance = 0;
char old_value_button = 0;
mode_t mode = selection_mode;

unsigned long speed_last_measure_time;
unsigned long distance_last_measure_time;


// ------------------------------------
// Function Prototypes
// ------------------------------------

/************** Tasks ****************/

// ------------------------------------
// Task: Communication Server
// Worst Compute Time: 9368 μs
// ------------------------------------
void comm_server_task();
// ------------------------------------
// Task: On/Off Accelerator
// Worst Compute Time: 16 μs
// ------------------------------------
void acceleration_task();
// ------------------------------------
// Task: On/Off Brake
// Worst Compute Time: 16 μs
// ------------------------------------
void brake_task();
// ------------------------------------
// Task: On/Off Mixer
// Worst Compute Time: 16 μs
// ------------------------------------
void mixer_task();
// ------------------------------------
// Task: On/Off Lamps
// Worst Compute Time: 8 μs
// ------------------------------------
void lamps_task();
// ------------------------------------
// Task: Compute and Show Speed
// Worst Compute Time: 128 μs
// ------------------------------------
void display_speed_task();
// ------------------------------------
// Task: Read Slope
// Worst Compute Time: 12 μs
// ------------------------------------
void read_slope_task();
// ------------------------------------
// Task: Read LDR value
// Worst Compute Time: 212 μs
// ------------------------------------
void read_ldr_task();
// ------------------------------------
// Task: Compute distance digit and show it 
// Worst Compute Time: 128 μs
// ------------------------------------
void display_depo_dist_task();
// ------------------------------------
// Task: Read potentiometer value 
// Worst Compute Time: 208 μs
// ------------------------------------
void read_potentiometer_task();
// ------------------------------------
// Task: Read button and take actual distance
// Worst Compute Time: 16 μs
// ------------------------------------
void read_button_task();

/********** Aux functions ************/
void update_speed();
void set_7seg_display(const unsigned char digit);
void comm_server();

/******** Request Handlers ***********/

// Get requests
void req_get_speed();
void req_get_slope();
void req_get_lit();
void req_get_dis();
// Set requests
void req_set_gas();
void req_set_brake();
void req_set_mixer();
void req_set_lamps();


/********** Aux functions ************/
void set_7seg_display(const unsigned char digit)
{
    unsigned char bit3, bit2, bit1, bit0;

    // get individual bits from given digit
    bit3 = digit & 0b00001000;
    bit2 = digit & 0b00000100;
    bit1 = digit & 0b00000010;
    bit0 = digit & 0b00000001;

    // now set 7 segment display
    digitalWrite(DECO_D, bit3);
    digitalWrite(DECO_C, bit2);
    digitalWrite(DECO_B, bit1);
    digitalWrite(DECO_A, bit0);
}


void update_speed()
{
    unsigned long speed_new_measure_time, delta;
    double acceleration;

    if (mode == stop_mode) {
        speed = 0;
        return;
    }

    speed_new_measure_time = micros();

    // get elapsed time between speed measurements
    if (speed_last_measure_time > speed_new_measure_time)
        delta = MAX_UNSIGNED_LONG - speed_last_measure_time + speed_new_measure_time;
    else
        delta = speed_new_measure_time - speed_last_measure_time;

    speed_last_measure_time = speed_new_measure_time;

    acceleration = MOD_GAS*gas_is_active + MOD_BRK*brake_is_active +
    MOD_SLOPE_DOWN*(railway_slope == down) + MOD_SLOPE_UP*(railway_slope == up);

    speed += acceleration * ((double) delta/us);
    if (speed < 0)
        speed = 0;
}


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
        if (requested_answered)
            Serial.print(answer);
        else
            Serial.print(ANS_ERROR);

        // reset flags and buffers
        request_received = false;
        requested_answered = false;
        memset(request, 0, MESSAGE_SIZE);
        memset(answer, 0, MESSAGE_SIZE);
    }

    while (Serial.available()) {
        // read one character
        char_aux = Serial.read();
          
        // skip if it is not a valid character
        if ( ( char_aux < 'A' || char_aux > 'Z' ) &&
        char_aux != ':' && char_aux != ' ' && char_aux != '\n' ) {
            continue;
        }
        
        // Store the character
        request[count] = char_aux;
        
        // If the last character is an enter or
        // There are 9th characters set an enter and finish.
        if ( request[count] == '\n' || count == 8 ) {
            request[count] = '\n';
            count = 0;
            request_received = true;
            break;
        }

        count++;
    }
}


/******** Request Handlers ***********/
void req_get_speed()
{ 
    // check request
    if ( !strcmp(REQ_SPEED, request) ) {  
        // convert double to string
        char num_str[5];
        dtostrf(speed, 4, 1, num_str);
        // add a 0 if speed < 10
        if (num_str[0] == ' ')
            num_str[0] = '0';

        SET_ANSWER(ANS_SPEED, num_str);
    }
}


void req_get_slope()
{
    // check request
    if ( !strcmp(REQ_SLOPE, request) ) {
        // set answer according to current slope
        switch (railway_slope) {
            case flat:
                SET_ANSWER(ANS_SLOPE_FLAT);
                break;
            case down:
                SET_ANSWER(ANS_SLOPE_DOWN);
                break;
            case up:
                SET_ANSWER(ANS_SLOPE_UP);
                break;
        }
    }
}


void req_get_lit()
{ 
    // check request
    if ( !strcmp(REQ_LIT, request) ) {  
        SET_ANSWER(ANS_LIT, ldr_value);
    }
}


void req_get_dis()
{ 
    // check request
    if ( !strcmp(REQ_DIS, request) ) {  
        SET_ANSWER(ANS_DIS, actual_distance);
    }
}


void req_get_movement()
{
    // check request
    if ( !strcmp(REQ_STP, request) ) {
        // set answer according to current op mode
        switch (mode) {
            case selection_mode:
                SET_ANSWER(ANS_STP_GO);
                break;
            case approach_mode:
                SET_ANSWER(ANS_STP_GO);
                break;            
            case stop_mode:
                SET_ANSWER(ANS_STP_STOP);
                break;
        }
    }
}


void req_set_gas()
{
    CHECK_SET_CLR_REQ_TYPE(REQ_GAS_SET, REQ_GAS_CLR);

    // check request
    if (!set || !clr) {
        // set value
        gas_is_active = !set + clr;
        SET_ANSWER(ANS_GAS_OK);
    }
}


void req_set_brake()
{
    CHECK_SET_CLR_REQ_TYPE(REQ_BRK_SET, REQ_BRK_CLR);

    // check request
    if (!set || !clr) {
        // set value
        brake_is_active = !set + clr;
        SET_ANSWER(ANS_BRK_OK);
    }
}


void req_set_mixer()
{
    CHECK_SET_CLR_REQ_TYPE(REQ_MIX_SET, REQ_MIX_CLR);

    // check request
    if (!set || !clr) {
        // set value
        mixer_is_active = !set + clr;
        SET_ANSWER(ANS_MIX_OK);
    }
}


void req_set_lamps()
{
    CHECK_SET_CLR_REQ_TYPE(REQ_LAM_SET, REQ_LAM_CLR);

    // check request
    if (!set || !clr) {
        // set value
        lamps_is_active = !set + clr;
        SET_ANSWER(ANS_LAM_OK);
    }
}

void req_set_emergency()
{ 
    // check request
    if ( !strcmp(REQ_EMERGENCY_SET, request) ) {
        mode = emergency_mode;  
        SET_ANSWER(ANS_EMERGENCY_OK);
    }
}
/************** Tasks ****************/
void comm_server_task()
{
    comm_server();
    // check if there is an unanswered request
    if (!request_received || requested_answered)
        return;

    req_get_speed();
    req_get_slope();
    req_get_lit();
    req_set_gas();
    req_set_brake();
    req_set_mixer();
    req_set_lamps();
    req_get_movement();
    if (mode != selection_mode)
        req_get_dis();
    req_set_emergency();
}


void acceleration_task()
{
    if (mode == emergency_mode){
        gas_is_active = false;    
    }
    digitalWrite(ACCELERATION, gas_is_active);
}


void brake_task()
{
    if (mode == emergency_mode){
        brake_is_active = true;    
    }
    digitalWrite(BRAKE, brake_is_active);
}


void mixer_task()
{
    digitalWrite(MIXER, mixer_is_active);
}


void lamps_task()
{
  if (mode == emergency_mode){
    lamps_is_active = true;    
  }
    digitalWrite(LAMPS, lamps_is_active);
}


void display_speed_task()
{
    unsigned char speed_led_pwm_value;

    // compute new speed first
    update_speed();

    // change speed LED intensity
    // if speed is out of bounds, the LED turns off
    if ( speed != constrain(speed, SPEED_MIN, SPEED_MAX) )
        speed_led_pwm_value = 0;
    else
        speed_led_pwm_value = (unsigned char) map(speed, SPEED_MIN, SPEED_MAX, 0, 255);

    analogWrite(SPEED, speed_led_pwm_value);
}


void display_depo_dist_task()
{
    unsigned long delta, distance_new_measure_time;
    double actual_distance_tmp = (double) actual_distance;

    switch (mode) {
        case selection_mode:
            set_7seg_display((unsigned char) (depo_distance/10000));
            break;
        case approach_mode:
            distance_new_measure_time = micros();

            // get elapsed time between distance measurements
            if (distance_last_measure_time > distance_new_measure_time)
                delta = MAX_UNSIGNED_LONG - distance_last_measure_time + distance_new_measure_time;
            else
                delta = distance_new_measure_time - distance_last_measure_time;

            distance_last_measure_time = distance_new_measure_time;
            actual_distance_tmp -= speed * (double) delta/us;
            actual_distance = (long) actual_distance_tmp;

            set_7seg_display((unsigned char) (actual_distance/10000));   

            if (actual_distance <= 0 && speed < 10) {
                actual_distance = 0;
                mode = stop_mode;            
            } else if (actual_distance <= 0 && speed > 10)
                mode = selection_mode;
            break;
        default:
            break;
    }
}


void read_slope_task()
{
    // read slope pins and compute current slope
    railway_slope = (slope_t) (flat + down*digitalRead(DOWN) + up*digitalRead(UP));
}


void read_ldr_task()
{
    // read LDR analog pin and convert the value to a percentage
    int ldr_analog = analogRead(LDR);
    ldr_value = (unsigned char) map(ldr_analog, 0, 1024, LDR_VAL_MIN, LDR_VAL_MAX);
}

void read_potentiometer_task()
{
    // read potentiometer analog pin and convert the value to a distance
    int potentio_analog = analogRead(POTENTIOMETER);
    depo_distance = (unsigned long) map(potentio_analog, 0, 1024, DEPO_DIST_MIN, DEPO_DIST_MAX);
}


void read_button_task()
{
    int value = digitalRead(PUSH_BUTTON);
    if ( (old_value_button == 0) && (value == 1) ) {
        if (mode == selection_mode){
            actual_distance = (signed long) depo_distance;
            mode = approach_mode;
        } else if (mode == stop_mode)
            mode = selection_mode;
    }
    old_value_button = value;
}


void selection_scheduler()
{
    unsigned char sc = 0; // secondary cycle
    unsigned long exe_start_time, exe_end_time, delta;

    // set initial execution start time
    exe_start_time = millis();

    while (1) {
        // execute tasks
        switch (sc) {
            case 0:
                acceleration_task();
                brake_task();
                mixer_task();
                read_potentiometer_task();
                display_depo_dist_task();   
                display_speed_task();
                read_slope_task();
                read_ldr_task();
                lamps_task();
                read_button_task();
                break;
            case 1:
                acceleration_task();
                brake_task();
                mixer_task();
                read_potentiometer_task();
                display_depo_dist_task();
                display_speed_task();
                read_slope_task();
                read_ldr_task();
                lamps_task();    
                read_button_task();     
                comm_server_task();              
                break;
        }

        sc = (sc + 1) % DIST_SEL_NUM_SC;
        exe_end_time = millis();

        // get elapsed execution time
        if (exe_start_time > exe_end_time)
            delta = (unsigned long) (MAX_UNSIGNED_LONG - exe_start_time + exe_end_time);
        else
            delta = (unsigned long) (exe_end_time - exe_start_time);

        // panic: execution took too long
        if (DIST_SEL_SC_TIME < delta)
            exit(-1);

        if (mode != selection_mode) {
            // initialize time for distance update
            distance_last_measure_time = micros();      
            return;
        }

        // wait until next secondary cycle
        delay(DIST_SEL_SC_TIME - delta + SC_REQUIRED_WAIT);
        exe_start_time += DIST_SEL_SC_TIME;
    }
}


void approach_scheduler()
{
    unsigned char sc = 0; // secondary cycle
    unsigned long exe_start_time, exe_end_time, delta;

    // set initial execution start time
    exe_start_time = millis();

    while (1) {
        switch (sc) {
            case 0:
                acceleration_task();
                brake_task();
                mixer_task();
                display_depo_dist_task();
                display_speed_task();
                read_slope_task();
                read_ldr_task();
                lamps_task();
                break;
            case 1:
                acceleration_task();
                brake_task();
                mixer_task();
                display_depo_dist_task();
                display_speed_task();
                read_slope_task();
                read_ldr_task();
                lamps_task();         
                comm_server_task(); 
                break;
        }

        sc = (sc + 1) % APPROACH_NUM_SC;
        exe_end_time = millis();

        // get elapsed execution time
        if (exe_start_time > exe_end_time)
            delta = (unsigned long) (MAX_UNSIGNED_LONG - exe_start_time + exe_end_time);
        else
            delta = (unsigned long) (exe_end_time - exe_start_time);

        // panic: execution took too long
        if (APPROACH_SC_TIME < delta)
            exit(-1);

        if (mode != approach_mode)
            return;

        // wait until next secondary cycle
        delay(APPROACH_SC_TIME - delta + SC_REQUIRED_WAIT);
        exe_start_time += APPROACH_SC_TIME;
    }
}


void stop_scheduler()
{
    unsigned char sc = 0; // secondary cycle
    unsigned long exe_start_time, exe_end_time, delta;

    // set initial execution start time
    exe_start_time = millis();

    while (1) {
        switch (sc) {
            case 0:
                acceleration_task();
                brake_task();
                mixer_task();
                display_speed_task();
                read_slope_task();
                read_ldr_task();
                lamps_task();
                read_button_task();                        
                break;
            case 1:
                acceleration_task();
                brake_task();
                mixer_task();
                display_speed_task();
                read_slope_task();
                read_ldr_task();
                lamps_task();         
                read_button_task();
                comm_server_task();       
                break;
        }

        sc = (sc + 1) % STOP_NUM_SC;
        exe_end_time = millis();

        // get elapsed execution time
        if (exe_start_time > exe_end_time)
            delta = (unsigned long) (MAX_UNSIGNED_LONG - exe_start_time + exe_end_time);
        else
            delta = (unsigned long) (exe_end_time - exe_start_time);

        // panic: execution took too long
        if (STOP_SC_TIME < delta)
            exit(-1);

        if (mode != stop_mode) {
            // update time so the speed starts from 0 properly
            speed_last_measure_time = micros();
            return;
        }  
        
        // wait until next secondary cycle
        delay(STOP_SC_TIME - delta + SC_REQUIRED_WAIT);
        exe_start_time += STOP_SC_TIME;
    }
}

void emergency_scheduler()
{
    unsigned char sc = 0; // secondary cycle
    unsigned long exe_start_time, exe_end_time, delta;

    // set initial execution start time
    exe_start_time = millis();

    while (1) {
        // execute tasks
        switch (sc) {
            case 0:
                acceleration_task();
                brake_task();
                mixer_task();
                display_speed_task();
                read_slope_task();
                lamps_task();
                break;
            case 1:
                acceleration_task();
                brake_task();
                mixer_task();
                display_speed_task();
                read_slope_task();
                lamps_task();    
                comm_server_task();              
                break;
        }

        sc = (sc + 1) % DIST_SEL_NUM_SC;
        exe_end_time = millis();

        // get elapsed execution time
        if (exe_start_time > exe_end_time)
            delta = (unsigned long) (MAX_UNSIGNED_LONG - exe_start_time + exe_end_time);
        else
            delta = (unsigned long) (exe_end_time - exe_start_time);

        // panic: execution took too long
        if (DIST_SEL_SC_TIME < delta)
            exit(-1);

        // wait until next secondary cycle
        delay(DIST_SEL_SC_TIME - delta + SC_REQUIRED_WAIT);
        exe_start_time += DIST_SEL_SC_TIME;
    }
}


void setup()
{
    // set pins
    pinMode(ACCELERATION,   OUTPUT);
    pinMode(BRAKE,          OUTPUT);
    pinMode(MIXER,          OUTPUT);
    pinMode(SPEED,          OUTPUT);
    pinMode(DOWN,           INPUT);
    pinMode(UP,             INPUT);
    pinMode(LAMPS,          OUTPUT);
    pinMode(DECO_D,         OUTPUT);
    pinMode(DECO_C,         OUTPUT);
    pinMode(DECO_B,         OUTPUT);
    pinMode(DECO_A,         OUTPUT);
    pinMode(PUSH_BUTTON,    INPUT);

    // Setup Serial Monitor
    Serial.begin(9600);

    // set initial speed measure time
    speed_last_measure_time = micros();
}


void loop()
{
    switch (mode) {
        case selection_mode:
            selection_scheduler();
            break;
        case approach_mode:
            approach_scheduler();
            break;
        case stop_mode:
            stop_scheduler();
            break;
        case emergency_mode:
            emergency_scheduler();
            break;
    }
}
