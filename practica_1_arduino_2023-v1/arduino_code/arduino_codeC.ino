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
#define MESSAGE_SIZE        8+2             // useful message size + required overhead ("/n", "/0")
#define MAX_UNSIGNED_LONG   0xffffffffUL
#define us                  1000000UL
// Cyclic Scheduler stuff
#define NUM_SC              2               // number of secondary cycles per main cycle
#define SC_TIME             100UL           // secondary cycle time length
#define SC_REQUIRED_WAIT    5UL             // required time to wait for next secondary cycle


// --------------------------------------
// Digital PIN Numbers
// --------------------------------------
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



// --------------------------------------
// Analog PIN Numbers
// --------------------------------------
#define LDR             0
#define POTENTIOMETER   1


// --------------------------------------
// Slope Legal Values
// --------------------------------------
typedef enum{flat = 0, down = 1, up = 2} slope_t;


// --------------------------------------
// Limits
// --------------------------------------
// Speed
#define SPEED_MIN       40.0
#define SPEED_MAX       70.0

// LDR
#define LDR_VAL_MIN     0
#define LDR_VAL_MAX     99

//Distance
#define DEPO_DIST_MIN   10000
#define DEPO_DIST_MAX   90000


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
#define REQ_LIT         "LIT: REQ\n"
#define REQ_LAM_SET     "LAM: SET\n"
#define REQ_LAM_CLR     "LAM: CLR\n"

// Answers
#define ANS_ERROR       "MSG: ERR\n"
#define ANS_SPEED       "SPD:%s\n"
#define ANS_SLOPE_FLAT  "SLP:FLAT\n"
#define ANS_SLOPE_DOWN  "SLP:DOWN\n"
#define ANS_SLOPE_UP    "SLP:  UP\n"
#define ANS_GAS_OK      "GAS:  OK\n"
#define ANS_BRK_OK      "BRK:  OK\n"
#define ANS_MIX_OK      "MIX:  OK\n"
#define ANS_LIT         "LIT: %.2d%%\n"
#define ANS_LAM_OK      "LAM:  OK\n"


// --------------------------------------
// Macros
// --------------------------------------
// write to "answer" buffer and set flag
#define SET_ANSWER(...) \
    sprintf(answer, __VA_ARGS__); \
    requested_answered = true;

// compare "request" buffer with given SET/CLR request strings
#define CHECK_SET_CLR_REQ_TYPE(SET_REQ, CLR_REQ) \
    int set, clr; \
    set = strcmp((SET_REQ), (const char *) request); \
    clr = strcmp((CLR_REQ), (const char *) request);

#define SET_7_SEG_DISPLAY(BIT3, BIT2, BIT1, BIT0) \
    digitalWrite(DECO_D, (BIT3)); \
    digitalWrite(DECO_C, (BIT2)); \
    digitalWrite(DECO_B, (BIT1)); \
    digitalWrite(DECO_A, (BIT0));


// --------------------------------------
// Global Variables
// --------------------------------------
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
unsigned long actual_distance = 0;
char old_value_button = 0;

unsigned long speed_last_measure_time;

// Scheduler stuff
unsigned char sc = 0; // secondary cycle
unsigned long exe_start_time;


// --------------------------------------
// Function Prototypes
// --------------------------------------
// Tasks

// --------------------------------------
// Task: Communication Server
// Worst Compute Time: 9368 μs
// --------------------------------------
void comm_server_task();
// --------------------------------------
// Task: On/Off Accelerator
// Worst Compute Time: 16 μs
// --------------------------------------
void acceleration_task();
// --------------------------------------
// Task: On/Off Brake
// Worst Compute Time: 16 μs
// --------------------------------------
void brake_task();
// --------------------------------------
// Task: On/Off Mixer
// Worst Compute Time: 16 μs
// --------------------------------------
void mixer_task();
// --------------------------------------
// Task: On/Off Lamps
// Worst Compute Time: 8 μs
// --------------------------------------
void lamps_task();
// --------------------------------------
// Task: Compute and Show Speed
// Worst Compute Time: 128 μs
// --------------------------------------
void display_speed_task();
// --------------------------------------
// Task: Read Slope
// Worst Compute Time: 12 μs
// --------------------------------------
void read_slope_task();
// --------------------------------------
// Task: Read LDR value
// Worst Compute Time: 212 μs
// --------------------------------------
void read_ldr_task();

// Aux functions
void update_speed();
void comm_server();
// Get requests
void req_get_speed();
void req_get_slope();
void req_get_lit();
// Set requests
void req_set_gas();
void req_set_brake();
void req_set_mixer();
void req_set_lamps();


// --------------------------------------
// Aux functions
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

    acceleration = MOD_GAS*gas_is_active + MOD_BRK*brake_is_active +
    MOD_SLOPE_DOWN*(railway_slope == down) + MOD_SLOPE_UP*(railway_slope == up);

    speed += (double) acceleration * ((double) delta/us);
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


// --------------------------------------
// Request Handlers
// --------------------------------------
void req_get_speed()
{ 
    // check request
    if ( !strcmp(REQ_SPEED, request) ) {  
        // convert double to string
        char num_str[5];
        dtostrf(speed, 4,1, num_str);

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


// --------------------------------------
// Tasks
// --------------------------------------
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
}


void acceleration_task()
{
    digitalWrite(ACCELERATION, gas_is_active);
}


void brake_task()
{
    digitalWrite(BRAKE, brake_is_active);
}


void mixer_task()
{
    digitalWrite(MIXER, mixer_is_active);
}


void lamps_task()
{
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
void display_depo_dist_task(){
    char display = (char) (depo_distance/10000);
    switch(display){
        case 0:
            SET_7_SEG_DISPLAY(0,0,0,0)
            break;
        case 1:
            SET_7_SEG_DISPLAY(0,0,0,1)
            break;
        case 2:
            SET_7_SEG_DISPLAY(0,0,1,0)
            break;
        case 3:
            SET_7_SEG_DISPLAY(0,0,1,1)
            break;
        case 4:
            SET_7_SEG_DISPLAY(0,1,0,0)
            break;
        case 5:
            SET_7_SEG_DISPLAY(0,1,0,1)
            break;
        case 6:
            SET_7_SEG_DISPLAY(0,1,1,0)
            break;
        case 7:
            SET_7_SEG_DISPLAY(0,1,1,1)
            break;
        case 8:
            SET_7_SEG_DISPLAY(1,0,0,0)
            break;
        case 9:
            SET_7_SEG_DISPLAY(1,0,0,1)
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

void read_button_task(){
    int value = digitalRead(PUSH_BUTTON);
    if ((old_value_button == 0) && (value == 1)){
      actual_distance = depo_distance;
      Serial.println(actual_distance);
    }
    old_value_button = value;
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

    // Setup Serial Monitor
    Serial.begin(9600);

    // set initial speed measure time
    speed_last_measure_time = micros();

    // set initial execution start time
    exe_start_time = millis();
}


void loop()
{
  /*  unsigned long exe_end_time, delta;
    
    // execute tasks
    switch (sc) {
        case 0:
            acceleration_task();
            brake_task();
            mixer_task();
            display_speed_task();
            read_slope_task();
            read_ldr_task();
            lamps_task();                        
            break;
        case 1:
            acceleration_task();
            brake_task();
            mixer_task();
            display_speed_task();
            read_slope_task();
            read_ldr_task();
            lamps_task();                        
            comm_server_task();
            break;
    }

    sc = (sc + 1) % NUM_SC;
    exe_end_time = millis();

    // get elapsed execution time
    if (exe_start_time > exe_end_time)
        delta = (unsigned long) (MAX_UNSIGNED_LONG - exe_start_time + exe_end_time);
    else
        delta = (unsigned long) (exe_end_time - exe_start_time);

    // panic: execution took too long
    if (SC_TIME < delta)
        exit(-1);

    // wait until next secondary cycle
    delay(SC_TIME - delta + SC_REQUIRED_WAIT);
    exe_start_time += SC_TIME;

    */



    unsigned long time_exec_begin, time_exec_end, elapsed;
    time_exec_begin = micros();
    //compute time code
    read_button_task();
    delay(100);


    

    time_exec_end = micros();
    elapsed = time_exec_end - time_exec_begin;
    // comm_server();
    // speed_req();

}

