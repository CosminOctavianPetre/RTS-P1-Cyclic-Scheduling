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
#define SC_TIME             50UL


// --------------------------------------
// Digital PIN Numbers
// --------------------------------------
#define ACCELERATION    13
#define BRAKE           12
#define MIXER           11
#define SPEED           10
#define DOWN            8
#define UP              9
#define LAMPS           7


// --------------------------------------
// Analog PIN Numbers
// --------------------------------------
#define LDR             0


// --------------------------------------
// Slope Legal Values
// --------------------------------------
typedef enum{flat = 0, down = 1, up = 2} slope_t;


// --------------------------------------
// Speed Limits
// --------------------------------------
#define SPEED_MIN   40.0
#define SPEED_MAX   70.0


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
bool lamps_is_active = false;
double speed = 55.5;
slope_t railway_slope = flat;
int ldr_value = 0;

unsigned long speed_last_measure_time;

// Scheduler stuff
unsigned long sc = 0; // secondary cycle
unsigned long exe_start_time;


// --------------------------------------
// Function: comm_server
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
        if (requested_answered)
            Serial.print(answer);
        else
            Serial.print(ANS_ERROR);

        // reset flags and buffers
        request_received = false;
        requested_answered = false;
        memset(request, '\0', MESSAGE_SIZE+1);
        memset(answer, '\0', MESSAGE_SIZE+1);
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
// Function: speed_req
// --------------------------------------
void speed_req()
{ 
    // If there is a request not answered, check if this is the one
    if ( !strcmp(REQ_SPEED, request) ) {  
        // send the answer for speed request
        char num_str[5];
        dtostrf(speed, 4,1, num_str);
        sprintf(answer, ANS_SPEED, num_str);

        // set request as answered
        requested_answered = true;
    }
}



// --------------------------------------
// Function: lit_req
// --------------------------------------
void lit_req()
{ 
    // If there is a request not answered, check if this is the one
    if ( !strcmp(REQ_LIT, request) ) {  
        // send the answer for lit request
        sprintf(answer, ANS_LIT, ldr_value);

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
    if ( !strcmp(REQ_SLOPE, request) ) {
        // send answer for read slope request
        switch (railway_slope) {
            case flat:
                sprintf(answer, ANS_SLOPE_FLAT);
                break;
            case down:
                sprintf(answer, ANS_SLOPE_DOWN);
                break;
            case up:
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

    set = strcmp(REQ_GAS_SET, (const char *) request);
    clr = strcmp(REQ_GAS_CLR, (const char *) request);

    // If there is a request not answered, check if this is the one
    if (!set || !clr) {
        gas_is_active = !set + clr;

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

    set = strcmp(REQ_BRK_SET, (const char *) request);
    clr = strcmp(REQ_BRK_CLR, (const char *) request);

    // If there is a request not answered, check if this is the one
    if (!set || !clr) {
        brake_is_active = !set + clr;

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

    set = strcmp(REQ_MIX_SET, (const char *) request);
    clr = strcmp(REQ_MIX_CLR, (const char *) request);

    // If there is a request not answered, check if this is the one
    if (!set || !clr) {
        mixer_is_active = !set + clr;

        // send answer for "activating mixer" request
        sprintf(answer, ANS_MIX_OK);
        // set request as answered
        requested_answered = true;
    }
}


// --------------------------------------
// Function: lamps_req
// --------------------------------------
void lamps_req()
{
    int set, clr;

    set = strcmp(REQ_LAM_SET, (const char *) request);
    clr = strcmp(REQ_LAM_CLR, (const char *) request);

    // If there is a request not answered, check if this is the one
    if (!set || !clr) {
        lamps_is_active = !set + clr;

        // send answer for "activating brake" request
        sprintf(answer, ANS_LAM_OK);
        // set request as answered
        requested_answered = true;
    }
}

// --------------------------------------
// Function: req_handler
// --------------------------------------
void req_handler()
{
    if (!request_received || requested_answered)
        return;

    speed_req();
    slope_req();
    gas_req();
    brake_req();
    mixer_req();
    lamps_req();
    lit_req();
}


// --------------------------------------
// Function: comm_server_wrapper
// Task: Communication Server
// Worst Compute Time: 9368 μs
// --------------------------------------
void comm_server_wrapper()
{
    comm_server();
    req_handler();
}


// --------------------------------------
// Function: acceleration_system
// Task: On/Off Accelerator
// Worst Compute Time: 16 μs
// --------------------------------------
void acceleration_system()
{
    digitalWrite(ACCELERATION, gas_is_active);
}


// --------------------------------------
// Function: brake_system
// Task: On/Off Brake
// Worst Compute Time: 16 μs
// --------------------------------------
void brake_system()
{
    digitalWrite(BRAKE, brake_is_active);
}


// --------------------------------------
// Function: mixer_system
// Task: On/Off Mixer
// Worst Compute Time: 16 μs
// --------------------------------------
void mixer_system()
{
    digitalWrite(MIXER, mixer_is_active);
}


// --------------------------------------
// Function: lamps_system
// Task: On/Off Lamps
// Worst Compute Time: 8 μs
// --------------------------------------
void lamps_system()
{
    digitalWrite(LAMPS, lamps_is_active);
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

    acceleration = MOD_GAS*gas_is_active + MOD_BRK*brake_is_active +
    MOD_SLOPE_DOWN*(railway_slope == down) + MOD_SLOPE_UP*(railway_slope == up);

    speed += (double) acceleration * ((double) delta/us);
}


// --------------------------------------
// Function: show_current_speed
// Task: Compute and Show Speed
// Worst Compute Time: 128 μs
// --------------------------------------
void show_current_speed()
{
    int pwm_value;

    // compute new speed first
    update_speed();

    // change speed LED intensity
    // if speed is out of bounds, the LED turns off
    pwm_value = ( speed != constrain(speed, SPEED_MIN, SPEED_MAX) ) ? 0
    : (int) map(speed, SPEED_MIN, SPEED_MAX, 0, 255);

    analogWrite(SPEED, pwm_value);
}


// --------------------------------------
// Function: read_slope
// Task: Read Slope
// Worst Compute Time: 12 μs
// --------------------------------------
void read_slope()
{
    // read slope pins and compute current slope
    railway_slope = (slope_t) (flat + down*digitalRead(DOWN) + up*digitalRead(UP));
}

// --------------------------------------
// Function: read_ldr
// Task: Read LDR value
// Worst Compute Time: 212 μs
// --------------------------------------
void read_ldr()
{
    // read LDR analog pin and return the value in percentage from 0 to 100
    int ldr_analog = analogRead(LDR);
    ldr_value = (int) map(ldr_analog, 0, 1024, 0, 100);
}


// --------------------------------------
// Function: setup
// --------------------------------------
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

    // Setup Serial Monitor
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
            lamps_system();                        
            break;
        case 2:
            read_slope();
            read_ldr();
            break;
        case 3:
            comm_server_wrapper();
            break;
    }

    sc = (sc + 1) % 4;
    exe_end_time = millis();

    // get elapsed execution time
    if (exe_start_time > exe_end_time) {
        Serial.println("overflow");
        delta = (unsigned long) (MAX_UNSIGNED_LONG - exe_start_time + exe_end_time);
    } else {
        delta = (unsigned long) (exe_end_time - exe_start_time);
    }

    // check if execution took too long
    if (SC_TIME < delta) {
        Serial.println("exit");
        exit(-1);
    }

    // wait until next secondary cycle
    delay(SC_TIME - delta+5);
    exe_start_time += SC_TIME;

}