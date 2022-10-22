// --------------------------------------
// Include files
// --------------------------------------
#include <string.h>
#include <stdio.h>
#include <Wire.h>

// --------------------------------------
// Global Constants
// --------------------------------------
#define SLAVE_ADDR 0x8
#define MESSAGE_SIZE 8
#define us 1000000

// --------------------------------------
// PIN numbers
// --------------------------------------
#define ACCELERATION 13
#define BRAKE 12
#define MIXER 11
#define SPEED 10
#define DOWN 8
#define UP 9

// --------------------------------------
// Global Variables
// --------------------------------------
double speed = 55.5;
bool request_received = false;
bool requested_answered = false;
char request[MESSAGE_SIZE+1];
char answer[MESSAGE_SIZE+1];

bool acceleration_is_active = false;
bool brake_is_active = false;
bool mixer_is_active = false;

int railway_slope = 0; //0 = Flat, 1 = Down, 2 = Up
unsigned long speed_last_measure_time;


// --------------------------------------
// Function: comm_server
// --------------------------------------
int comm_server()
{
    static int count = 0;
    char car_aux;

    // If there were a received msg, send the processed answer or ERROR if none.
    // then reset for the next request.
    // NOTE: this requires that between two calls of com_server all possible 
    //       answers have been processed.
    if (request_received) {
        // if there is an answer send it, else error
        if (requested_answered) {
            Serial.print(answer);
        } else {
            Serial.print("MSG: ERR\n");
        }  
        // reset flags and buffers
        request_received = false;
        requested_answered = false;
        memset(request,'\0', MESSAGE_SIZE+1);
        memset(answer,'\0', MESSAGE_SIZE+1);
    }

    while (Serial.available()) {
        // read one character
        car_aux =Serial.read();

        //skip if it is not a valid character
        if  ( ( (car_aux < 'A') || (car_aux > 'Z') ) &&
            (car_aux != ':') && (car_aux != ' ') && (car_aux != '\n') ) {
            continue;
        }

        //Store the character
        request[count] = car_aux;

        // If the last character is an enter or
        // There are 9th characters set an enter and finish.
        if ( (request[count] == '\n') || (count == 8) ) {
            request[count] = '\n';
            count = 0;
            request_received = true;
            break;
        }

        // Increment the count
        count++;
    }
}


// --------------------------------------
// Function: speed_req
// --------------------------------------
int speed_req()
{
    // If there is a request not answered, check if this is the one
    if ( (request_received) && (!requested_answered) && 
        (0 == strcmp("SPD: REQ\n",request)) ) {

        // send the answer for speed request
        char num_str[5];
        dtostrf(speed,4,1,num_str);
        sprintf(answer,"SPD:%s\n",num_str);

        // set request as answered
        requested_answered = true;
    }
    return 0;
}


// --------------------------------------
// Function: acceleration_system
// --------------------------------------
void acceleration_system()
{
    // Worst compute time 16 MICROseconds not miliseconds
    digitalWrite(ACCELERATION, acceleration_is_active);
}


// --------------------------------------
// Function: brake_system
// --------------------------------------
void brake_system()
{
    // Worst compute time 
    digitalWrite(BRAKE, brake_is_active);
}


// --------------------------------------
// Function: mixer_system
// --------------------------------------
void mixer_system()
{
    // Worst compute time 
    digitalWrite(MIXER, mixer_is_active);
}


// --------------------------------------
// Function: read_slope
// --------------------------------------
void read_slope()
{
    int up = digitalRead(UP);
    int down = digitalRead(DOWN);
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
    unsigned long time_new, delta;
    double acceleration;

    time_new = micros();
    //TODO: handle overflow
    delta = time_new - speed_last_measure_time;
    speed_last_measure_time = time_new;

    acceleration = 0.5 * acceleration_is_active - 0.5 * brake_is_active;
    if (railway_slope)
        acceleration += (railway_slope == 1) ? 0.25 : -0.25;

    speed += (double) acceleration * (delta/us);
}


// --------------------------------------
// Function: show_current_speed
// --------------------------------------
void show_current_speed()
{
    update_speed();

    if (speed <= 40){
      analogWrite(SPEED, 0);
    }else if (speed >= 70){
      analogWrite(SPEED,255);
    }else{
      int pwm_value = (int) ((speed-40)*255)/30;
      analogWrite(SPEED, pwm_value);
    }

    // send the answer for speed request
    char speed_str_tmp[5], speed_str[16];
    strcpy(speed_str, "speed: ");

    dtostrf(speed, 2,2, speed_str_tmp);
    sprintf(speed_str, "%s %s", speed_str, speed_str_tmp);
    Serial.println(speed_str);
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
    pinMode(DOWN, INPUT);
    pinMode(UP, INPUT);
    pinMode(SPEED, OUTPUT);

    // Setup Serial Monitor
    Serial.begin(9600);

    speed_last_measure_time = micros();
}


// --------------------------------------
// Function: loop
// --------------------------------------
void loop()
{
    unsigned long time_exec_begin, time_exec_end, elapsed;

    read_slope();
    time_exec_begin = micros();
    //compute time code    
    show_current_speed();

    time_exec_end = micros();
    elapsed = time_exec_end - time_exec_begin;
    // Serial.println(elapsed);
    delay(1000);
    // speed += 1;
    // if (speed >= 70) speed = 40;
    // comm_server();
    // speed_req();
    
}
