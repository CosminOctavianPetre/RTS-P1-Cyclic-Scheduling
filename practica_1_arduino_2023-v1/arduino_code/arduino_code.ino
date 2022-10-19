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

void show_current_speed(){
    if (speed <= 40){
      analogWrite(SPEED, 0);
    }else if (speed >= 70){
      analogWrite(SPEED,255);
    }else{
      int pwm_value = (int) ((speed-40)*255)/30;
      analogWrite(SPEED, pwm_value);
    }
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

    // Setup Serial Monitor
    Serial.begin(9600);
}

// --------------------------------------
// Function: loop
// --------------------------------------
void loop()
{
    unsigned long time_exec_begin, time_exec_end, elapsed;
    time_exec_begin = micros();
    //compute time code    
    show_current_speed();




    time_exec_end = micros();
    elapsed = time_exec_end - time_exec_begin;
    Serial.println(elapsed);
    delay(1000);
    speed += 0.25;
    // comm_server();
    // speed_req();
    
}
