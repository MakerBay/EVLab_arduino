/* Pin Map **************************
  Pin 0, 1: TXD / RXD - For debug purpose
  Pin 2 : RC ch1
  Pin 3 : RC ch2
  Pin 4 : RC ch3
  Pin 5 : Brake Stepper Direction
  Pin 6 : BrakeStepper Pulse
  Pin 7 : Direction relay (front)
  Pin 8 : Steering Stepper Direction
  Pin 9 : Steering Stepper Pulse
  Pin 11 : Pot Servo
  Pin 12 : Direction relay (Back)
  Pin 13 : Main relay
  Pin A0 : Steering feedback (linear pot)
*/

//Steering control
#include "TimerOne.h"
#include "PID_v1.h"
#define ster_step_dir_pin 8
#define ster_step_puls_pin 9
#define deadzone_steering 40
#define steering_range 1200

//Speed control
#define pot_pin A0
#define deadzone_throttle 80
#define Front_relay 7 // FRONT - arbitrary pin value
#define Back_relay 12 // BACK - arbitrary pin value
#define Main_relay 13
int rval; // Output variable to servo

//ServoTimer2
#include <ServoTimer2.h>
ServoTimer2 servo1;
#define degreesToUS( _degrees) (_degrees * 6 + 900)
#define Servopin 11

//Communication protocol
#define serial_control
//#define RC_control
#define RC_ch1 2
#define RC_ch2 3
#define RC_ch3 4
int ch1 = 512; //start with the mid-point values
int ch2 = 512;
int ch3 = 0;

//Brake control
#include <EEPROM.h>
int addr = 0;
int brake_curr = 0;
#define brak_step_dir_pin 5
#define brak_step_puls_pin 6
#define brak_step_max 500

int steering;
int brakes;
int throttle;
byte fanSpeed;
byte indicator;
int LEDDisplay;

//PID set-up:
double setpoint_1, input_1, output_1;
double Kp_1 = 5, Ki_1 = 0, Kd_1 = 0.3; //Subject to tuning
PID steeringPID(&input_1, &output_1, &setpoint_1, Kp_1, Ki_1, Kd_1, DIRECT);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pot_pin, INPUT);
  pinMode(ster_step_dir_pin, OUTPUT);
  pinMode(ster_step_puls_pin, OUTPUT);

  pinMode(Front_relay, OUTPUT); // send output to front relay
  pinMode(Back_relay, OUTPUT); // send output to back relay

  servo1.attach(Servopin);

  steeringPID.SetOutputLimits(-steering_range, steering_range);
  steeringPID.SetMode(AUTOMATIC);

  brake_curr = EEPROM.read(addr);
  while (Serial.available() > 0) {
    Serial.read();
  }
}

String msg = "";

void loop() {
#ifdef serial_control
  start:
  while (Serial.available() > 0) {
    msg += (char) Serial.read();
    //Serial.println(msg);
    if (msg.length() >= 20){
      break;
    }
  }
  if (msg.length() == 20) {
    Serial.println(msg);
    String serialTemp;
    int serialTempInt[4];
    for (int i = 0; i < 4; i++) {
      serialTemp = msg.substring(i * 2, i * 2 + 2);
      serialTempInt[i] = CharToInt(serialTemp[0]) * 16 + CharToInt(serialTemp[1]);
    }
    //Serial.println((serialTempInt[0] + serialTempInt[1] + serialTempInt[2]) % 256);
    //Serial.println(serialTempInt[3]);
    if (((serialTempInt[0] + serialTempInt[1] + serialTempInt[2]) % 256) == serialTempInt[3] && msg[18] == '2' && msg[19] == '3') {
      steering = map(serialTempInt[0], 0, 255, 0, 1023);
      throttle = map(serialTempInt[1], 0, 255, 0, 1023);
      brakes = map(serialTempInt[2], 0, 255, 0, 1023);
      serialTemp = msg.substring(8, 10);
      fanSpeed = CharToInt(serialTemp[0]) * 16 + CharToInt(serialTemp[1]);
      serialTemp = msg.substring(10, 12);
      indicator = CharToInt(serialTemp[0]) * 16 + CharToInt(serialTemp[1]);
      serialTemp = msg.substring(12, 16);
      LEDDisplay = CharToInt(serialTemp[0]) * 16 * 16 * 16 + CharToInt(serialTemp[1]) * 16 * 16 + CharToInt(serialTemp[2]) * 16 + CharToInt(serialTemp[3]);
      Serial.print("steering: ");
      Serial.print(steering);
      Serial.print(", ");
      Serial.print("throttle: ");
      Serial.print(throttle);
      Serial.print(",");
      Serial.print("brakes: ");
      Serial.print(brakes);
      Serial.print(", ");
      Serial.print("fanSpeed: ");
      Serial.print(fanSpeed);
      Serial.print(", ");
      Serial.print("indicator: ");
      Serial.print(indicator);
      Serial.print(", ");
      Serial.print("LEDDisplay: ");
      Serial.print(LEDDisplay);
      Serial.print(", ");
    }
    else {
      msg = "";
      while(Serial.read()!='3'){
        Serial.read();  
      }
      Serial.println("Invalid msg");
      goto start;
    }
  }
  else {
    goto start;
  }
#endif
  
  
  Serial.println("Start of control code");
  //Direction control
  if (throttle > (512 + deadzone_throttle)) {
    digitalWrite(Main_relay, HIGH);
    digitalWrite(Back_relay, LOW); // Turns ON Relays front
    digitalWrite(Front_relay, HIGH); // Turns OFF Relay back
  }
  else if (throttle < (512 - deadzone_throttle)) {
    digitalWrite(Main_relay, HIGH);
    digitalWrite(Front_relay, LOW); // Turns OFF Relays front
    digitalWrite(Back_relay, HIGH); // Turns ON Relay back
  }
  else {
    digitalWrite(Main_relay, LOW);
    digitalWrite(Front_relay, LOW);
    digitalWrite(Back_relay, LOW);
  }

  //Speed control
  rval = abs(throttle - 512);
  rval = map(rval, 0, 512, 0, 256); //5K pot used
  servo1.write(degreesToUS(rval));

  //Steering control
  int pot_raw = analogRead(pot_pin);
  pot_raw = map(pot_raw, 0, 680, 0, 1023);

  //PID
  input_1 = pot_raw;
  setpoint_1 = steering;
  steeringPID.Compute();

  output_1 > 0 ? digitalWrite(ster_step_dir_pin, HIGH) : digitalWrite(ster_step_dir_pin, LOW);

  long period = steering_range - abs(output_1);
  period = (period < 320) ? 320 : period;
  period = abs(period);
  //set period
  Timer1.detachInterrupt();
  if (abs(setpoint_1 - pot_raw) > deadzone_steering) {
    Timer1.initialize(period);
    Timer1.attachInterrupt(callback);
  }

  //Brake control
  //Mapping need to be done;
  brakes - brake_curr > 0 ? digitalWrite(brak_step_dir_pin, HIGH) : digitalWrite(brak_step_dir_pin, LOW);
  for (int i = 0; i < abs(ch3 - brake_curr); i++) {
    digitalWrite(brak_step_puls_pin, HIGH);
    delayMicroseconds(300);
    digitalWrite(brak_step_puls_pin, LOW);
    delayMicroseconds(100);
  }
  EEPROM.write(addr, brake_curr);

  //Print data for debugging and testing
  Serial.print("Set point: ");
  Serial.print(setpoint_1);
  Serial.print(", ");
  Serial.print("Input: ");
  Serial.print(pot_raw);
  Serial.print(", ");
  Serial.print("Output: ");
  Serial.print(output_1);
  Serial.print(", ");
  Serial.print("Period: ");
  Serial.print(period);
  Serial.print(", ");
  Serial.print("rval: ");
  Serial.print(rval);
  Serial.print(",");
  Serial.println(brake_curr);
  msg="";
  delay(1);
}

void callback()
{
  digitalWrite(ster_step_puls_pin, digitalRead(ster_step_puls_pin) ^ 1);
}

int CharToInt(char temp) {
  if (temp <= '9') {
    return temp - '0';
  }
  else {
    return (temp - 0x41) + 10;
  }
}

