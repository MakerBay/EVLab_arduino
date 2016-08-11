#include <PID_v1.h>

#include "TimerOne.h"

#define stepper_dir_pin 8   //HIGH as anti-clockwise
#define stepper_pulse_pin 9

#define pot_pin A0
#define deadzone_steering 40
#define deadzone_throttle 80

#include <DigiPotX9Cxxx.h>                                            //10K - 100 steps
#define Front_relay  7                                                // FRONT - arbitrary pin value                   
#define Back_relay  12                                               // BACK - arbitrary pin value
#define Main_relay  13

#define brake_dir 5
#define brake_pulse  9

#define RC_ch1  2
#define RC_ch2  3
#define RC_ch3  4

DigiPot pot(2, 3, 4);

#define serial_control
//#define RC_control

String packet;


//PID set-up:
double setpoint_1, input_1, output_1;
double Kp_1 = 5, Ki_1 = 0, Kd_1 = 0.3;                                                //Subject to tuning
PID steeringPID(&input_1, &output_1, &setpoint_1, Kp_1, Ki_1, Kd_1, DIRECT);

// Output variable to digipot
int rval;

int ch1 = 512;                    // Steeringstart with the mid-point values
int ch2 = 512;                    // Throttle
int ch3 = 0;                      // Brakes

void setup() {
  Serial.begin(9600);
  pinMode(pot_pin, INPUT);
  pinMode(stepper_dir_pin, OUTPUT);
  pinMode(stepper_pulse_pin, OUTPUT);

  pinMode(RC_ch1, INPUT);
  pinMode(RC_ch2, INPUT);
  pinMode(RC_ch3, INPUT);

  pinMode(Front_relay, OUTPUT);                                    // send output to front relay
  pinMode(Back_relay, OUTPUT);                                    // send output to back relay

  steeringPID.SetOutputLimits(-1200, 1200);
  steeringPID.SetMode(AUTOMATIC);
}

void loop() {
  //Get target value from serial
#ifdef serial_control
  if (Serial.available() > 0) {
    /*packet = '\n';
      while (Serial.available() > 0)  {
      packet += (char(Serial.read()));
      if (Serial.peek () == '#') {
        break;
      }
      }*/
    ch1 = Serial.parseInt();
    ch2 = Serial.parseInt();
    ch3 = Serial.parseInt();
    if (Serial.read() == '#') {
      ch1 = map(ch1, 0, 255, 0, 1023);
      ch2 = map(ch2, 0, 255, 0, 1023);
      ch3 = map(ch3, 0, 255, 0, 1023);
      Serial.print("ch1: ");
      Serial.print(ch1);
      Serial.print(", ");
      Serial.print("ch2: ");
      Serial.print(ch2);
      Serial.print(",");
      Serial.print("ch3: ");
      Serial.print(ch3);
      Serial.print(", ");
    }
  }

  /*
    for(int i=0;i<packet.length();i++){
    Serial.write(packet[i]);
    }*/



#endif

#ifdef RC_control
  //Get target values from RF controller:
  ch1 = pulseIn (RC_ch1, HIGH);
  ch2 = pulseIn (RC_ch2, HIGH);
  ch3 = pulseIn (RC_ch3, HIGH);
  constrain(ch1, 1128, 1734);
  constrain(ch2, 1132, 1939);
  constrain(ch3, 1087, 1881);
  ch1 = map(ch1, 1216, 1520, 0, 1023);                    //ch1 - steering: <512 = right
  ch2 = map(ch2, 1937, 1130, 0, 1023);                    //ch2 - throttle: <512 = backwards
  ch3 = map(ch3, 1881, 1087, 0, 1023);                    //ch3 - brakes
#endif

  //set direction first before applying acceleration
  if (ch2 > (512 + deadzone_throttle)) {
    digitalWrite(Main_relay, HIGH);
    digitalWrite(Back_relay, LOW);                           // Turns ON Relays front
    digitalWrite(Front_relay, HIGH);                           // Turns OFF Relay back
  }
  else if (ch2 < (512 - deadzone_throttle)) {
    digitalWrite(Main_relay, HIGH);
    digitalWrite(Front_relay, LOW);                           // Turns OFF Relays front
    digitalWrite(Back_relay, HIGH);                         // Turns ON Relay back
  }
  else {
    digitalWrite(Main_relay, LOW);
    digitalWrite(Front_relay, LOW);
    digitalWrite(Back_relay, LOW);
  }

  rval = abs(ch2 - 512);
  rval = map(rval, 0, 512, 0, 5000);
  pot.set(rval);

  //Get pot value
  int pot_raw = analogRead(pot_pin);
  pot_raw = map(pot_raw, 0, 680, 0, 1023);
  //Upper & lower bound
  //pot_raw = (pot_raw > pot_Max)? pot_Max: pot_raw;
  //pot_raw = (pot_raw < pot_Min)? pot_Min: pot_raw;

  //PID
  /*float output = (target_val-pot_raw)*Kp+(output-prev_output)*Kd;
    prev_output=output;
    //scale interrupt perion
  */

  input_1 = pot_raw;
  setpoint_1 = ch1;
  steeringPID.Compute();

  output_1 > 0 ? digitalWrite(stepper_dir_pin, HIGH) : digitalWrite(stepper_dir_pin, LOW);

  long period = 1500 - abs(output_1);
  period = (period < 320) ? 320 : period;
  period = abs(period);
  //set period
  Timer1.detachInterrupt();
  if (abs(setpoint_1 - pot_raw) > deadzone_steering) {
    Timer1.initialize(period);
    Timer1.attachInterrupt(callback);
  }

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
  Serial.println(rval);
}

void callback()
{
  digitalWrite(stepper_pulse_pin, digitalRead(stepper_pulse_pin) ^ 1);
}
