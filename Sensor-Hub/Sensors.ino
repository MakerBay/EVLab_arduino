//Copyright Statement for the I2C Dev lib
/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)                                  //Left in just in case
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)                                                 //Left in just in case
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.                            //Left in just in case
//#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT_PIN 2  // interrupt pin for mpu6050

// MPU control/status variables
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion variables
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 12, TXPin = 13;                 //Software serial pins for GPS
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial ss(RXPin, TXPin);

// A custom version of delay():
// Used in timing GPS
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

#include "DHT.h"

#define DHTPIN 11                //DHT Sensor pin
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

//Ultrasonic calculation variables:
unsigned long duration_1;                    //in microseconds
unsigned long duration_2;
unsigned long duration_3;
unsigned long duration_4;
unsigned long duration_5;
unsigned long duration_6;
unsigned long duration_7;
unsigned long duration_8;

//Ultrasonic pin variables:
int trig_1 = 22;
int trig_2 = 24;
int trig_3 = 26;
int trig_4 = 28;
int trig_5 = 30;
int trig_6 = 32;                                           //For US-015
int trig_7 = 34;
int trig_8 = 36;
int echo_1 = 23;
int echo_2 = 25;
int echo_3 = 27;
int echo_4 = 29;
int echo_5 = 31;
int echo_6 = 33;                                           //For US-015
int echo_7 = 35;
int echo_8 = 37;

//PIR sensor pin:
int PIRsensor_1 = 6;
int PIRsensor_2 = 7;

//Ultrasonic sensor range variables (unit:cm):
//Works up to about 1.5 meters; if no object detected within 1.5 meters, returns 0
float range_1;
float range_2;
float range_3;
float range_4;
float range_5;
float range_6;
float range_7;
float range_8;

//PIR sensor variable:
bool PIR_1;
bool PIR_2;

//DHT sensor variable:
float humidity;
float temperature;

//Counter variable:
long i = 0;

#include <Ultrasonic.h>

// Setting up the ultrasonic sensors (HC-SR04):
Ultrasonic ultrasonic_1(trig_1, echo_1);
Ultrasonic ultrasonic_2(trig_2, echo_2);
Ultrasonic ultrasonic_3(trig_3, echo_3);
Ultrasonic ultrasonic_4(trig_4, echo_4);
Ultrasonic ultrasonic_5(trig_5, echo_5);
Ultrasonic ultrasonic_7(trig_7, echo_7);
Ultrasonic ultrasonic_8(trig_8, echo_8);

void setup() {

  ss.begin(GPSBaud);                                                      //Setting up the GPS

  // Setting up MPU6050
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication (Baud rate subject to change)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Gyro and accelerometer offsets subject to further tuning:
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  dht.begin();                                //Setting up the DHT

  //Initialise pins for ultrasonic sensors:
  pinMode(trig_1, OUTPUT);
  pinMode(echo_1, INPUT);
  pinMode(trig_2, OUTPUT);
  pinMode(echo_2, INPUT);
  pinMode(trig_3, OUTPUT);
  pinMode(echo_3, INPUT);
  pinMode(trig_4, OUTPUT);
  pinMode(echo_4, INPUT);
  pinMode(trig_5, OUTPUT);
  pinMode(echo_5, INPUT);
  pinMode(trig_6, OUTPUT);
  pinMode(echo_6, INPUT);
  pinMode(trig_7, OUTPUT);
  pinMode(echo_7, INPUT);
  pinMode(trig_8, OUTPUT);
  pinMode(echo_8, INPUT);

}

void loop() {
  // Getting data from mpu6050
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Reading from PIR Sensor:
    PIR_1 = digitalRead(PIRsensor_1);
    PIR_2 = digitalRead(PIRsensor_2);

    // Send a 10us pulse to sensors
    if (i % 8 == 0) {
      digitalWrite(trig_1, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_1, LOW);
    }
    if (i % 8 == 1) {
      digitalWrite(trig_2, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_2, LOW);
    }
    if (i % 8 == 2) {
      digitalWrite(trig_3, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_3, LOW);
    }
    if (i % 8 == 3) {
      digitalWrite(trig_4, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_4, LOW);
    }
    if (i % 8 == 4) {
      digitalWrite(trig_5, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_5, LOW);
    }
    if (i % 8 == 5) {
      digitalWrite(trig_6, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_6, LOW);
    }
    if (i % 8 == 6) {
      digitalWrite(trig_7, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_7, LOW);
    }
    if (i % 8 == 7) {
      digitalWrite(trig_8, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig_8, LOW);
    }

    // Read the duration from 1 of the ulrasonic sensors, and calculate the distance in cm
    // Then put the result into range variable
    // On the next pass, data from another sensor will be read, so we don't slow down the loop too much that the FIFO of mpu6050 overflows

    if (i % 8 == 0) {
      duration_1 = pulseIn(echo_1, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_1 = ultrasonic_1.convert(duration_4, Ultrasonic::CM);                                                      // Work out the range in cm
    }
    if (i % 8 == 1) {
      duration_2 = pulseIn(echo_2, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_2 = ultrasonic_2.convert(duration_4, Ultrasonic::CM);                                                      // Work out the range in cm
    }
    if (i % 8 == 2) {
      duration_3 = pulseIn(echo_3, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_3 = ultrasonic_3.convert(duration_4, Ultrasonic::CM);                                                      // Work out the range in cm
    }
    if (i % 8 == 3) {
      duration_4 = pulseIn(echo_4, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_4 = ultrasonic_4.convert(duration_4, Ultrasonic::CM);                             // Work out the range in cm
    }
    if (i % 8 == 4) {
      duration_5 = pulseIn(echo_5, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_4 = ultrasonic_5.convert(duration_4, Ultrasonic::CM);                                                       // Work out the range in cm
    }
    if (i % 8 == 5) {
      duration_6 = pulseIn(echo_6, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_6 = (duration_6 * 50) / 3026;                                                   // Work out the range in cm (Specially for US-15 sensors)
    }
    if (i % 8 == 6) {
      duration_7 = pulseIn(echo_7, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_4 = ultrasonic_7.convert(duration_4, Ultrasonic::CM);                                                      // Work out the range in cm
    }
    if (i % 8 == 7) {
      duration_8 = pulseIn(echo_8, HIGH, 81600);                                            // Timeout set at 8816 us, appropriate for a 3 meter round trip;
      range_4 = ultrasonic_8.convert(duration_4, Ultrasonic::CM);                                                      // Work out the range in cm
    }

    // Reading from DHT sensor: (Updates every two seconds)
    // Reading temperature or humidity takes about 250 milliseconds
    if (i % 255 == 0 ) {
      humidity = dht.readHumidity();
      // Read temperature as Celsius
      temperature = dht.readTemperature();
    }

#ifdef OUTPUT_READABLE_QUATERNION                          //Left in just in case
    // display quaternion values in easy matrix form: w x y z
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    Serial.print("quat\t");
    Serial.print(q.w);
    Serial.print("\t");
    Serial.print(q.x);
    Serial.print("\t");
    Serial.print(q.y);
    Serial.print("\t");
    Serial.println(q.z);
#endif

#ifdef OUTPUT_READABLE_EULER                                //Left in just in case
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler\t");
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(", Yaw: ");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print(", Pitch: ");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print(", Roll: ");
    Serial.print(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print(", Acceleration_x: ");
    Serial.print(aaReal.x);
    Serial.print(", Acceleration_y: ");
    Serial.print(aaReal.y);
    Serial.print(", Acceleration_z: ");
    Serial.print(aaReal.z);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL                                            //left in just in case
    // display initial world-frame acceleration, adjusted to remove gravity
    // and rotated based on known orientation from quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
    Serial.print("aworld\t");
    Serial.print(aaWorld.x);
    Serial.print("\t");
    Serial.print(aaWorld.y);
    Serial.print("\t");
    Serial.print(aaWorld.z);
#endif

    Serial.print(", Ultrasonic_1: ");                      //Print everything to serial
    Serial.print(range_1);
    Serial.print(", Ultrasonic_2: ");
    Serial.print(range_2);
    Serial.print(", Ultrasonic_3: ");
    Serial.print(range_3);
    Serial.print(", Ultrasonic_4: ");
    Serial.print(range_4);
    Serial.print(", Ultrasonic_5: ");
    Serial.print(range_5);
    Serial.print(", Ultrasonic_6: ");
    Serial.print(range_6);
    Serial.print(", Ultrasonic_7: ");
    Serial.print(range_7);
    Serial.print(", Ultrasonic_8: ");
    Serial.print(range_8);
    Serial.print(", PIR_1: ");
    Serial.print(PIR_1);
    Serial.print(", PIR_2: ");
    Serial.print(PIR_2);

    Serial.print("Latitude(in deg)");
    printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    Serial.print(", Longitude(in deg): ");
    printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    Serial.print(", Land Speed(in km/h): ");
    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
    Serial.print(", Heading:");
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);

    Serial.print(", Humidity: ");
    Serial.print(humidity);
    Serial.print(", Temperature:");
    Serial.print(temperature);
    Serial.print(", ");

    Serial.println(i);

    i += 1;
  }
}
