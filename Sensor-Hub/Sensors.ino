#include <Ultrasonic.h>

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
int UltrasonicTrig[8] = {22, 24, 26, 28, 30, 32, 34, 36};
int UltrasonicEcho[8] = {23, 25, 27, 29, 31, 33, 35, 37};

//PIR sensor pin:
int PIRsensor_1 = 6;
int PIRsensor_2 = 7;

//Ultrasonic sensor range variables (unit:cm):
//Works up to about 1.5 meters; if no object detected within 1.5 meters, returns 0
float UltrasonicRange[8];
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
Ultrasonic ultrasonic_1(UltrasonicTrig[0], UltrasonicEcho[0]);
Ultrasonic ultrasonic_2(UltrasonicTrig[1], UltrasonicEcho[1]);
Ultrasonic ultrasonic_3(UltrasonicTrig[2], UltrasonicEcho[2]);
Ultrasonic ultrasonic_4(UltrasonicTrig[3], UltrasonicEcho[3]);
Ultrasonic ultrasonic_5(UltrasonicTrig[4], UltrasonicEcho[4]);
Ultrasonic ultrasonic_7(UltrasonicTrig[6], UltrasonicEcho[6]);
Ultrasonic ultrasonic_8(UltrasonicTrig[7], UltrasonicEcho[7]);

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
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // Gyro and accelerometer offsets subject to further tuning:
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);


  if (devStatus == 0) {                                                             // make sure it worked (returns 0 if so)
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
  }

  dht.begin();                                //Setting up the DHT

  //Initialise pins for ultrasonic sensors:
  for (int i = 0; i < 8; i++) {
    pinMode(UltrasonicTrig[i], OUTPUT);
    pinMode(UltrasonicEcho[i], INPUT);

  }
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
    //Serial.println(F("FIFO overflow!"));

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

    // Send a 10us pulse to Ultrasonic sensor
    digitalWrite(UltrasonicTrig[i % 8], HIGH);
    delayMicroseconds(10);
    digitalWrite(UltrasonicTrig[i % 8], LOW);

    // Read the duration from 1 of the ulrasonic sensors, and calculate the distance in cm
    // Then put the result into range variable
    // On the next pass, data from another sensor will be read, so we don't slow down the loop too much that the FIFO of mpu6050 overflows

    unsigned long UltrasonicDuration = pulseIn(UltrasonicEcho[i % 8], HIGH, 81600);
    if (i % 8 != 5) {
      UltrasonicRange[i % 8] = ultrasonic_1.convert(UltrasonicDuration, Ultrasonic::CM);
    }
    else {
      UltrasonicRange[5] = (UltrasonicDuration * 50) / 3026;
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
    // Serial.print("euler\t");
    // Serial.print(euler[0] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.print(euler[1] * 180 / M_PI);
    // Serial.print("\t");
    // Serial.println(euler[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    //Serial.print(", Yaw: ");
    int IMUtemp;
    IMUtemp = ypr[0] * 180 / M_PI;
    Serial.write(IMUtemp);
    //Serial.print(", Pitch: ");
    IMUtemp = ypr[1] * 180 / M_PI;
    Serial.print(IMUtemp);
    //Serial.print(", Roll: ");
    IMUtemp = ypr[2] * 180 / M_PI;
    Serial.print(IMUtemp);
#endif

#ifdef OUTPUT_READABLE_REALACCEL
    // display real acceleration, adjusted to remove gravity
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    //Serial.print(", Acceleration_x: ");
    Serial.print(aaReal.x);
    //Serial.print(", Acceleration_y: ");
    Serial.print(aaReal.y);
    //Serial.print(", Acceleration_z: ");
    Serial.print(aaReal.z);
#endif


    Serial.print((int) range_1);
    Serial.print((int) range_2);
    Serial.print((int) range_3);
    Serial.print((int) range_4);
    Serial.print((int) range_5);
    Serial.print((int) range_6);
    Serial.print((int) range_7);
    Serial.print((int) range_8);

    //Output PIR value
    Serial.write((int) PIR_1);
    Serial.write((int) PIR_2);

    //Output environment value
    Serial.print((int) humidity);
    Serial.print((int) temperature);
    //Serial.println();
    //Serial.println(i);
    i += 1;
  }
}


