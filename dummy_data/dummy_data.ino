void setup() {
  Serial.begin(115200); //Setup UART baud rate
}
/*************Define dummy data**********************
Sensor hub protocol
first packet
index(1 byte)
yaw(2 bytes)
pitch(2 bytes)
row(2 bytes)
x acceleration (2 bytes)
y acceleration (2 bytes)
z acceleration (2 bytes)
ultrasound 1 (1 byte)
ultrasound 2 (1 byte)
ultrasound 3 (1 byte)
ultrasound 4 (1 byte)
ultrasound 5 (1 byte)
ultrasound 6 (1 byte)
ultrasound 7 (1 byte)
 
Second packet
index(1 byte)
ultrasound 8 (1 byte)
PIR 1 (1 byte)
PIR 2 (1 byte)
Humidity (1 byte)
Indoor temperature(1 byte)
Outdoor temperature (1 byte)
13 bytes not yet define(0x0000000000000)
 */
int data[19]={-1234,-5678,1234,5678,1234,1234,1,2,3,4,5,6,7,8,0,2,58,23,30}; 


void loop() {
  //Packet 1 information
  Serial.write(byte(int(1)));
  for(int i=0;i<6;i++){
    Serial.write(byte(data[i]>>8));
    Serial.write(byte(data[i]));
  }
  for(int i=6;i<13;i++){
    Serial.write(byte(data[i]));
  }
//  Packet 2 information
  Serial.write(byte(int(2)));
  for(int i=13;i<19;i++){
    Serial.write(byte(data[i]));
  }
  for(int i=0;i<13;i++){
    Serial.write(byte(int(0)));
  }
  delay(1000);
}


