#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <Wire.h>
#include <BMI270.h>

#define MY_TRACKER_ID 0
#define NUM_TRACKERS 10
#define NUM_BASES 3
#define TRIG_PIN D5
#define ECHO_PIN D6

volatile uint32_t echoStart=0, echoEnd=0;
volatile bool newMeasurement=false;

float framerate=100;
uint32_t FRAME_US, SLOT_US;

WiFiUDP udp;
IPAddress trackerGroup(239,1,1,1);
int port=6969;
IPAddress routerIP(192,168,1,1);
int oscPort=9000;

struct XYZ{ float x,y,z; };
struct Quaternion{ float w,x,y,z; };

XYZ myPos;
XYZ myVel;
Quaternion myQuat;
XYZ baseCorrectedXYZ[NUM_BASES][NUM_TRACKERS];

BMI270 imu;
float lastFreq=40000;
float receivedFreq=40000;
float waveSpeed=343.0;
float dopplerFactor=0.5;

void IRAM_ATTR echoISR() {
  if(digitalRead(ECHO_PIN)) echoStart=micros();
  else { echoEnd=micros(); newMeasurement=true; }
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  Serial.begin(115200);
  Wire.begin();
  imu.begin();
  imu.setGyroRange(BMI270_GYRO_2000DPS);
  imu.setAccelRange(BMI270_ACCEL_8G);
  imu.setDlpfFreq(1000);

  WiFi.begin("asus_router","password");
  while(WiFi.status()!=WL_CONNECTED) delay(100);

  udp.beginMulticast(WiFi.localIP(), trackerGroup, port);

  FRAME_US=(uint32_t)(1e6/framerate);
  SLOT_US=FRAME_US/(NUM_TRACKERS+NUM_BASES);
}

void loop() {
  uint32_t frameStart=micros();
  uint32_t slot=(frameStart%FRAME_US)*(NUM_TRACKERS+NUM_BASES)/FRAME_US;

  if(slot==MY_TRACKER_ID){
    triggerPulse();
    if(newMeasurement) measureDistance();
    measureDoppler();
    readIMU();
    applyBaseCorrection();
    sendOSC();
  }

  receiveBasePackets();

  uint32_t frameEnd=micros();
  uint32_t elapsed=frameEnd-frameStart;
  uint32_t sleepTime=(elapsed<10000)?(10000-elapsed):0;
  FRAME_US=constrain(10000-sleepTime,3333,10000);
  framerate=1e6/FRAME_US;
  SLOT_US=FRAME_US/(NUM_TRACKERS+NUM_BASES);
}

void triggerPulse() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
}

void measureDistance() {
  long duration=echoEnd-echoStart;
  long d=duration*0.0343/2; // cm
  myPos.x=d; myPos.y=0; myPos.z=0;
  newMeasurement=false;
}

void measureDoppler() {
  float deltaF = receivedFreq - lastFreq;
  float v = deltaF * waveSpeed / lastFreq; // m/s
  myVel.x = dopplerFactor*myVel.x + (1-dopplerFactor)*v*100; // cm/frame
  myPos.x += myVel.x;
  lastFreq = receivedFreq;
}

void readIMU() {
  imu.readSensor();
  float gx = imu.getGyroX_rads();
  float gy = imu.getGyroY_rads();
  float gz = imu.getGyroZ_rads();
  float ax = imu.getAccelX_mss();
  float ay = imu.getAccelY_mss();
  float az = imu.getAccelZ_mss();
  myQuat.w = 1; myQuat.x = gx*0.001; myQuat.y = gy*0.001; myQuat.z = gz*0.001;
}

void applyBaseCorrection() {
  int count=0;
  float correctedX=myPos.x;
  for(int b=0;b<NUM_BASES;b++){
    if(baseCorrectedXYZ[b][MY_TRACKER_ID].x>0){
      correctedX=(correctedX + baseCorrectedXYZ[b][MY_TRACKER_ID].x)/2.0;
      count++;
    }
  }
  if(count>0) myPos.x=correctedX;
}

void receiveBasePackets() {
  int size=udp.parsePacket();
  if(size>0){
    XYZ baseReceived[NUM_BASES][NUM_TRACKERS];
    udp.read((uint8_t*)baseReceived,sizeof(baseReceived));
    for(int b=0;b<NUM_BASES;b++){
      for(int t=0;t<NUM_TRACKERS;t++){
        if(baseReceived[b][t].x>0)
          baseCorrectedXYZ[b][t]=baseReceived[b][t];
      }
    }
  }
}

void sendOSC() {
  OSCMessage msg("/tracker");
  msg.add((int32_t)MY_TRACKER_ID);
  msg.add(myPos.x); msg.add(myPos.y); msg.add(myPos.z);
  msg.add(myQuat.w); msg.add(myQuat.x); msg.add(myQuat.y); msg.add(myQuat.z);

  udp.beginPacket(routerIP,oscPort);
  msg.send(udp);
  udp.endPacket();
  msg.empty();
}
