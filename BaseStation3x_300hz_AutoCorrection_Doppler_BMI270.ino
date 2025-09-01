#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define MY_BASE_ID 0       // 0,1,2
#define NUM_BASES 3
#define NUM_TRACKERS 10
#define TRIG_PIN D5
#define ECHO_PIN D6

volatile uint32_t echoStart=0, echoEnd=0;
volatile bool newMeasurement=false;

float framerate=100;
uint32_t FRAME_US, SLOT_US;

WiFiUDP udp;
IPAddress trackerGroup(239,1,1,1);
int port=6969;

struct XYZ{ float x,y,z; };
XYZ trackerXYZ[NUM_TRACKERS];       // broadcasted absolute positions
XYZ baseMeasuredXYZ[NUM_TRACKERS];  // base ToF measurements

void IRAM_ATTR echoISR() {
  if(digitalRead(ECHO_PIN)) echoStart=micros();
  else { echoEnd=micros(); newMeasurement=true; }
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoISR, CHANGE);

  WiFi.begin("asus_router","password");
  while(WiFi.status()!=WL_CONNECTED) delay(100);

  udp.beginMulticast(WiFi.localIP(), trackerGroup, port);

  FRAME_US=(uint32_t)(1e6/framerate);
  SLOT_US=FRAME_US/(NUM_TRACKERS+NUM_BASES);
}

void loop() {
  uint32_t frameStart=micros();
  uint32_t slot=(frameStart%FRAME_US)*(NUM_TRACKERS+NUM_BASES)/FRAME_US;

  if(slot==NUM_TRACKERS+MY_BASE_ID){
    triggerPulse();
    if(newMeasurement) measureTrackers();
    broadcastCorrectedXYZ();
    receiveTrackerPackets();
  }

  // dynamic framerate
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

void measureTrackers() {
  if(newMeasurement){
    long duration=echoEnd-echoStart;
    long d=duration*0.0343/2; // cm
    for(int i=0;i<NUM_TRACKERS;i++){
      baseMeasuredXYZ[i].x=d;
      baseMeasuredXYZ[i].y=0;
      baseMeasuredXYZ[i].z=0;
      if(trackerXYZ[i].x>0)
        trackerXYZ[i].x=(trackerXYZ[i].x+baseMeasuredXYZ[i].x)/2.0;
    }
    newMeasurement=false;
  }
}

void receiveTrackerPackets() {
  int size=udp.parsePacket();
  if(size>0){
    XYZ trackerReceived[NUM_TRACKERS];
    udp.read((uint8_t*)trackerReceived,sizeof(trackerReceived));
    for(int i=0;i<NUM_TRACKERS;i++){
      if(trackerReceived[i].x>0)
        trackerXYZ[i]=trackerReceived[i];
    }
  }
}

void broadcastCorrectedXYZ() {
  udp.beginPacketMulticast(trackerGroup, port, WiFi.localIP());
  udp.write((uint8_t*)&trackerXYZ,sizeof(trackerXYZ));
  udp.endPacket();
}
