#include <Bitcraze_PMW3901.h>
#include <HCSR04.h>
#define PMW3901_CS 10
#define DIST_TRIG 6
#define DIST_ECHO 7
#define UPDATE_SERIAL_WAIT 100

Bitcraze_PMW3901 onboard_flow_sensor(PMW3901_CS);

unsigned long lastUpdate = 0UL;
int frame[35*35];

void setup(){
  //Open up serial port and enable communication
  Serial.begin(115200);

  while(!Serial){
    delay(1);
  }

  if (!onboard_flow_sensor.begin()) {
    while(1){
      Serial.println("ERROR");
      delay(1000);
    }
  }
  onboard_flow_sensor.enableFrameBuffer();
  lastUpdate = millis();
}

void loop() {
  if(Serial.available() > 0){
    while(Serial.available() > 0){
      Serial.read();
      delay(UPDATE_SERIAL_WAIT);
    }
  }else{
    return;
  }

  Serial.println("Reading...");
  onboard_flow_sensor.readFrameBuffer((char*) frame);
  Serial.println("====================================");

  for (int i = 0; i < 35*35; i++){
    Serial.print(frame[i]);
    Serial.print(",");
  }
  Serial.println("====================================");
  Serial.flush();
  delay(50);
}