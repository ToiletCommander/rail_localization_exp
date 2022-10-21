#include <Bitcraze_PMW3901.h>
#define PMW3901_CS 10
#define UPDATE_SERIAL_WAIT 5

Bitcraze_PMW3901 onboard_flow_sensor(PMW3901_CS);
unsigned long lastUpdate = 0UL;


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
  lastUpdate = millis();
}

void loop() {
  if(Serial.available() <= 0){
    return;
  }
  while(Serial.available() > 0){
    Serial.read(); //Read an arbitrary byte sent by the host
    delay(UPDATE_SERIAL_WAIT);
  }
  int16_t dx = 0, dy = 0;
  unsigned long ct = millis();
  unsigned long dt = ct - lastUpdate;

  onboard_flow_sensor.readMotionCount(&dx, &dy);

  Serial.print(dx);
  Serial.print(",");
  Serial.print(dy);
  Serial.print(",");
  Serial.println(dt);
  Serial.println();
  Serial.flush();
  lastUpdate = ct;
  //delay(UPDATE_FREQUENCY_MS);
}