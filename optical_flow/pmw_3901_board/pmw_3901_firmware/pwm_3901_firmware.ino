#include <Bitcraze_PMW3901.h>
#include <HCSR04.h>
#define PMW3901_CS 10
#define DIST_TRIG 6
#define DIST_ECHO 7
#define UPDATE_SERIAL_WAIT 5

//https://github.com/gamegine/HCSR04-ultrasonic-sensor-lib

Bitcraze_PMW3901 onboard_flow_sensor(PMW3901_CS);
HCSR04 ultrasonic_sensor(DIST_TRIG, DIST_ECHO);
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

  float dist_cm = ultrasonic_sensor.dist();

  int16_t dx = 0, dy = 0;
  unsigned long ct = millis();
  unsigned long dt = ct - lastUpdate;

  onboard_flow_sensor.readMotionCount(&dx, &dy);

  Serial.print(dx);
  Serial.print(",");
  Serial.print(dy);
  Serial.print(",");
  Serial.print(dist_cm);
  Serial.print(",");
  Serial.println(dt);
  Serial.flush();
  lastUpdate = ct;
  //delay(UPDATE_FREQUENCY_MS);
}