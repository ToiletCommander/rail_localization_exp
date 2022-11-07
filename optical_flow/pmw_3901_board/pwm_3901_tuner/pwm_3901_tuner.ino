#include <Bitcraze_PMW3901.h>
#include <HCSR04.h>
#define PMW3901_CS 10
#define DIST_TRIG 6
#define DIST_ECHO 7
#define UPDATE_SERIAL_WAIT 100

Bitcraze_PMW3901 onboard_flow_sensor(PMW3901_CS);
HCSR04 ultrasonic_sensor(DIST_TRIG, DIST_ECHO);

unsigned long lastUpdate = 0UL;
long total_dx = 0L;
long total_dy = 0L;

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
  int16_t dx = 0, dy = 0;

  if(Serial.available() > 0){
    while(Serial.available() > 0){
      Serial.read();
      onboard_flow_sensor.readMotionCount(&dx, &dy);
      delay(UPDATE_SERIAL_WAIT);
    }
    total_dx = 0L;
    total_dy = 0L;
  }

  float dist_cm = ultrasonic_sensor.dist();
  
  unsigned long ct = millis();
  unsigned long dt = ct - lastUpdate;

  onboard_flow_sensor.readMotionCount(&dx, &dy);
  total_dx += dx;
  total_dy += dy;

  Serial.print(dist_cm);
  Serial.print(",");
  Serial.print(dx);
  Serial.print(",");
  Serial.print(dy);
  Serial.print(",");
  Serial.print(dt);
  Serial.print(",");
  Serial.print(total_dx);
  Serial.print(",");
  Serial.println(total_dy);
  Serial.flush();
  lastUpdate = ct;
  delay(50);
}