#include <Adafruit_VL53L0X.h>
#include <Bitcraze_PMW3901.h>
#define PMW3901_CS 10
#define UPDATE_FREQUENCY_MS 10

Adafruit_VL53L0X onboard_dist_sensor = Adafruit_VL53L0X();
uint16_t last_dist = 0xffff;
Bitcraze_PMW3901 onboard_flow_sensor(PMW3901_CS);

void setup(){
  //Open up serial port and enable communication
  Serial.begin(115200);

  while(!Serial){
    delay(1);
  }

  if(!onboard_dist_sensor.begin()){
    while(1){
      Serial.println("{\"error\":\"Failed to detect and initialize distance sensor!\"}");
      delay(1000);
    }
  }
  if (!onboard_flow_sensor.begin()) {
    while(1){
      Serial.println("\"error\":\"Failed to detect and initialize flow sensor!\"");
      delay(1000);
    }
  }
  onboard_dist_sensor.startRangeContinuous();
}

void loop() {
  if(onboard_dist_sensor.isRangeComplete()){
    uint16_t dist = onboard_dist_sensor.readRange();
    if(dist != 0xffff){
      last_dist = dist;
    }
  }
  if(last_dist == 0xffff){
    return;
  }
  int16_t dx = 0, dy = 0;
  onboard_flow_sensor.readMotionCount(&dx, &dy);
  Serial.println("{\"dist\":" + String(last_dist) + ",\"dx\":" + String(dx) + ",\"dy\":" + String(dy) + "}");
}