#include "definitions.hh"
#include "volume-library/Volume.h"

#define UPDATE_MS 25

unsigned long timestamp;
void setup() {
  // put your setup code here, to run once:
  //HoldSensor hello = HoldSensor(3);
  Serial.begin(9600);
  Serial.println("System boot");
  timestamp = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - timestamp < UPDATE_MS) {
    return;
  }

  
}
