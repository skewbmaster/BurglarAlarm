#include "definitions.hh"
#include "Volume.h"

#define UPDATE_MS 25
#define DEFAULT_BUZZER_VOLUME 200

Volume* volumeController;
Buzzer* buzzer;

ControlPanel* alarm;

unsigned long timestamp;
void setup() {
  Serial.begin(9600);
  //Serial.println("System boot");

  volumeController = new Volume();
  buzzer = new Buzzer(BUZZER_PIN, DEFAULT_BUZZER_VOLUME, volumeController);

  alarm = new ControlPanel(buzzer);

  timestamp = millis();
}

void loop() {
  if (millis() - timestamp < UPDATE_MS) {
    return;
  }

  //alarm->Update();

  timestamp = millis();
}
