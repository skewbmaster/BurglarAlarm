#include "definitions.hh"
#include "Volume.h"

#define UPDATE_MS 20
#define DEFAULT_BUZZER_VOLUME 200

Volume* volumeController;
Buzzer* buzzer;

ControlPanel* alarm;

unsigned long timestamp;
void setup() {
  Serial.begin(9600);
  Serial.setTimeout(500);
  //Serial.println("System boot");

  // Volume object needs to be created in the ino file because arduino isn't good
  volumeController = new Volume();
  buzzer = new Buzzer(BUZZER_PIN, DEFAULT_BUZZER_VOLUME, volumeController);
  // Pass buzzer into the alarm constructor
  alarm = new ControlPanel(buzzer);

  timestamp = millis();
}

void loop() {
  if (millis() - timestamp < UPDATE_MS) {
    return;
  }
  timestamp = millis();

  // Continuously update the main class
  alarm->Update();
}
