#ifndef ALARM_DEFINITIONS_H
#define ALARM_DEFINITIONS_H

#include "Volume.h"
#include <stdio.h>

#define LED_COUNT 3
#define BUZZER_FREQUENCY 400
#define CONFIRM_ENTRY_TIMEOUT_MS 45000

#define ACKNOWLEDGE_MSG "ACK"
#define ENTRY_MSG "ENTRY_CONFIRM"
#define DOOR_MSG "DOOR_CHANGE"

#define LED_DOOR_PIN 7
#define LED_WINDOW_PIN 6
#define LED_ARMED_PIN 5
#define BUZZER_PIN 4
#define MOTION_SENSOR_PIN 9
#define DOOR_RFID_BUTTON_PIN 18
#define WINDOW_SENSOR_PIN 8
#define SOLENOID_PIN 11


class ArduinoInput {
  protected:
    int arduinoPin;
    virtual bool GetValue();
  public:
    ArduinoInput(int pinNumber);
};

class HoldSensor : public ArduinoInput {
  protected:
    bool detected;
  public:
    HoldSensor(int pinNumber);
    bool GetState() { return detected; };
    void Update();
    void Reset();
};

class TriggerSensor : public ArduinoInput {
  private:
    bool triggered;
    byte lastState;
  public:
    TriggerSensor(int sensorPin);
    void Update();
    bool IsTriggered();
    void Reset();
};

class ArduinoOutput {
  protected:
    int arduinoPin;
  public:
    ArduinoOutput(int pinNumber);
    void SetValue(bool value);
};

class LED : public ArduinoOutput {
  private:
    byte state;
  public:
    LED(int ledPin) : ArduinoOutput(ledPin) { };
    void SetState(byte newState);
    void FlipState();
};

class Solenoid : public ArduinoOutput {
  private:
    bool state;
  public:
    Solenoid(int solenoidPin);
    void ChangeLockState();
};

class Buzzer : public ArduinoOutput {
  private:
    byte volumeLevel; 
    Volume* volumeController;
  public:
    Buzzer(int buzzerPin, byte initialVolume, Volume* volumeController);
    void Play();
    void Stop();
    void ChangeVolume(byte newVolume);
};

class SerialCommunicationDevice {
  protected:
    bool success;
  public:
    SerialCommunicationDevice();
    bool SendMessage(char* message);
    String ReceiveMessage();
};

class ControlPanel {
  private:
    typedef enum {
      Success,
      Failure,
      Waiting
    } EntryOutcome;
    typedef enum {
      DoorLED,
      WindowLED,
      ArmedLED
    } LEDNames;

    Buzzer* buzzer;
    HoldSensor* windowSensor;
    TriggerSensor* doorSolenoidButton;
    TriggerSensor* motionSensor;
    Solenoid* solenoidLock;
    SerialCommunicationDevice* communication;
    LED* LEDs[LED_COUNT];

    bool alarmActive, alarmArmed;
    bool confirmingIdentity, reArming;
    unsigned long timerStart;

    void DisarmedUpdate();
    void SoundAlarm();
    void StopAlarm();
    void UnlockHandler();
    ControlPanel::EntryOutcome CommunicateEntry();

  public:
    ControlPanel(Buzzer* buzzerObject);
    void Update();
};



#endif
