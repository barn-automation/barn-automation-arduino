 #include <dht.h>
#include <stddef.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#define DHT11_0_PIN 8
#define DOOR_0_SWITCH_PIN 2
#define DOOR_1_SWITCH_PIN 3
#define DOOR_1_LED_PIN 9
#define RELAY_0_PIN 7
#define WATER_0_PIN A0
#define PHOTORESISTOR_0_PIN A1
#define SERIESRESISTOR 560    
#define ZERO_VOLUME_RESISTANCE    1530.80    // Resistance value (in ohms) when no liquid is present.
#define CALIBRATION_RESISTANCE    621.00    // Resistance value (in ohms) when liquid is at max line.
#define CALIBRATION_VOLUME        700.00    // Volume (in any units) when liquid is at max line.

const int NO_TYPE = -1;
const int MOTOR_0 = 0;  
const int MOTOR_1 = 1;
const int DOOR_0 = 10;
const int DOOR_1 = 11;
const int RELAY_0 = 20;
const int WATER_0 = 30;
const int WATER_1 = 31;
const int DOOR_LED_0 = 50;
const int DOOR_LED_1 = 51;

const int OPEN = 100;
const int CLOSE = 101;
const int ON = 200;
const int OFF = 201;
const int EMPTY = 300;
const int FILL = 301;

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
dht DHT;
String serialRead;

struct IncomingMessage {
  int type;
  int message;
};

volatile IncomingMessage queuedMsg;

class RelayControl {
  public:
    int relayState;
    int relayPin;

  private:
    long onTime;
    long offTime;
    unsigned long previousMillis;

  public:
    RelayControl(int pin, long on, long off=0) {
      relayPin = pin;
      onTime = on;
      offTime = off;
      relayState = LOW;
      previousMillis = 0;  
    }

    void Update(unsigned long currentMillis) {
      if( relayState == HIGH && (currentMillis - previousMillis >= onTime) ) {
        relayState = LOW;
        digitalWrite(relayPin, LOW);
        previousMillis = currentMillis;
      }  
      else if( (relayState == LOW) && (currentMillis - previousMillis >= offTime) ) {
        relayState = HIGH;
        digitalWrite(relayPin, HIGH);
        previousMillis = currentMillis;
      }
    }
};

class MotorControl {
  public:
    int motorState;
    uint8_t motorDirection;
    
  private:
    int motorPin;      
    long onTime;
    long offTime;
    unsigned long previousMillis;
 
  public:
    MotorControl(int pin, long on, long off=0) {
      motorPin = pin;
      onTime = on;
      offTime = off;
      motorState = LOW; 
      previousMillis = 0;
    }
 
    void Update(unsigned long currentMillis, uint8_t dir=FORWARD) {
      motorDirection = dir;
      if((motorState == HIGH) && (currentMillis - previousMillis >= onTime)) {
        motorState = LOW;  
        previousMillis = currentMillis;
      }
      else if ((motorState == LOW) && (currentMillis - previousMillis >= offTime)) {
        motorState = HIGH;
        previousMillis = currentMillis;   
      }
    }
};

MotorControl motorControl0(1, 2200); //door_1
Adafruit_DCMotor *motor0 = AFMS.getMotor(1); //door_1
Adafruit_DCMotor *motor1 = AFMS.getMotor(2); //pump
uint8_t motor1State = LOW;
bool water0WasRefilled = false;
bool emptyWater0 = false;
bool water0Enabled = false;
RelayControl relayControl0(RELAY_0_PIN, 5000);

void setup() {
  Serial.begin(9600);
  pinMode(DOOR_0_SWITCH_PIN, INPUT_PULLUP);
  pinMode(DOOR_1_SWITCH_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RELAY_0_PIN, OUTPUT);
  pinMode(WATER_0_PIN, INPUT);
  pinMode(PHOTORESISTOR_0_PIN, INPUT);
  pinMode(DOOR_1_LED_PIN, OUTPUT);
  AFMS.begin();

  motor0->setSpeed(43);

  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

SIGNAL(TIMER0_COMPA_vect) {
  unsigned long currentMillis = millis();
  if( queuedMsg.type != NO_TYPE ) {
    switch(queuedMsg.type) {
      case DOOR_1:
        // start the motor
        motorControl0.Update(currentMillis, queuedMsg.message == OPEN ? FORWARD : BACKWARD);
        break;  
      case DOOR_0:
        relayControl0.Update(currentMillis);
        break;
      case DOOR_LED_1:
        digitalWrite( DOOR_1_LED_PIN, queuedMsg.message == ON ? HIGH : LOW );
        break;
      case WATER_0:
        if( queuedMsg.message == EMPTY ) {
          emptyWater0 = true;
        }
        if( queuedMsg.message == ON || queuedMsg.message == OFF ) {
          water0Enabled = queuedMsg.message == ON;
        }
        break;
      default:
        break;
    }
    IncomingMessage newMsg = { NO_TYPE, "" };
    queuedMsg = newMsg;
  }
  /* check if motors/doors should be stopped/closed */
  if( motorControl0.motorState == HIGH ) {
    motorControl0.Update(currentMillis, motorControl0.motorDirection);  
  }
  if( relayControl0.relayState == HIGH ) {
    relayControl0.Update(currentMillis);  
  }
} 

void loop() {
  unsigned long curMillis = millis();
  checkWater();

  /* handle incoming messages */
  IncomingMessage msg = readIncomingMessage();
  // message handling happens in the timer0 above, but we read it here because timer's can't read serial ^^^
  /* end incoming messages */

  if( emptyWater0 && water0Enabled ) {
    int water0Level = readWater( WATER_0_PIN );
    if( water0Level > 50 && motor1State == LOW ) {
      motor1->run(BACKWARD);
      motor1State = HIGH;
    } 
    if( water0Level < 50 && motor1State == HIGH ) {
      emptyWater0 = false;
      motor1->run(RELEASE); 
      motor1State = LOW;
    }
  }
  if( !water0Enabled ) {
    emptyWater0 = false;
    motor1->run(RELEASE); 
    motor1State = LOW;
  }
  
  if( motorControl0.motorState == HIGH ) {      
    motor0->run(motorControl0.motorDirection);
  }
  else if( motorControl0.motorState == LOW ) {
    motor0->run(RELEASE);  
  }
  /* setup output buffer */
  StaticJsonBuffer<400> outputBuffer;
  JsonObject& doc = outputBuffer.createObject();
  doc["status"] = "OK";
  JsonArray& messages = doc.createNestedArray("messages");

  dht temp = collectTemp(DHT11_0_PIN);

  // door status
  JsonObject& door0Obj = outputBuffer.createObject();
  door0Obj["type"] = "DOOR_0";
  JsonObject& door0Data = door0Obj.createNestedObject("data");
  door0Data["status"] = readDoor(DOOR_0_SWITCH_PIN) ? "OPEN" : "CLOSED";
  messages.add( door0Obj );
  
  JsonObject& door1Obj = outputBuffer.createObject();
  door1Obj["type"] = "DOOR_1";
  JsonObject& door1Data = door1Obj.createNestedObject("data");
  door1Data["status"] = readDoor(DOOR_1_SWITCH_PIN) ? "OPEN" : "CLOSED";
  door1Data["light"] = digitalRead(DOOR_1_LED_PIN) == HIGH ? "ON": "OFF";
  messages.add( door1Obj );
  // weather
  JsonObject& tempObj = outputBuffer.createObject();
  tempObj["type"] = "TEMP_0";
  JsonObject& tempData = tempObj.createNestedObject("data");
  tempData["celsius"] = temp.temperature;
  tempData["fahrenheit"] = cToF(temp.temperature);
  tempData["humidity"] = temp.humidity;
  messages.add( tempObj );
  
  // water
  JsonObject& water0Obj = outputBuffer.createObject();
  water0Obj["type"] = "WATER_0";
  JsonObject& water0Data = water0Obj.createNestedObject("data");
  water0Data["level"] = readWater( WATER_0_PIN );
  water0Data["enabled"] = water0Enabled;
  if( water0WasRefilled ) {
      water0Data["refilled"] = true;
  }
  messages.add( water0Obj );

  // light
  JsonObject& light0Obj = outputBuffer.createObject();
  light0Obj["type"] = "LIGHT_0";
  JsonObject& light0Data = light0Obj.createNestedObject("data");
  light0Data["level"] = analogRead( PHOTORESISTOR_0_PIN );
  messages.add( light0Obj );

  doc.printTo(Serial);
  Serial.print('\n');
  /* end output buffer */
   
  delay(1000); // slight delay, to give the Pi side time to consume the contents before moving on
}

IncomingMessage readIncomingMessage() {
  IncomingMessage msg = { NO_TYPE, "" };
  
  if (Serial.available() > 0) {
    serialRead = Serial.readString();
  }
  //todo: handle it
  if( serialRead.length() > 0 ) {
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(serialRead);
    int iType = root["type"].as<int>();
    int iMsg = root["message"].as<int>();
    msg = { iType, iMsg };
    serialRead = "";  
  }
  queuedMsg = msg;
  return msg;
}

float readWater(int pin) {
  float water0Level = analogRead( pin );
  float reading = (1023.0 / water0Level)  - 1.0;
  reading = SERIESRESISTOR / reading;
  float waterMl = resistanceToVolume(reading, ZERO_VOLUME_RESISTANCE, CALIBRATION_RESISTANCE, CALIBRATION_VOLUME);
  return waterMl;
}

void checkWater() {
  motor1->setSpeed(255);
  water0WasRefilled = false;
  if( !emptyWater0 && water0Enabled ) {
    float water0Level = readWater( WATER_0_PIN );

    if( water0Level <= 100 && motor1State == LOW ) {
      motor1->run(FORWARD);
      motor1State = HIGH;
    } 
    if( water0Level >= 100 && motor1State == HIGH ) {
      motor1->run(RELEASE); 
      motor1State = LOW;
      water0WasRefilled = true;
    }
  }
}

float resistanceToVolume(float resistance, float zeroResistance, float calResistance, float calVolume) {
  if (resistance > zeroResistance || (zeroResistance - calResistance) == 0.0) {
    // Stop if the value is above the zero threshold, or no max resistance is set (would be divide by zero).
    return 0.0;
  }
  // Compute scale factor by mapping resistance to 0...1.0+ range relative to maxResistance value.
  float scale = (zeroResistance - resistance) / (zeroResistance - calResistance);
  // Scale maxVolume based on computed scale factor.
  return calVolume * scale;
}

boolean readDoor(int pin) {
  return digitalRead(pin) == HIGH;
}

dht collectTemp(int pin) {
  int chk = DHT.read11(pin);
  return DHT;
}

/* utility functions */
int cToF(int c) {
  return c * 9 / 5 + 32;
}
