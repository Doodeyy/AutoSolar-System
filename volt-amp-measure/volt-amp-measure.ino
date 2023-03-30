#define voltageSenorPinBatt A0
#define currentSensorPinBatt A1
#define voltageSenorPinSolar A2
#define DIGITAL_IN_PIN 11
#define DIGITAL_IN_PIN2 13
#define BUTTON_PIN 12

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <ArduinoMqttClient.h>
#include <SPI.h>
#include <WiFiNINA.h>

char ssid[] = "ASSPAP";
char pass[] = "1234567890";   // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char broker[] = "192.168.2.2";
int        port     = 1883;
const char topic[]  = "BattVolt";
const char topic1[]  = "BattAmp";
const char topic2[]  = "SolarVolt";
const char topic3[] = "EnableButton";
bool enable;
bool autoS;

//set interval for sending messages (milliseconds)
const long interval = 2000;
unsigned long previousMillis = 0;

int count = 0;


// Set the LCD address to 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A by Ti
LiquidCrystal_I2C lcd(0x27, 16, 3);

byte battery[8] =  //icon for battery
{
  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};

byte energy[8] =  // icon for power
{
  0b00010,
  0b00100,
  0b01000,
  0b11111,
  0b00010,
  0b00100,
  0b01000,
  0b00000
};

byte charge[8] = // icon for battery charge
{
  0b01010,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b01110,
  0b00100,
  0b00100,
};

byte not_charge[8]=
{
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
  0b00000,
};


void setup(){
  // Setup Serial Monitor
  Serial.begin(9600);
  
  lcd.begin();
  lcd.createChar(0, battery);
  lcd.createChar(1, charge);
  lcd.createChar(2, not_charge);
  lcd.createChar(3, energy);
  lcd.home();

  // attempt to connect to Wifi network:
  lcd.setCursor(0, 0);
  lcd.print("Connecting: ");
  lcd.setCursor(13, 0);
  lcd.print(ssid);

  Serial.print("Attempting to connect to WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // failed, retry
    lcd.setCursor(0, 1);
    lcd.print("Connection Failed!");
    Serial.print(".");
    delay(5000);
  }
  lcd.setCursor(0, 1);
  lcd.print("                  ");
  
  Serial.println("You're connected to the network");
  Serial.println();

  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("You're connected to the MQTT broker!");
  Serial.println();

  pinMode(DIGITAL_IN_PIN, OUTPUT);

  lcd.setCursor(13, 0);
  lcd.print("      ");
  mqttClient.onMessage(onMqttMessage);
  
}

float votlMeasure(int Pin) {
  float R1 = 30000.0;
  float R2 = 7500.0;
  float adc_voltage = 0.0;
  float ref_voltage = 5.0;
  int adc_value = 0;

  adc_value = analogRead(Pin);  
  adc_voltage  = (adc_value * ref_voltage) / 1024.0; 

  return adc_voltage / (R2/(R1+R2)); 
}

float ampMeasure(int Pin) {
  unsigned int x=0;
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++) {
    AcsValue = analogRead(Pin);
    Samples = Samples + AcsValue;
    delay (3); 
  }

  AvgAcs=Samples/150.0;
  return (((2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.185));
}

void loop(){
  float battVoltSens = votlMeasure(voltageSenorPinBatt);
  float battAmpConsumption = (ampMeasure(currentSensorPinBatt)) < 0 ? 0.0 : ampMeasure(currentSensorPinBatt);
  float solarVoltSens = votlMeasure(voltageSenorPinSolar);
  
    // call poll() regularly to allow the library to send MQTT keep alive which
  // avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;


    // Serial.println("Battery voltage: " + String(battVoltSens));
    // Serial.println("Battery Amperage: " + String(battAmpConsumption));  
    // Serial.println("Solar voltage: " + String(solarVoltSens));
    // Serial.println("-------------------------------");

    mqttClient.beginMessage(topic);
    mqttClient.print(battVoltSens);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic1);
    mqttClient.print(battAmpConsumption);
    mqttClient.endMessage();

    mqttClient.beginMessage(topic2);
    mqttClient.print(solarVoltSens);
    mqttClient.endMessage();

    mqttClient.subscribe(topic3);

    Serial.println();
  } 


  delay(100);

  if (solarVoltSens > 11.0) {
    lcd.setCursor(0, 0);
    lcd.print("Charge State: ");
    lcd.setCursor(15, 0);
    lcd.write(1);
    digitalWrite(DIGITAL_IN_PIN , HIGH);

  } else {
    lcd.setCursor(0, 0);
    lcd.print("Charge State: ");
    lcd.setCursor(15, 0);
    lcd.write(2);
    digitalWrite(DIGITAL_IN_PIN , LOW);

  }
  if (battVoltSens < 0){
    battVoltSens = 0;
  }

  lcd.setCursor(0, 1);
  lcd.print("Power Volt: ");
  lcd.setCursor(15, 1);
  lcd.print(battVoltSens);



  lcd.setCursor(0, 2);
  lcd.print("Power Amp: ");
  lcd.setCursor(15, 2);
  lcd.print(battAmpConsumption);

  if (solarVoltSens < 0){
    solarVoltSens = 0.0;
  }

  lcd.setCursor(0, 3);
  lcd.print("Solar Volt: ");
  lcd.setCursor(15, 3);
  lcd.print(solarVoltSens);
  // if(autoS == false) {
  //   if(enable) {
  //     digitalWrite(DIGITAL_IN_PIN , HIGH);
  //   } else {
  //     digitalWrite(DIGITAL_IN_PIN , HIGH);
  //   }
  // }
}

void onMqttMessage(int messageSize) {
  // we received a message, print out the topic and contents
  Serial.println("Received a message with topic '");
  Serial.print(mqttClient.messageTopic());
  Serial.print("', length ");
  Serial.print(messageSize);
  Serial.println(" bytes:");
  // use the Stream interface to print the contents
  while (mqttClient.available()) {
    Serial.println((char)mqttClient.read());
    Serial.println((char)mqttClient.read() == "f");
    if((char)mqttClient.read() == "n") {
      autoS = false;
      enable = true;
    } else if ((char)mqttClient.read() == "f") {
      enable = false;
      autoS = false;

    } else if ((char)mqttClient.read() == "a") {
      autoS = true;
    }
  }
  Serial.println(autoS);
  Serial.println(enable);
}
