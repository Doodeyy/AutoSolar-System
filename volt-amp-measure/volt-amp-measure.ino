#define voltageSenorPinBatt A0
#define currentSensorPinBatt A1
#define voltageSenorPinSolar A2

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 in PCF8574 by NXP and Set to 0x3F in PCF8574A by Ti
LiquidCrystal_I2C lcd(0x27, 16, 3);

byte customChar[] = {
  B11111,
  B00000,
  B11111,
  B00100,
  B00100,
  B11111,
  B00000,
  B11111
};

byte battery[8] =  {//icon for battery

  0b01110,
  0b11011,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b10001,
  0b11111
};

byte energy[8] = { // icon for power

  0b00010,
  0b00100,
  0b01000,
  0b11111,
  0b00010,
  0b00100,
  0b01000,
  0b00000
};

byte charge[8] = { // icon for battery charge

  0b01010,
  0b11111,
  0b10001,
  0b10001,
  0b10001,
  0b01110,
  0b00100,
  0b00100,
};

byte not_charge[8] = {
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
  Serial.begin(9600);
  lcd.begin();
  lcd.createChar(0, battery);
  lcd.createChar(1, charge);
  lcd.createChar(2, not_charge);
  lcd.createChar(3, energy);
  lcd.home();  
}

int votlMeasure(int Pin) {
  float R1 = 30000.0;
  float R2 = 7500.0;
  float adc_voltage = 0.0;
  float ref_voltage = 5.0;
  int adc_value = 0;

  adc_value = analogRead(Pin);  
  adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
  return adc_voltage / (R2/(R1+R2)); 
}

int ampMeasure(int Pin) {
  unsigned int x=0;
  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

  for (int x = 0; x < 150; x++) {
    AcsValue = analogRead(Pin);
    Samples = Samples + AcsValue;
    delay (3); 
  }

  AvgAcs=Samples/150.0;
  return ((2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.185);
}

void lcdPrint(int textCordsX, int textCordsY, char printMsg, int iconCordsX, int iconCordsY, int iconNumber, char textPrint) {
  lcd.setCursor(textCordsX, textCordsY);
  lcd.print(printMsg);
  lcd.setCursor(iconCordsX, iconCordsY);
  iconNumber == -1 ? lcd.write(iconNumber) : lcd.print(textPrint);
}

void loop() {
  float battVoltSens = votlMeasure(voltageSenorPinBatt);
  float battAmpConsumption = (ampMeasure(currentSensorPinBatt)) < 0 ? 0.0 : ampMeasure(currentSensorPinBatt);
  float solarVoltSens = votlMeasure(voltageSenorPinSolar);
  
  Serial.println("Battery voltage: " + String(battVoltSens));
  Serial.println("Battery Amperage: " + String(battAmpConsumption));  
  Serial.println("Solar voltage: " + String(solarVoltSens));
  Serial.println("-------------------------------");

  solarVoltSens > 11.0 ? lcdPrint(0, 0, "Charge State: ", 15, 0, 1, " ") : lcdPrint(0, 0, "Charge State: ", 15, 0, 2, " ");

  lcdPrint(0, 1, "Power Volt:  ", 15, 1, -1, solarVoltSens);
  lcdPrint(0, 3, "Power Amp: ", 15, 2, -1, battAmpConsumption);
  lcdPrint(0, 3, "Solar Volt: ", 15, 3, 1, " ");

  delay(100);
}