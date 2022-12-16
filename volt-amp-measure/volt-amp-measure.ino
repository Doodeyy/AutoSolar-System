#define ANALOG_IN_PIN A0
#define ANALOG_IN_PIN_SOLAR A2

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

float adc_voltage = 0.0;
float in_voltage = 0.0;
float adc_voltage_s = 0.0;
float in_voltage_s = 0.0;

// Floats for resistor values in divider (in ohms)
float R1 = 30000.0;
float R2 = 7500.0; 

// Float for Reference Voltage
float ref_voltage = 5.0;
float ref_voltage_s = 5.0;

// Integer for ADC value
int adc_value = 0;
int adc_value_s = 0;

void setup(){
  // Setup Serial Monitor
  Serial.begin(9600);
  pinMode(9, OUTPUT);

  lcd.begin();
  lcd.createChar(0, battery);
  lcd.createChar(1, charge);
  lcd.createChar(2, not_charge);
  lcd.createChar(3, energy);
  lcd.home();
  
}

void loop(){
      digitalWrite(9, HIGH);

    // Read the Analog Input
    adc_value = analogRead(ANALOG_IN_PIN);
    
    // Determine voltage at ADC input
    adc_voltage  = (adc_value * ref_voltage) / 1024.0; 
    
    // Calculate voltage at divider input
    in_voltage = adc_voltage / (R2/(R1+R2)); 
    
    // Print results to Serial Monitor to 2 decimal places
    Serial.println("Battery voltage: " + String(in_voltage));


    
    
    //amp measure

    unsigned int x=0;
    float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;

    for (int x = 0; x < 150; x++){ //Get 150 samples
      AcsValue = analogRead(A1);     //Read current sensor values   
      Samples = Samples + AcsValue;  //Add samples together
      delay (3); // let ADC settle before next sample 3ms
      }
    AvgAcs=Samples/150.0;//Taking Average of Samples

    //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
    //2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
    //out to be 2.5 which is out offset. If your arduino is working on different voltage than 
    //you must change the offset according to the input voltage)
    //0.185v(185mV) is rise in output voltage when 1A current flows at input
    AcsValueF = ((2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.185);

    Serial.println("Battery Amperage: " + String(AcsValueF));//Print the read current on Serial monitor
    delay(3);

    // voltage measure solar panel

    // Read the Analog Input
    adc_value_s = analogRead(ANALOG_IN_PIN_SOLAR);
    
    // Determine voltage at ADC input
    adc_voltage_s  = (adc_value_s * ref_voltage_s) / 1024.0; 
    
    // Calculate voltage at divider input
    in_voltage_s = adc_voltage_s / (R2/(R1+R2)); 
    
    // Print results to Serial Monitor to 2 decimal places
    Serial.println("Solar voltage: " + String(in_voltage_s));
    Serial.println("-------------------------------");
    delay(100);

    if (in_voltage_s > 11.0) {
      lcd.setCursor(0, 0);
      lcd.print("Charge State: ");
      lcd.setCursor(15, 0);
      lcd.write(1);

    }
    else {
      lcd.setCursor(0, 0);
      lcd.print("Charge State: ");
      lcd.setCursor(15, 0);
      lcd.write(2);

    }

    lcd.setCursor(0, 1);
    lcd.print("Power Volt: ");
    lcd.setCursor(15, 1);
    lcd.print(in_voltage);

    if (AcsValueF < 0){
      AcsValueF = 0.0;
    } 

    lcd.setCursor(0, 2);
    lcd.print("Power Amp: ");
    lcd.setCursor(15, 2);
    lcd.print(AcsValueF);

    lcd.setCursor(0, 3);
    lcd.print("Solar Volt: ");
    lcd.setCursor(15, 3);
    lcd.print(in_voltage_s);
    




    
    
    
}