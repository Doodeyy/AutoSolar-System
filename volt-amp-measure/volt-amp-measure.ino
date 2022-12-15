#define ANALOG_IN_PIN A0
#define ANALOG_IN_PIN_SOLAR A2

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
    AcsValueF = ((2.5 - (AvgAcs * (5.0 / 1024.0)) )/0.185) - 0.079;

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
}