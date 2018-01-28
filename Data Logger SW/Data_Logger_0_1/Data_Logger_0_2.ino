

/*
Data Logger Code Version Proto 0.20
Designed to run on an Uno R3 type Arduino
Schematic Rev 2.0


Rev History
0.20 Proto 2 - upated SD Card error message (added Op State 4) 
               changed numeral display on LCD to XX.XX format
               changed current measurement to Unidirectional
               added writeProtectLED to D9
0.10 Proto 1 - start of code development

I/O DEFINITIONS
A0 - System battery voltage
A1 - Current Measurement of Hall Effect Transducer
A2 - Temperature Thermistor 1 - Optional
A3 - Temperature Thermistor 2 - Optional
A4 - I2C SDA
A5 - I2C SCL
D0 - Rx
D1 - Tx
D2 - (INT1 pin) Not used
D3 - (INT2 pin) Not used
D4 - Current Sensor Status Blue LED 
D5 - Current Sensor Status Yellow LED 
D6 - SD Card present LED (L1) on Data Logger Shield
D7 - SD Card write protect on Data Logger Shield
D8 - SD Card detect on Data Logger Shield
D9 - SD Write Protect LED (L2) on Data Logger Shield
D10 - Chip Select for SD card
D11 - Not used
D12 - Not used
D13 - Arduino D13 LED

I2C Bus Addresses
0x00 - LCD Display
0x68 - RTC

*/

/*-------------------------------------------------------------
*                        INCLUDES
*-------------------------------------------------------------*/

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Adafruit_LiquidCrystal.h>
Adafruit_LiquidCrystal lcd(0); //Default address #0 A0-A2 not jumpered

/*----------------------------------------------------------
 *                      Define Variables 
 *----------------------------------------------------------- */                  

 const int currentFlowLED = 4;     //blue LED
 const int currentZeroLED= 5;      //yellow LED
 const int chipSelect= 10;         //SD card chip select
 const int cardDetect= 8;          //SD card detect on data log shield
 const int cardPresentLED = 6;     //SD card present L1 on data log shield
 const int writeProtect = 7;       //SD card write protect on data log shield
 const int writeProtectLED = 9;    //SD card write protect L2 on data log shield
 const int battVolt = A0;          //system battery voltage
 const int currentTransducer= A1;  //load current
 const int temp1 = A2;             //temperature 1
 const int temp2 = A3;             //temperature 2
 const int thermistorNominal=10000;
 const int temperatureNominal=25; 
 const int bCoefficient=3950;
 const int seriesResistor=10000;   //nominal value, measure for better accuracy
 unsigned int timeCounter = 0;     //counter incremented by TIMER INT ISR
 unsigned int x = 0;               //used with timeCounter to determine 5 min
 float sensorValue = 0.0;          //should this be float or int?
 float sysVoltage = 0.0;           //battery voltage 
 float loadAmps = 0.0;             //load current
 float loadPower = 0.0;            //loadAmps x sysVoltage
 float temperature1 = 0.0;         //temperature 1
 float temperature2 = 0.0;         //temperature 2
 float steinhart = 0.0;
 unsigned long startTime = 0;      //used in delay timers
 unsigned long delay1000 = 1000;   //delay 1000ms
 unsigned long delay500 = 500;     //delay 500ms
 unsigned long delay200 = 200;     //delay 200ms 
 volatile byte operatingState = 2; //initialize op state to "not logging"
 //boolean cardPresent = HIGH;     //initialize to "no card installed"


//String Variables
 String firmwareVer="0.20";

//End of Variables Definition ------------------------- 


/*----------------------------------------------
 * FUNCTION DEFINITIONS
 -----------------------------------------------*/

//measure battery voltage
float measureVoltage() {
sensorValue=0.0;
for (int i=0; i<5; i++) {
     sensorValue = sensorValue + analogRead(A0);
     delay(10);
   }
sensorValue=sensorValue/5.0;
sysVoltage = ((sensorValue *0.00488*7.0)+0.7);//voltage scaled to 35V full scale
return sysVoltage;
} //end of measureVoltage()


//measure load current
float measureCurrent() {
sensorValue=0.0;
for (int i=0; i<5; i++) {
     sensorValue = sensorValue + analogRead(A1);
     delay(10);
   }
sensorValue=sensorValue/5.0;                     //average of 5 readings
loadAmps = (sensorValue * 0.00488);              //scaled to 5V full scale 
loadAmps = (loadAmps * 10);                      //scaled to 50A full scale
return loadAmps;
} //end of measureCurrent()

//calulate load power
float calcLoadPwr() {
loadPower  = sysVoltage * loadAmps;
return loadPower;
} //end of calcLoadPwr function


//measure temperature 1
float measureTemp1() {
sensorValue=0.0;
for (int i=0; i<5; i++) {
     sensorValue = sensorValue + analogRead(A2);
     delay(10);
   }
sensorValue=sensorValue/5;                       //Average of 5 readings
sensorValue = 1023 / sensorValue - 1;
  sensorValue = seriesResistor / sensorValue;
  steinhart = sensorValue / thermistorNominal;   // (R/Ro)
  steinhart = log(steinhart);                    // ln(R/Ro)
  steinhart /= bCoefficient;                     // 1/B * ln(R/Ro)
  steinhart += 1.0 / (temperatureNominal + 273.15);// + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15;  // convert to C
  steinhart = steinhart * 9.0 / 5.0 +32.0;       // convert to F
  temperature1 = steinhart;
  return temperature1;
} //end of measureTemp1()


//measure temperature 2
float measureTemp2() {
sensorValue=0.0;
for (int i=0; i<5; i++) {
     sensorValue = sensorValue + analogRead(A3);
     delay(10);
   }
sensorValue=sensorValue/5;                       //Average of 5 readings
sensorValue = 1023 / sensorValue - 1;
  sensorValue = seriesResistor / sensorValue;
  steinhart = sensorValue / thermistorNominal;   // (R/Ro)
  steinhart = log(steinhart);                    // ln(R/Ro)
  steinhart /= bCoefficient;                     // 1/B * ln(R/Ro)
  steinhart += 1.0 / (temperatureNominal + 273.15);// + (1/To)
  steinhart = 1.0 / steinhart;                   // Invert
  steinhart -= 273.15;                           // convert to C
  steinhart = steinhart * 9.0 / 5.0 +32.0;       // convert to F
  temperature2 = steinhart;
  return temperature2;
} //end of measureTemp2()

//check operating state function
byte checkOperatingState() {
//0 = No SD Card Present
//1 = Battery BackUp
//2 = Power Good Not Logging Data Low Current
//3 = Power Good Logging Data Normal Current
//4 = SD Card Write Protect Enabled

if (digitalRead(cardDetect)== LOW) {
    digitalWrite(cardPresentLED , HIGH); //turn on card LED if installed
}else { digitalWrite(cardPresentLED , LOW);
}

measureVoltage();
measureCurrent();

if (digitalRead(cardDetect) == HIGH) {
  operatingState = 0;
  digitalWrite(cardPresentLED , LOW); //turn off card Present LED
} else {
  if ((digitalRead(cardDetect)==LOW)&&(digitalRead(writeProtect)==HIGH)) {
  operatingState = 4;
} else {
  if (sysVoltage < 10.0) {operatingState = 1;
     } else {
  if ((sysVoltage >= 10.0) && (loadAmps >= 0.5)) {operatingState = 3;
     } else {
  if ((sysVoltage >= 10.0) && (loadAmps < 0.5)){operatingState = 2;
   }
}
}
}
}
return operatingState;
} //closing brace for check operating mode function




/*----------------------------------------------
 * ISR ROUTINES
 -----------------------------------------------*/


//Timer Interrupt ISR
ISR(TIMER1_COMPA_vect)                //timer compare interrupt service routine
{
 checkOperatingState();               //
 timeCounter += 1;                    //increment countHour on each interrupt
 x = (timeCounter%300);               //test for 5 min interval
 if (x == 0){
  timeCounter = 0;                    //if 5 minutes, log data. ADD LOGGING 
 }
} //end of timer isr


/* ----------------------------------------------
 *SETUP LOOP
-------------------------------------------------*/                 

void setup() {
 
   analogReference(DEFAULT);
   Serial.begin(9600);                 
   lcd.begin(16,2);                   //set up LCD number of rows and columns
   pinMode(currentFlowLED,OUTPUT);    //blue current sensor OK LED
   digitalWrite(currentFlowLED,LOW);
   pinMode(currentZeroLED,OUTPUT);    //yellow current sensor fault LED
   digitalWrite(currentZeroLED,LOW);
   pinMode(chipSelect,OUTPUT);        //SD card chip select
   digitalWrite(chipSelect,LOW);
   pinMode(cardDetect,INPUT_PULLUP);  //SD card detect on data log shield
   pinMode(writeProtect,INPUT_PULLUP);//SD write protect on data log shield
   pinMode(cardPresentLED,OUTPUT);    //SD card present L1 on data log shield
   digitalWrite(cardPresentLED,LOW);
   pinMode(writeProtectLED,OUTPUT);   //SD write protect L2 on data log shield
   digitalWrite(writeProtectLED,LOW);

  // set up Timer 1 interrupt
  noInterrupts();                     // disable all interrupts
                                      //set timer1 interrupt at 1Hz
  TCCR1A = 0;                         // set entire TCCR1A register to 0
  TCCR1B = 0;                         // same for TCCR1B
  TCNT1  = 0;                         //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;                      // = (16mHz)/(1*1024)-1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  interrupts();                       // enable all interrupts
  
   //post flash

for (int i=0; i <= 9; i++){
      digitalWrite(currentFlowLED, HIGH);
      startTime = millis();           //set startTime for delay loop
      while ((startTime + delay500) > millis() ) {
      } //500 mS delay
      digitalWrite(currentFlowLED, LOW);
      startTime = millis();           //set startTime for delay loop
      while ((startTime + delay500) > millis() ) {
      } //500 mS delay
   } //end of POST Flash for loop
  


//set up lcd and and send welcome message 

    lcd.clear();
    startTime = millis();             //set startTime for delay loop
    while ((startTime + delay500) > millis() ) {
    } //500 mS delay
    lcd.setCursor(0,0);               //Print opening message to the LCD
    lcd.print("Autonomous Power");
    lcd.setCursor(1,1);
    lcd.print("Energy Monitor");
    startTime = millis();             //set startTime for delay loop
    while ((startTime + 2000) > millis() ) {
    } //2000 mS delay
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("F/W Version ");
    lcd.setCursor(12,0);
    lcd.print(firmwareVer);
    startTime = millis();             //set startTime for delay loop
    while ((startTime + 2000) > millis() ) {
    } //2 Sec delay
    lcd.clear();
  

} //closing brace for setup loop

/*---------------------------------------------
 *MAIN LOOP
----------------------------------------------*/
void loop() {
//Serial Print for debug
//Serial.print("Int Counter = ");     //for debug
//Serial.println(timeCounter);        //for debug
Serial.print("Operating state = ");   //for debug
Serial.println(operatingState);       //for debug
Serial.print("Battery Voltage = ");   //for debug
Serial.println(sysVoltage);           //for debug
Serial.print("Load Current = ");      //for debug
Serial.println(loadAmps);             //for debug
loadPower = sysVoltage * loadAmps;
Serial.print("Load Power = ");        //for debug
Serial.println(loadPower);            //for debug
Serial.println("");
startTime = millis();                 //set startTime for delay loop
 while ((startTime + delay500) > millis() ) {
 } //500 mS delay


//0 = No SD Card Present
//1 = Battery BackUp
//2 = Power Good Not Logging Data Low Current
//3 = Power Good Logging Data Normal Current
//4 = SD Card Write Protect Enabled

if (operatingState == 0) {
   digitalWrite(cardPresentLED, LOW);
   digitalWrite(currentFlowLED, LOW);
   digitalWrite(currentZeroLED, LOW);
   digitalWrite(writeProtectLED, LOW);
   lcd.clear(); 
   lcd.setCursor(2,0); //LCD line 1, third position
   lcd.print("SD CARD ERROR");
   lcd.setCursor(0,1); //LCD line 2, first position
   lcd.print("CARD NOT PRESENT"); 
}
while (operatingState == 0) {
} //wait here for operatingState to change

if (operatingState == 4) {
   digitalWrite(cardPresentLED, HIGH);
   digitalWrite(currentFlowLED, LOW);
   digitalWrite(currentZeroLED, LOW);
   digitalWrite(writeProtectLED, HIGH);
   lcd.clear(); 
   lcd.setCursor(2,0); //LCD line 1, third position
   lcd.print("SD CARD ERROR");
   lcd.setCursor(2,1); //LCD line 2, first position
   lcd.print("WRITE PROTECT"); 
}

while (operatingState == 4) {
} //wait here for operatingState to change

//use do while ?

if (operatingState == 1) {
   digitalWrite(currentFlowLED, LOW);
   digitalWrite(currentZeroLED, LOW);
   digitalWrite(cardPresentLED, HIGH);
   digitalWrite(writeProtectLED, LOW);
   lcd.clear(); 
   lcd.setCursor(5,0);
   lcd.print("ERROR");
   lcd.setCursor(1,1);
   lcd.print("BATTERY BACKUP");  
}
while (operatingState == 1) {
} //wait here for operatingState to change
 

if (operatingState == 2) {
   digitalWrite(currentFlowLED, LOW);
   digitalWrite(currentZeroLED, HIGH);
   digitalWrite(cardPresentLED, HIGH);
   digitalWrite(writeProtectLED, LOW);
   lcd.clear();
   lcd.setCursor(3,0);
   lcd.print("NOT LOGGING");
   lcd.setCursor(3,1);
   lcd.print("LOW CURRENT");
}
while (operatingState == 2){
} //wait for opera