.           v�DdJdJ  �DdJ    ..          v�DdJdJ  �DdJW#    �5 0 9 4 .  <t m p   ����  �����9 2 6 0 6  <7 6 2 8 5 2   1 2 �_ 1 F l a  <s h . i n o   4 9 �D a t a _  <L o g g e r   _ 0 �ATA_L~1TMP  w�DdJdJ  �DdJ�)  �_ 1 F l a  �s h . i n o     ���D a t a _  �L o g g e r   _ 0 �ATA_L~1INO  w�DdJdJ  �DdJ�)  �0 1 0 1 .  <t m p   ����  �����5 5 5 7 7  <7 4 0 0 8 9   8 0 �_ 1 F l a  <s h . i n o   4 7 �D a t a _  <L o g g e r   _ 0 �ATA_L~1TMP  lIEdJdJ  JEdJ,/  �_ 1 F l a  �s h . i n o     ���D a t a _  �L o g g e r   _ 0 �ATA_L~1INO  w�DdJdJ  JEdJ,/  �4 9 7 4 .  <t m p   ����  �����5 7 5 3 5  <0 6 0 5 9 9   1 0 �_ 1 F l a  <s h . i n o   5 1 �D a t a _  <L o g g e r   _ 0 �ATA_L~1TMP  {YJdJdJ  ZJdJW�  �_ 1 F l a  �s h . i n o     ���D a t a _  �L o g g e r   _ 0 �ATA_L~1INO  w�DdJdJ  ZJdJW�  �9 2 1 2 .  <t m p   ����  �����1 0 6 8 0  <2 4 4 1 8 9   2 5 �_ 1 F l a  <s h . i n o   6 4 �D a t a _  <L o g g e r   _ 0 �ATA_L~1TMP  �JdJdJ  �JdJY�  �_ 1 F l a  �s h . i n o     ���D a t a _  �L o g g e r   _ 0 �ATA_L~1INO  w�DdJ�J  �JdJY�                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  

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
  if ((digitalRe