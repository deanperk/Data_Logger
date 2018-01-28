

 /*
Data Logger Code Version Proto 0.10
Designed to run on an Uno R3 type Arduino
Schematic Rev 2.0


Rev History
0.10 Proto 1

I/O DEFINITIONS
A0 - System battery voltage
A1 - Current Measurement of Hall Effect Transducer
A2 - Temperature Thermistor 1
A3 - Temperature Thermistor 2
A4 - I2C SDA
A5 - I2C SCL
D0 - Rx
D1 - Tx
D2 - (INT1 pin) Not used
D3 - (INT2 pin) Not used
D4 - Blue Current Sensor Status LED 
D5 - Yellow Current Sensor Status LED 
D6 - SD Card present LED (L1)
D7 - SD Card write protect
D8 - SD Card detect
D9 - Not used
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
 volatile byte operatingState = 0; //initialize op state to "not logging"
 boolean cardPresent = LOW;        //initialize to "no card installed"


//String Variables
 String firmwareVer="0.10";

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
sensorValue=sensorValue/5.0;                        //average of 5 readings
loadAmps = (sensorValue * 0.00488);                 //scaled to 5V full scale 
loadAmps = (loadAmps * 10);                         //scaled to 50A full scale
// loadAmps = abs(loadAmps);
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
sensorValue=sensorValue/5;                          //Average of 5 readings
sensorValue = 1023 / sensorValue - 1;
  sensorValue = seriesResistor / sensorValue;
  steinhart = sensorValue / thermistorNominal;      // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= bCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (temperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart; // Invert
  steinhart -= 273.15;  // convert to C
  steinhart = steinhart * 9.0 / 5.0 +32.0;          // convert to F
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
sensorValue=sensorValue/5;                          //Average of 5 readings
sensorValue = 1023 / sensorValue - 1;
  sensorValue = seriesResistor / sensorValue;
  steinhart = sensorValue / thermistorNominal;      // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= bCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (temperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C
  steinhart = steinhart * 9.0 / 5.0 +32.0;          // convert to F
  temperature2 = steinhart;
  return temperature2;
} //end of measureTemp2()

//check operating state function
byte checkOperatingState() {
//0 = No SD Card detected or write protect not enabled
//1 = Battery BackUp
//2 = Power Good Not Logging
//3 = Power Good Logging
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
  if ((digitalRead(cardDetect) == LOW) && (digitalRead(writeProtect) == HIGH)) {
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
ISR(TIMER1_COMPA_vect)                 //timer compare interrupt service routine
{
 checkOperatingState();               //
 timeCounter += 1;                     //increment countHour on each interrupt
 x = (timeCounter%300);                //test for 5 min interval
 if (x == 0){
  timeCounter = 0;                     //if 5 minutes, log data. ADD LOGGING 
 }
} //end of timer isr


/* ----------------------------------------------
 *SETUP LOOP
-------------------------------------------------*/                 

void setup() {
 
   analogReference(DEFAULT);
   Serial.begin(9600);                 
   lcd.begin(16,2);                    //set up LCD number of rows and columns
   pinMode(currentFlowLED,OUTPUT);     //blue current sensor OK LED
   digitalWrite(currentFlowLED,LOW);
   pinMode(currentZeroLED,OUTPUT);     //yellow current sensor fault LED
   digitalWrite(currentZeroLED,LOW);
   pinMode(chipSelect,OUTPUT);         //SD card chip select
   digitalWrite(chipSelect,LOW);
   pinMode(cardDetect,INPUT_PULLUP);   //SD card detect on data log shield
   pinMode(writeProtect,INPUT_PULLUP); //SD write protect on data log shield
   pinMode(cardPresentLED,OUTPUT);     //SD card present LED L1 on data log shield
   digitalWrite(cardPresentLED,LOW);

  // set up Timer 1 interrupt
  noInterrupts();                      // disable all interrupts
                                       //set timer1 i
void setup() {
  Serial.begin(9600);
  // initialize digital pin
  pinMode(8, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  
}

// the loop function runs over and over again forever
void loop() {
  digitalRead(8);
  if (digitalRead(8) == LOW) {
    digitalWrite(6, HIGH);
  }else{digitalWrite(6,LOW);
  }
  digitalRead(7);
  Serial.print("Card Detect = "); //for debug
  Serial.println(digitalRead(8)); //for debug
  Serial.print("Write Protect = "); //for debug
  Serial.println(digitalRead(7)); //for debug
  Serial.println("");
  delay(1000);
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           alue = 1023 / sensorValue - 1;
  sensorValue = seriesResistor / sensorValue;
  steinhart = sensorValue / thermistorNominal;      // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= bCoefficient;                        // 1/B * ln(R/Ro)
  steinhart += 1.0 / (temperatureNominal + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C
  steinhart = steinhart * 9.0 / 5.0 +32.0;          // convert to F
  temperature2 = steinhart;
  return temperature2;
} //end of measureTemp2()

//check operating state function
byte checkOperatingState() {
//0 = No SD Card detected or write protect not enabled
//1 = Battery BackUp
//2 = Power Good Not Logging
//3 = Power Good Logging
if (digitalRead(cardDetect))== LOW {
    digitalWrite(cardPresentLED , HIGH); //turn on card LED if installed
}
measureVoltage();
measureCurrent();
if ((digitalRead(cardDetect) == HIGH) || (digitalRead(writeProtect) == HIGH)) {operatingState = 0;
} else {
  if (sysVoltage < 10.0) {operatingState = 1;
     } else {
  if ((sysVoltage >= 10.0) && (loadAmps >= 0.5)){operatingState = 3;
     } else {
  if ((sysVoltage >= 10.0) && (loadAmps < 0.5)){operatingState = 2;
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
ISR(TIMER1_COMPA_vect)                 //timer compare interrupt service routine
{
  measureVoltage();                    //should this be "check op state"?
  measureCurrent();                    //should this be volatile vaiable?
 timeCounter += 1;                     //increment countHour on each interrupt
 x = (timeCounter%300);                //test for 5 min interval
 if (x == 0){
  timeCounter = 0;                     //if 5 minutes, log data. ADD LOGGING 
 }
} //end of timer isr


/* ----------------------------------------------
 *SETUP LOOP
-------------------------------------------------*/                 

void setup() {
 
   analogReference(DEFAULT);
   Serial.begin(9600);                 
   lcd.begin(16,2);                    //set up LCD number of rows and columns
   pinMode(currentFlowLED,OUTPUT);     //blue current sensor OK LED
   digitalWrite(currentFlowLED,LOW);
   pinMode(currentZeroLED,OUTPUT);     //yellow current sensor fault LED
   digitalWrite(currentZeroLED,LOW);
   pinMode(chipSelect,OUTPUT);         //SD card chip select
   digitalWrite(chipSelect,LOW);
   pinMode(cardDetect,INPUT_PULLUP);   //SD card detect on data log shield
   pinMode(writeProtect,INPUT_PULLUP); //SD write protect on data log shield
   pinMode(cardPresentLED,OUTPUT);     //SD card present LED L1 on data log shield
   digitalWrite(cardPresentLED,LOW);

  // set up Timer 1 interrupt
  noInterrupts();                      // disable all interrupts
                                       //set timer1 interrupt at 1Hz
  TCCR1A = 0;                       Ç¹0     0fÄ;%¨É*½Øø9ˆ˜ùù/ƒÇ»­£^­1·İ"ÃŸÈOÍ'Ÿõ¡nwÎßÆîğ§Æ÷X‰¿3yÈu)‰ø`ğ4Tk'Ø
O­{÷¬™WåêGiTˆ††·4ş”úı_€aã €; –u™ö+@d<H¹&£ÌÙI%ù@Å9ëD§z£C€©‘°8ÎG-%iÊ»Ë¿FßM7Â%Lıt¼6şn:š^@4ÑTê²µÔ0¡!ú¾¡æ’¤Kı¬^C´ËîÕ<B¼À„Ù”@ä62Ç.€Ìµ8êk8zş=
fìû`Ù—'~W…ZzİyÒ«®ªÑÕMÅKáºœı˜¨R˜ÿ´è”ÅvñƒÀ\,‰&öfñ!ü”…^A#¢A|,zXxò@ÿëo,Işh,h¡=”×Cå#ÄÏ};ró7Ü÷/§GuIä¼G9›àGUÔZ¶ÿù¸>Èn¼êŞ²a=pÆKÆUx1t?Í|ÓO”èßˆwW•fñ'mòĞ4:{½¹Iïş¬eÒ¼´PÅBiÅd×{J-è”ëÍÍ¹XÌXKé)Ïï\û=¾C'xb¬¨Ş²Í¨<A'”ğ^D˜x¸EP‰Ô?NÇ·İßqbˆÆô«H&±‹¬û.b@R&˜Íx	TŠÒâsYÒ†Å|ı°|µ”ïSÄ—Ğó 5»›œ~ÂŸt`J“…&c÷gnæG>÷“¹şŒUoôfâª=yqîLl"LSHuˆí£8Ú€Úá…Ù¯·ç÷ò“”D¡ƒî:2™TgÏ›€¶Ë. ecú‘½å;Í 0ÂTcpaóDŠŒsç
Ò¿?¿‰¤Œº”NzÃìØ‹Ì`ì§jk_î˜W+ágM³è;¨|­«Ï}F×‡’Éà¶~´Ô¡JhúÅ	õ