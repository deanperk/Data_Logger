nsorValue = sensorValue + analogRead(A2);
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
  TCCR1A = 0;                       ǹ0     0f�;%��*���9����/�ǻ��^�1���"ß�O�'���nw�ߝ�����X��3y�u)��`�4Tk'�
O�{���W��GiT����4����_�a�� �; �u��+@d<H�&���I%��@�9�D�z�C����8�G-%iʻ˿F�M7�%L�t�6�n:�^@4�T����0�!���撐�K��^�C����<B���ٔ@�62�.�̵�8�k8z�=
f���`ٗ'~W�Zz�yҫ����M�K�����R������v��\��,�&�f�!���^A#�A|,z�Xx��@��o,I�h�,h�=��C�#���};r�7��/�GuI�G9��GU�Z����>�n����a=p�K�Ux1t?�|�O��߈�wW��f��'m��4:{��I���eҼ�P�Bi�d�{J-���͐�X�XK�)��\�=�C'xb��޲ͨ<A'��^D�x�EP��?NǷ��qb����H&����.b@R&��x	T���sY҆�|��|���Sė�� 5���~t`J��&c�gn�G>�����Uo�f�=yq�Ll"LSHu���8ڀ��ٯ����D���:2�Tg�����.�ec����;��0�Tcpa�D��s�
ҿ?������NzÝ����`�jk_�W+�gM���;�|��ϐ}F�����~�ԡJh��	���\��Kp�$�i��4M�eF�
R֜�hi 1�<NE��e�<���-;��l�Y���ѕRc�'��L���L�6� �39��Dډs��1�즟Zn�)�0Wϻ�d���aLTl��[>�g �6�b�P�G�e�п
�o��Ddk�� � ��!�m;��$��^)'F��¢b������s*u2���+C���� 1Xh�G�&[XY�C�� ��wMI�p�����	����]���� ����$<f�� gV4U��"U���ï�ӿ6��X����]��XsH��?��*+֤��������0��ˬ�O�c�����0�V^�ZOj��@X��Gge��p�9��Y��'@ ׅ��'A��Wh�Y��99/���e�7��H
�ZIa)a����=<K���D�Ҷ�>�?<Y�J���Gi��Q��}!H�~~�Ą���b ��B/�sl����i�*��QG��x|�gŢ�$ ('��(p�=����'���II�NV
É���Z3ɣ!�pa$�"�3��<���iCku��a ��t���7�/���j��"��.�HfS_��Ļ歗4N{�����>3k�ƌlP�$�TRTPi�����Ǻ
-T�(����6�i�GU���[�k����oo%��\=�+���˲*4�W�]R!i̲5�W�#8��\�ҧi��v\[�E���3x�V&��oE�^SZ��ܣy�f��� bA{�d)�*\`X��ʮ���H����Ŵ�y�a�Au�+��~���}�Ohm�mx�۔>�t)&>�?����b�Y��@��t�6>}�H/����$o�(��'�Sƅ|P�7+cZ�Ǟb�.� �g���;���8�)����M��
_G8;��Ā����o-������!�<��90�����DJ� fo2� �c�(����z��]8�_���C&�y@�{.)BU��T{�����m�����x,��ef���nUc�|�=7H�,�M��R���%Ƭ �Z��Zp�zM'�e�=�0tg��D4e���ŕL��	��^y�M�J���r�ǁ}�%��=�KRwO�i�������D�C��+��Xc�4��F���hH`����x�?l�YD� M�ҿM�V�i����f�%���-,��|����$�����f e}���������'����;u�5R�ę1:>ϓ|���#���9���x�-Rs��TR�:�v"�;e��I9����#�T����r���y�֥<����b�댕�D��.H�!�P�j�����)���,�Ѵ�~���[rK���3�kD2���u�J#9�w��g��������VKo���e ��Vhԧq�v��9tJ	ƈ���7O�4_�;��a!�/ xB�U�"'z�w�Kx�d(��9;��^���U�� ��}|Y�����v�զ|�[R~tBT^V�o6'���\F��\ٌ8��j�e�Ya>��uI4v�g�?,Ϣ�G�f1^J_|$�FEW���cx~�
����m2[=lܳb��t�g�zFi��)`9�Wj[���J$���Df֒�Z�����L���b1���	����.��3<������6f�Yg�W@i�L5��Ā����p�����dyj��{H��D��?ŭ~�A��N7�{K�:+Xh�c�����PM�Dܵ5��M�翟{��Sp�ù����vȾwn���K�J��4[�Ѻ��ֺ