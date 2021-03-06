#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

// A simple data logger for the Arduino analog pins

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 1000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    0 // Wait for serial input in setup()

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// The analog pins that connect to the sensors
#define photocellPin 0           // analog 0
#define tempPin 1                // analog 1
#define BANDGAPREF 14            // special indicator that we want to measure the bandgap

#define aref_voltage 3.3         // we tie 3.3V to ARef and measure it with a multimeter!
#define bandgap_voltage 1.1      // this is not super guaranteed but its not -too- off

RTC_DS1307 RTC; // define the Real Time Clock object

// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

// the logging file
File logfile;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);

  while(1);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.println();
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
#if WAIT_TO_START
  Serial.println("Type any character to start");
  while (!Serial.available());
#endif //WAIT_TO_START

  // initialize the SD card
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  Serial.println("card initialized.");
  
  // create a new file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
  
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);

  // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
#if ECHO_TO_SERIAL
    Serial.println("RTC failed");
#endif  //ECHO_TO_SERIAL
  }
  

  logfile.println("millis,stamp,datetime,light,temp,vcc");    
#if ECHO_TO_SERIAL
  Serial.println("millis,stamp,datetime,light,temp,vcc");
#endif //ECHO_TO_SERIAL
 
  // If you want to set the aref to something other than 5v
  analogReference(EXTERNAL);
}

void loop(void)
{
  DateTime now;

  // delay for the amount of time we want between readings
  delay((LOG_INTERVAL -1) - (millis() % LOG_INTERVAL));
  
  digitalWrite(greenLEDpin, HIGH);
  
  // log milliseconds since starting
  uint32_t m = millis();
  logfile.print(m);           // milliseconds since start
  logfile.print(", ");    
#if ECHO_TO_SERIAL
  Serial.print(m);         // milliseconds since start
  Serial.print(", ");  
#endif

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print(now.unixtime()); // seconds since 1/1/1970
  logfile.print(", ");
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print(":");
  logfile.print(now.second(), DEC);
  logfile.print('"');
#if ECHO_TO_SERIAL
  Serial.print(now.unixtime()); // seconds since 1/1/1970
  Serial.print(", ");
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print(":");
  Serial.print(now.second(), DEC);
  Serial.print('"');
#endif //ECHO_TO_SERIAL

  analogRead(photocellPin);
  delay(10); 
  int photocellReading = analogRead(photocellPin);  
  
  analogRead(tempPin); 
  delay(10);
  int tempReading = analogRead(tempPin);    
  
  // converting that reading to voltage, for 3.3v arduino use 3.3, for 5.0, use 5.0
  float voltage = tempReading * aref_voltage / 1024;  
  float temperatureC = (voltage - 0.5) * 100 ;
  float temperatureF = (temperatureC * 9 / 5) + 32;
  
  logfile.print(", ");    
  logfile.print(photocellReading);
  logfile.print(", ");    
  logfile.print(temperatureF);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(photocellReading);
  Serial.print(", ");    
  Serial.print(temperatureF);
#endif //ECHO_TO_SERIAL

  // Log the estimated 'VCC' voltage by measuring the internal 1.1v ref
  analogRead(BANDGAPREF); 
  delay(10);
  int refReading = analogRead(BANDGAPREF); 
  float supplyvoltage = (bandgap_voltage * 1024) / refReading; 
  
  logfile.print(", ");
  logfile.print(supplyvoltage);
#if ECHO_TO_SERIAL
  Serial.print(", ");   
  Serial.print(supplyvoltage);
#endif // ECHO_TO_SERIAL

  logfile.println();
#if ECHO_TO_SERIAL
  Serial.println();
#endif // ECHO_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}


                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     2pbd @  �  	   �d  x^�\y|Tս?����¾x�-X2d�K	6I �T[�IrI��g&l�z�"�k-����@�j"
��(u�Z}�V^Kպ��Z�~��s瞙{'y�}��������=�v/ ���ĸ��:�w�3oO��vNv�q�S�ݚu5�-��%�uzk�7��"�l����'j=�=z�nU$��-O�ix<�n��:Y c�����9Y�c=��[��U�i�~�Vm�sKd���]"����@ZDx���xb�G��_ ݸ�pKP�$��z�K��1�ߠ��o���"�N�1���e�iĸ�/�HɈ�SU�g��U�&M�()R$�3��=<�.e��@�,��A��$zl���;c���ȿQipr�f̉ĸ�� ��Zq��4�)�U���C"�H������-^���!t�X�9+�c,s=&�Z���A:VԽ<�������'��)s%�A���2��,s㉱]<>��-���v�����9$�Z�&�a:^87̹�9��xv�_2(s.�``"����ݺ������%�5�Vk���۴^��C�N]�(%�/�D��d k�K��(˰F�$�ԝ,�I��JKEP�l��C)U��_V�6��2H	���x:Ͼ�X�'�Qto�W\�k�(����Zw�2�$���Y�H�E�G�I��tH�r�����D~#�L�m�[��Rɚ�(��ի'�f�00"�f�x�,�2b<���Ѝ�y��b�]	,�19�J~��ב_��(#��W�L(?�"ҟ���6���E*)�!��b<N���,���j��k��,�S���^�̱����8��;Py�'%�D��iO��_餞�Vc;-�;�?h/a����5������y�J��!w�۞�]&��i�]��*b�D���@�72���x�@51��U�T)�'��d8CH����b�B6�:P���vu`*1��5ui�Vx�il�R�^��!�݁ ˌi�x	۷���z,�'�����<��S�.�#Ӊ��N���G��c�A�w	V=日.�toH��u8r21���"��G9�������;Č7e�9����9�5x5�������&K�0/JRB��\;�S��<��p��75 ŏͷ��1�!�@
���9r�z*1���)#ј_��ޕu-#��Yz�p���KI�dőg{QF,8#�ܕ�+�2r:1��+��+�\�W�fTe�:c�|����b<CO�ܣ�2�9)Ř?�1f�;��gf�it����y��tu������O2pf��̑�gg8�x�&��ef&1�3o��L�18S��L1ЙW2p�N�Lg�s�L��p��M&���Vqg.cE�G�љ?�3�;�:W,�#�tM�sE)8��s�,b�@}¥w�ƺ�e!)^�5�{uBO�`h"s�xw��g�7��'�؈��D���tB��U�Dk��h��t�m�T{[��00�`��6���l�ӟ�Ƹ��X45M0��$�������	0w��g�|���o��X͘}����՗��c��T�.[a��Uo:�/ӹB�ќ~٥�<X���	���uC�/F�G�2�L�������m9�!���'&c�e��=&#G�f2܉�G��E�k��P`J��\+]��EWŵ��To�)ڒ�K�VO�j�`�b��2?��RS5�[5U+�?���~�
v���b�N��:bѰ>���[V]孨��5�bA#����``J�o��aS<2����� щ�O��u�d�����A�W,�g[E1��f�r
f���/g'?�Z 4Af���.=�̭uH:�źMf��
ĉ���zb�G��9J�(QRP�D�ݝV�L�Z�Cr3WK���U7㷨z��j9�Og��ZN�m6s���;� T��Z�J7�/}L�ӫV��f�>M�^H��Qu��j9��X��A'�n$�;t�P]��Z�>�z3C���Y���MM\5_�:�o��R�`��9m۰�*i-ɏE*F�F֋b�j1ޢg
ݓ�+5	��W`!O���Cz�3��>p�M�x��tQ.��s�H?;�QU.���ʛ��m�OH�\Z�%��P���!�����O���**/pQ.����D�'%?���1B�bb�F���Q��nu�V��CF���/���W�<_B�b'�*/��eI��2�wت��X��B�0��3����.�N��E�+M�5�s����ם��gT�%�ѥ�j~NA6��S��]�$w &�m��b�yl@�.��m�-M�Y�@�W@�ǻj�q������=�/�)̦pz8�`K��/n���-��v � �mY�s���}$^��4��l�Wr��뮻H� ��p���s����{��ҟ���}��Д]ԩ�Y��Ž��|
ҟy>j�W�v2g^��d�>1>@�eȼ�\p�~^ݬM��C߿�aW�m $ 3���8s+Ӽ��"�X��tsOK�������Ne�����!]&Jt��a9�
�fTw�[o;����\b���MKNQ6}������8�o��,,MSm
����5�s#y� ��\�剬�A���<���J^MZ�q���R%T^ML:Egu��3�AZ�1�X���F�+S�v�y��7$�K)�華���Bi�Q)��32P*0H+0���9�ԉ�Tz*�mx(�&#����N