//
// programing of the u-blox 7m module from Arduino for timepulse TP-5
// hex was taken from u-center software configuration view
//
//
// plus, flashing LED connected to pin 10 indicates when enough sat's in view
// u-blox module to connect o 3 and 4 for using soft serial of Arduino
// CT2GQV 2019 mixing multiple libs and examples.



// copy code from the image because blogger screws completely any attempt of including the includes...
//Writen for pro micro
//These proved to be usefull 
//http://arduino.stackexchange.com/questions/1471/arduino-pro-micro-get-data-out-of-tx-pin
//https://forum.sparkfun.com/viewtopic.php?f=32&t=38889&sid=8178cdb38005ff33cc380a5da34fb583&start=15



#include <SoftwareSerial.h>

//SoftwareSerial lcd(10, 11); //RX (not used), TX


#include "TinyGPS.h"
//#include "si5351.h"
//S15351 s15351;12c 0x60
#include <Wire.h>
#include <LiquidCrystal.h>

int SATLED = 10;  // for showing we have satelites
int sats = 0 ;

TinyGPS gps;
 
SoftwareSerial sslcd(11,10); // RX, TX

//LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//LiquidCrystal_I2C lcd(0x27,2,1, 0, 4,5, 6,7) ;  // 4 = SDA 5 = SCL ???
//LiquidCrystal_I2C lcd(0x27,2,1, 0, 4,5, 6,7) ;  // 0x27 = Adr. 2=En  1 =RW  0=Rs  4= D4 5=D5  6=D6   7= D7  Backlightpin , t_backlighPol pol = POSITIVE   



const char UBLOX_INIT[] PROGMEM = {  
  
  
// the actual programing string, uncoment for the one needed. 10Mhz, 2.5Mhz, 24Mhz or 2Mhz
// any frequency not integer divide of 48Mhz will have some jitter since module reference is 48. Best use is for 24 or 2 Mhz

/*
 //*** CFG-TP5 1Hz / 10Mhz sync
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x96,
  0x98, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x08, 0x00, 0x00, 0x7E, 0xA8,
*/

/*
  //*** CFG-TP5 1Hz / 10Mhz no sync 50ms cable delay
  
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x80, 0x96, 0x98, 0x00, 
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xA8, 0x08,
*/

/* 
  //*** CFG-TP5 1Hz / 24 Mhz no sync 0ms cable delay
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x36, 0x6E, 0x01,
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0x6D, 0x8D,
*/

/*
 //*** CFG-TP5 1Hz / 2.5Mhz no sync 50ms cable delay
  0xB5, 0x62, 0x06, 0x31, 0x20, 0x00, 0x00, 0x01, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0xA0, 0x25, 0x26, 0x00, 
  0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x6F, 0x00, 0x00, 0x00, 0xE5, 0x21,
*/


// *********  Nur dieses grad ausgewählt 
  // CFG-TP5 1Hz / 2 Mhz no sync 50ms cable delay
  0xB5,0x62,// header
  0x06,0x31,// time pulse get/set  Time Pulse Parameters
  0x20,// lenght 32
  0x00,// tpIdx time pulse selection >>> 0 = timepulse1 , 1 = timepulse2  (U1 Char)
  0x00,// reserved0 U1
  0x01, 0x00,// reserved1 U2
  0x00, 0x32,// antCableDelay ns 
  0x00, 0x00,// rf group delay in nS I2 
  0x00, 0x01, 0x00, 0x00,// Frequency or period time, depending on setting of bit 'isFreq'
  0x00, 0x80, 0x84, 0x1E,// freqPeriodLoc  Frequency or period time when locked to GPS time, only used if 'lockedOtherSet' is set
  0x00,0x00, 0x00, 0x00,// pulselenRatio  , Pulse length or duty cycle, depending on 'isLength'
  0x80, 0x00, 0x00, 0x00,// pulselenRatio , Pulse length or duty cycle when locked to GPS time, only used if 'lockedOtherSet' is set
  0x80, 0x00, 0x00, 0x00,// userConfigDelay ns  User configurable time pulse delay
  0x00, 0x6F, 0x00, 0x00,// flags - page 135 u-blox 7 Receiver Description Including Protocol Specification V14.pdf
  0x00, 0x1C, 0x1E,

// Flags
// 0 = Active  , if set enable time pulse; if pin assigned to another function, other function takes precedence
// 1 = LockGpsFreq , if set synchronize time pulse to GPS as soon as GPS time is valid, otherwise use local clock
// 2 = lockedOtherSet , if set use 'freqPeriodLock' and 'pulseLenRatioLock' as soon as GPS time is valid and 'freqPeriod' and 'pulseLenRatio' if GPS time is invalid,
//                       if flag is cleared 'freqPeriod' and 'pulseLenRatio' used regardless of GPS time
// 3 = isFreq ,  if set 'freqPeriodLock' and 'freqPeriod' interpreted as frequency, otherwise interpreted as period
// 4 = isLength , if set 'pulseLenRatioLock' and 'pulseLenRatio' interpreted as pulse length, otherwise interpreted as duty cycle
// 5 = alignToTow , align pulse to top of second (period time must be integer fraction of 1s)
// 6 = polarity , pulse polarity: 0 = falling edge at top of second 1 = rising edge at top of second
// 7 = gridUtcGps , timegrid to use: 0 = UTC  1 = GPS

/*  
  //*** UBX-CFG-TP5 parameters (not fully complete)
  0xB5, 0x62, // header
  0x06, 0x31, // time pulse get/set
  0x20,  // lenght 32
  0x00, // tpIdx time pulse selection = 0 = timepulse, 1 = timepulse2  (U1 Char)
  0x00,  // reserved0 U1
  0x01, 0x00, // reserved1 U2
  0x00, 0x32, // antCableDelay ns 
  0x00, 0x00, // rf group delay I2 
  0x00, 0x90, 0xD0, 0x03, // freqPeriod
  0x00, 0x40, 0x42, 0x0F, // freqPeriodLoc
  0x00, 0xF0, 0x49, 0x02, // pulselenRatio
  0x00, 0x60, 0xAE, 0x0A, // pulselenRatio
  0x00, 0x00, 0x00, 0x00, // userConfigDelay ns
  0x00, 0x77, 0x00, 0x00, // flags - page 135 u-blox 7 Receiver Description Including Protocol Specification V14.pdf
  0x00, 0x48, 0x65,
*/  

};

void setup()
{  
 //pinMode(9, OUTPUT);  
  //digitalWrite(9, HIGH); 
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  } //Endwhile
 Serial1.begin(9600);

 lcd.begin(16, 2); // two 16 characters lines
 lcd.display();
 
 
  Serial.println("= sent init string to GPS =");
  lcd.setCursor(0, 0); lcd.print("NO DATA   "); 
 

  
  lcd.begin(16, 2);                           // LCD set for 16 by 2 display
 lcd.display();
 
  //lcd.setBacklightPin(3,POSITIVE);            // (BL, BL_POL)
 // lcd.setBacklight(HIGH);                     // LCD backlight turned ON
  
  lcd.setCursor(0, 0);                        //
  

  pinMode(SATLED, OUTPUT); // to indicate we have enough satelites
  digitalWrite(SATLED, HIGH);
  delay(2000);
  digitalWrite(SATLED, LOW);

// actual u-blox 7m programing  
   for(int i = 0; i < sizeof(UBLOX_INIT); i++) {                        
  Serial1.write( pgm_read_byte(UBLOX_INIT+i) );
    delay(5); // simulating a 38400baud pace (or less), otherwise commands are not accepted by the device.
  }
// ends here   


} // End setup 
 

void loop()
{
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c = Serial1.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    sats = gps.satellites();
    lcd.setCursor(0, 0); lcd.print("Satellites:");lcd.print(sats);
    if (sats >= 3){digitalWrite(SATLED, HIGH); delay(20);digitalWrite(SATLED, LOW );};
    //if (sats < 3 ){digitalWrite(SATLED, LOW );};
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  }
  
  gps.stats(&chars, &sentences, &failed);
 if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
}
