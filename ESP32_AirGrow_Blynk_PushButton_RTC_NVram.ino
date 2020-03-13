//#define BLYNK_USE_DIRECT_CONNECT
//#define BLYNK_DEBUG
#define BLYNK_TIMEOUT_MS  500  // must be BEFORE BlynkSimpleEsp8266.h doesn't work !!!
#define BLYNK_HEARTBEAT   17   // must be BEFORE BlynkSimpleEsp8266.h works OK as 17s

#include "RTClib.h"
//#include <SimpleTimer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <WidgetRTC.h>
//****** OTA ****************//
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h> 
//****** BME-280*************//
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//********Light Sensor******//
#include <ErriezBH1750.h>
#include <HCSR04.h>


#define SEALEVELPRESSURE_HPA (1013.25)


//***********Define Push Button and Relay****************************//
int Fan_PB = 18;
int Pump_PB = 23;
int Light_PB = 32;

int FanRelay = 16;
int PumpRelay = 17;
int LightRelay = 19;

int FanState, PumpState, LightState, MainPower;                          
int FanButtonState = LOW;     // the current state of the output pin
int PumpButtonState = LOW;
int LightButtonState = LOW;
int lastFanButtonState, lastPumpButtonState, lastLightButtonState; // the previous reading from the input pin

unsigned long lastDebounceTime1 = 0;
unsigned long lastDebounceTime2 = 0;
unsigned long lastDebounceTime3 = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;  // the debounce time; increase if the output flickers


//****************** Blynk LCD *********************************//
long previousBlynkLCDMillis = 0;    // for LCD screen update
long BlynklcdInterval = 4000;       // 4sec interval beween display
int Blynkscreen = 0;   
int BlynkscreenMax = 3;         // Maximum screen to show
bool BlynkscreenChanged = true; // initially we have a new screen,  by definition
#define B_TEMP_HUM    0         // defines of the screens to show
#define B_ALT_PRES    1
#define B_WAT_LUX     2



//***************** Water Level Variable ***************************// 
const int Length = 100;     //Length of tank is 100cm/1m
const int Width = 30;       //Width of tank is 30cm/0.3m           
const int Depth = 50;       //total depth of tank in cm , from sensor to base inside 
int Litres = 0;
int Distance, WaterDepth;


uint16_t lux;
int Temperature, Pressure, Altitude, Humidity;
const int analogInPin = 36;  // Analog input pin for MQ135 Air Quality Sensor
int AirQuality = 0;        // value read from MQ135 Air Quality Sensor

//*********** Moisture Measurement Variable **********************//
const int ToneOutput1 = 25;    // pin 13 is used for sending a 600 kHz PWM tone to the MoistSensor1 measurement circuit
const int freq = 600000;
const int Channel1 = 1;
const int resolution = 8;
const int MoistSensor1Pin1 = 34;// 12 bits ADC pin 15 senses the voltage level of the MoistSensor1 (values 0 - 4095)
const int MoistSensor1Pin2 = 35;// 12 bits ADC pin 4 senses the voltage level of the MoistSensor1 (values 0 - 4095)
int Moistlevel1, Moistlevel2, MoistAvg1, MoistAvg2;
int Counter1 = 0;
int Counter2 = 0; 

Adafruit_BME280 bme; // define bme280 temp/hum sensor
BH1750 sensor(LOW);  // define bh1750 light sensor
UltraSonicDistanceSensor distanceSensor(27, 26);  //Initialize UltraSonic sensor that uses pins Trig-27 and Echo-26.
RTC_DS1307 rtc;
BlynkTimer timer;
WidgetLCD lcd(V1);
WidgetRTC RTC;

bool online = 0;  // Check Blynk Online/Offline
String currentTime; //Time from RTC DS_1307

char auth[] = "WqKGAdpJM8c6imqepzqUhi0yN8oCq3FY";
char ssid[] = "AndroidAP";
char pass[] = "12345678";



BLYNK_CONNECTED() {
  RTC.begin();
}

BLYNK_WRITE(V2) {
  TimeInputParam t(param);
  
  int SThourV2 = t.getStartHour();   
  int STminV2  = t.getStartMinute(); 
  int STsecV2  = t.getStartSecond();
  
  int SPhourV2 = t.getStopHour();   
  int SPminV2  = t.getStopMinute();   
  int SPsecV2  = t.getStartSecond();

  rtc.writenvram(45, SThourV2);
  rtc.writenvram(46, STminV2);
  rtc.writenvram(47, SPhourV2);
  rtc.writenvram(48, SPminV2);
  }

BLYNK_WRITE(V3) {
  TimeInputParam t(param);

  int SThourV3 = t.getStartHour(); 
  int STminV3  = t.getStartMinute();
  int STsecV3  = t.getStartSecond();
  
  int SPhourV3 = t.getStopHour();   
  int SPminV3  = t.getStopMinute(); 
  int SPsecV3  = t.getStartSecond();

  rtc.writenvram(50, SThourV3);
  rtc.writenvram(51, STminV3);
  rtc.writenvram(52, SPhourV3);
  rtc.writenvram(53, SPminV3);
  }

BLYNK_WRITE(V0)       //Vpin for Main Power control
{
   MainPower  = param.asInt(); 
   if ( MainPower == HIGH )  {
     MainPowerHIGH();
    }
   
   else if ( MainPower == LOW )   {
     MainPowerLOW();
    }
 }

BLYNK_WRITE(V10)       //Vpin for Fan
{
  FanState = param.asInt(); 
  Fanloop();
 }

BLYNK_WRITE(V11)       //Vpin for Pump 
{
  PumpState = param.asInt(); 
  Pumploop();
 }

BLYNK_WRITE(V12)       //Vpin for Light
{
  LightState = param.asInt(); 
  Lightloop();
}

BLYNK_WRITE(V13)    //Time Sync Push Button
{
  int TimeSync = param.asInt(); 
  if (TimeSync == HIGH)  {
  rtc.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));  
  }
}

void TimeInput ()     {
//************ Read Time from Blynk RTC *******************************//
  char Date[16];
  char Time[16];
  sprintf(Date, "%02d/%02d/%04d",  day(), month(), year());
  sprintf(Time, "%02d:%02d:%02d", hour(), minute(), second());       
//************ Read set time from DS_1307 NVRAM ***********************//  
  DateTime now = rtc.now();
  int STHV2 = rtc.readnvram(45); //STH-StarT Hour from V2
  int STMV2 = rtc.readnvram(46); //STH-StarT Minut from V2
  int SPHV2 = rtc.readnvram(47); //SPH-StoP Hour from V2
  int SPMV2 = rtc.readnvram(48); //SPH-StoP Minut from V2
  int STHV3 = rtc.readnvram(50); //STH-StarT Hour from V3
  int STMV3 = rtc.readnvram(51); //STH-StarT Minut from V3
  int SPHV3 = rtc.readnvram(52); //SPH-StoP Hour from V3
  int SPMV3 = rtc.readnvram(53); //SPH-StoP Minut from V3
//************ Time based events *************************************//
  if ( STHV2 == now.hour() && STMV2 == now.minute() ) 
     {    MainPowerHIGH();   }
  if ( SPHV2 == now.hour() && SPMV2 == now.minute() )    
     {    MainPowerLOW();    }
  if ( STHV3 == now.hour() && STMV3 == now.minute() ) 
     {    MainPowerHIGH();   }
  if ( SPHV3 == now.hour() && SPMV3 == now.minute() )    
     {    MainPowerLOW();    }     

//************ Print RTC_DS1307 Time *******************************//  
  currentTime = String(now.hour()) + ":" + now.minute() + ":" + now.second(); //For Blynk Virtual Write-V37
  String currentDate = String(now.day()) + "/" + now.month() + "/" + now.year();
  Serial.print("Current Date/time: ");
  Serial.print(currentDate);
  Serial.print(" ");
  Serial.print(currentTime);
  Serial.println();
}

void AfterReboot()    {
  DateTime now = rtc.now();
   int STHV2 = rtc.readnvram(45); //STH-StarT Hour from V2
   int STMV2 = rtc.readnvram(46); //STH-StarT Minut from V2
   int SPHV2 = rtc.readnvram(47); //SPH-StoP Hour from V2
   int SPMV2 = rtc.readnvram(48); //SPH-StoP Minut from V2
   int STHV3 = rtc.readnvram(50); //STH-StarT Hour from V3
   int STMV3 = rtc.readnvram(51); //STH-StarT Minut from V3
   int SPHV3 = rtc.readnvram(52); //SPH-StoP Hour from V3
   int SPMV3 = rtc.readnvram(53); //SPH-StoP Minut from V3
                                       
  if ( (STHV2 >= now.hour() <= SPHV2) && ( STMV2 >= now.minute() <= SPMV2) ) {
         MainPowerHIGH();   }    
  if ( (STHV3 >= now.hour() <= SPHV3) && ( STMV3 >= now.minute() <= SPMV3) ) {
         MainPowerHIGH();   }
  if ( (SPHV2 >= now.hour() <= STHV3) && ( SPMV2 >= now.minute() <= STMV3) ) {
         MainPowerLOW();    }    
  if ( (SPHV3 >= now.hour() <= STHV2) && ( SPMV3 >= now.minute() <= STMV2) ) {
         MainPowerLOW();    }
}

void MainPowerHIGH()  {   
     PumpState   = HIGH;
     FanState    = HIGH;
     LightState  = HIGH;
     Blynk.virtualWrite(V0, HIGH);
     Fanloop();
     timer.setTimeout(500, []() {
     Pumploop();
     });
     timer.setTimeout(1000, []() {
     Lightloop();
     });
   }

void MainPowerLOW()  {   
    PumpState   = LOW;
    FanState    = LOW;
    LightState  = LOW;
    Blynk.virtualWrite(V0, LOW);
    Fanloop();
    timer.setTimeout(500, []() {
    Pumploop();
     });
    timer.setTimeout(1000, []() {
    Lightloop();
     }); 
    }
  
void Fanloop()    {
      digitalWrite(FanRelay, FanState);
      Blynk.virtualWrite(V10, FanState);
      }

void Pumploop()    {      
      digitalWrite(PumpRelay, PumpState);
      Blynk.virtualWrite(V11, PumpState);
      }

void Lightloop()   {     
      digitalWrite(LightRelay, LightState);
      Blynk.virtualWrite(V12, LightState);
      }

//**********************************************************************************//
//************ Push Button Loop ****************************************************//
//**********************************************************************************//
//------------------------------FAN--------------------------------------------------//
void PushButton()  {
  int reading1 = digitalRead(Fan_PB);
  if (reading1 != lastFanButtonState) {
    lastDebounceTime1 = millis();
}
  if ((millis() - lastDebounceTime1) > debounceDelay) {
    if (reading1 != FanButtonState) {
      FanButtonState = reading1;
      if (FanButtonState == HIGH)  {
        FanState = !FanState;
        Fanloop();
  }
 }
}
    lastFanButtonState = reading1;
  
  //---------------------------PUMP-----------------------------------------------//
  int reading2 = digitalRead(Pump_PB);
  if (reading2 != lastPumpButtonState) {
    lastDebounceTime2 = millis();
  }
  if ((millis() - lastDebounceTime2) > debounceDelay) {
    if (reading2 != PumpButtonState) {
      PumpButtonState = reading2;
      if (PumpButtonState == HIGH) {
        PumpState = !PumpState;
        Pumploop();   
      }
    }
  }
     lastPumpButtonState = reading2;
  
  //-------------------------LIGHT------------------------------------------------//
   int reading3 = digitalRead(Light_PB);
    if (reading3 != lastLightButtonState) {
    lastDebounceTime3 = millis();
  }
  if ((millis() - lastDebounceTime3) > debounceDelay) {
    if (reading3 != LightButtonState) {
      LightButtonState = reading3;
      if (LightButtonState == HIGH) {
        LightState = !LightState;
        Lightloop();
      }
    }
  }
     lastLightButtonState = reading3;
}



//******************************************************************************//
//**************   Setup Function  *********************************************//
//******************************************************************************//
void setup() {

  Serial.begin(9600);
  ArduinoOTA.setHostname("AirGrow");

  pinMode(Fan_PB, INPUT);
  pinMode(Pump_PB, INPUT);
  pinMode(Light_PB, INPUT);
  pinMode(FanRelay, OUTPUT);
  pinMode(PumpRelay, OUTPUT);
  pinMode(LightRelay, OUTPUT);
  
  bme.begin(0x76); //Initialize bme-280 Sensor
  Wire.begin();    //Initialize I2C bus
  sensor.begin(ModeContinuous, ResolutionHigh);  // Initialize sensor in continues mode, high 0.5 lx resolution
  sensor.startConversion();  // Start conversion

  rtc.begin();
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  AfterReboot();

  ledcSetup(Channel1, freq, resolution);    // configure the PWM functionalitites
  ledcAttachPin(ToneOutput1, Channel1);     // attach the channel 1 to the GPIO pin 25 for generating a PWM signal of 600kHz
  pinMode(MoistSensor1Pin1, INPUT);    // Initializes the Moisure sensor pin for measuring the MoistureLevelValue1
  pinMode(MoistSensor1Pin2, INPUT);  
   
  Blynk.config(auth);
  CheckConnection(); 

  timer.setInterval(1000L, TimeInput);
  timer.setInterval(5000L, CheckConnection); 
  timer.setInterval(1000L, ReadSensor);
  timer.setInterval(1500L, myTimerEvent); 
  timer.setInterval(4000, MeasureAirQualWater);
  timer.setInterval(900000, MEASUREMOISTURE);
  // timer.setInterval(60000L, BlynkNotify);
}


//**************************************************************************************//
//*************************** READ SENSORS  ********************************************//
//*************************************************************************************//
void ReadSensor() {
//************************ BME-280 ******************************************//  
     Temperature = bme.readTemperature();
     
     Pressure = ( bme.readPressure() / 100.0F );
    
     Altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    
     Humidity = bme.readHumidity();


//************************ LUX Sensor ***************************************//     
     int Read = sensor.read();
         lux = Read/2;
  }

//************************ Air Quality ***************************************//
void MeasureAirQualWater()   { 
    int AirQualityRead = analogRead(analogInPin);
     AirQuality = map ( AirQualityRead, 0, 4095, 0, 1023);
     Serial.printf("AirQuality reading: %d\n", AirQuality);

     
//********************** Ultrasonic Sensor **********************************//  
     Distance = distanceSensor.measureDistanceCm();
     if (Distance <= Depth && Distance >= 20 )  {
     WaterDepth = Depth - Distance  ;            //calculate the depth of the water
     int Volume = Length * Width * WaterDepth ;  // calculate Volume of Tank
     Litres = (Volume  / 1000);                 //calculate the volume of the water in litres
  }
}


//********************* Moisture Sensor ************************************// 
void MEASUREMOISTURE ()  {

  Moistlevel1 = 0;
  Moistlevel2 = 0;
  ledcWrite(Channel1, 128);     // send a PWM signal of 600 kHz to pin 25 with a dutycycle of 50%

  timer.setTimeout(200, []() {  // allow the circuit to stabilize                                            
  timer.setTimer(1000, []() {   // take 5 consecutive measurements in 5 seconds
  Moistlevel1 = Moistlevel1 + analogRead(MoistSensor1Pin1) ;    // Read data from analog pin 34 
  Counter1 = Counter1 + 1;
  }, 5);
  timer.setTimer(1000, []() {   // take 5 consecutive measurements in 5 seconds
  Moistlevel2 = Moistlevel2 + analogRead(MoistSensor1Pin2) ;    // Read data from analog pin 35
  Counter2 = Counter2 + 1;
  }, 5);
 });
}

void Moistloop()   {   
//////////////// Moisture Sensor-1 /////////////////////////////////
    if (Counter1 == 5) {
    Moistlevel1 = Moistlevel1 / 5;                       // Determine the average of 5 measurements 
    Serial.print("PinValue1:");
    Serial.print(Moistlevel1);
    MoistAvg1 = map ( Moistlevel1, 3100, 300, 0, 100);   // Map sensor-1 value, from 0% to 100%
    Serial.print("   ");
    Serial.print("MoistLvl:");
    Serial.print(MoistAvg1);
    Serial.println("%");
    Counter1 = Counter1 + 1;
} 
//////////////// Moisture Sensor-2 /////////////////////////////////
    if (Counter2 == 5) {
    Moistlevel2 = Moistlevel2 / 5;                       // Determine the average of 5 measurements 
    Serial.print("PinValue2:");
    Serial.print(Moistlevel2);
    MoistAvg2 = map ( Moistlevel2, 3000, 400, 0, 100);   // Map sensor-2 value, from 0% to 100%
    Serial.print("   ");
    Serial.print("MoistLv2:");
    Serial.print(MoistAvg2);
    Serial.println("%");
    Counter2 = Counter2 + 1;
}
    if ( Counter1 == 6 && Counter2 == 6)  {
    ledcWrite(Channel1, 0);        // stop generating PWM tones at pin 25
    Counter1 = 0;
    Counter2 = 0;
 }
}


//**********************************************************************************//
//*************Print Values to Blynk LCD and Serial***************************************//
//**********************************************************************************//
void PrintValues()   {
  unsigned long currentBlynkLCDMillis = millis();

  // MUST WE SWITCH SCREEN?
  if(currentBlynkLCDMillis - previousBlynkLCDMillis > BlynklcdInterval)   { // save the last time you changed the display
  previousBlynkLCDMillis = currentBlynkLCDMillis;
  Blynkscreen++;
  if (Blynkscreen > BlynkscreenMax) Blynkscreen = 0;  // all screens done? => start over
  BlynkscreenChanged = true;
 }
  //Serial.println(screen);
  // DISPLAY CURRENT SCREEN
  if (BlynkscreenChanged)  { // only update the screen if the screen is changed.
  BlynkscreenChanged = false; // reset for next iteration

 switch(Blynkscreen)  {
    
  case B_TEMP_HUM:
   lcd.clear(); 
   lcd.print(0, 0, "Tempe: " );   // position X: 0-15, position Y: 0-1, "Message you want to print"
   lcd.print(8, 0, Temperature );
   lcd.print(10, 0," °C" );
   Serial.print("Temperature = ");
   Serial.print(Temperature);
   Serial.println(" *C");
   lcd.print(0, 1, "Humid: " );
   lcd.print(8, 1, Humidity );
   lcd.print(10, 1," %" );
   Serial.print("Humidity = ");
   Serial.print(Humidity);
   Serial.println(" %");
   break;
   
  case B_ALT_PRES:
   lcd.clear();
   lcd.print(0, 0, "Altitude: " ); // position X: 0-15, position Y: 0-1, "Message you want to print"
   lcd.print(9, 0, Altitude );
   lcd.print(12, 0," m" );
   Serial.print("Approx. Altitude = ");
   Serial.print(Altitude);
   Serial.println(" m");
   lcd.print(0, 1, "Pressure: " ); // position X: 0-15, position Y: 0-1, "Message you want to print"
   lcd.print(9, 1, Pressure );
   lcd.print(12, 1," hPa" );
   Serial.print("Pressure = ");
   Serial.print(Pressure);
   Serial.println(" hPa");
   break;

  case B_WAT_LUX:
   lcd.clear();
   if (Distance <= Depth && Distance >= 20 )  {
   lcd.print(0, 0, "Water: " );
   lcd.print(7, 0,Litres);
   lcd.print(13, 0,"Ltr" );
   Serial.println();
   Serial.print("Water:  ");
   Serial.print(Litres);
   Serial.print(" Ltr");
}
  else {
   lcd.print(0, 0, "Water: " );
   lcd.print(7, 0, "Error" );
   Serial.println("Water: False Reading!");
}
   lcd.print(0, 1, "Light: " );
   lcd.print(7, 1,lux);
   lcd.print(14, 1,"Lx" );
   Serial.println();
   Serial.print(F("Light: "));
   Serial.print(lux);
   Serial.println(F(" LUX"));
   break;
   
   default:
      // cannot happen -> showError() ?
   break;
    }
  }  
} 

void myTimerEvent()   {
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  if (online == 1)     {
 
  Blynk.virtualWrite(V5, Temperature);
  Blynk.virtualWrite(V7, AirQuality);
  Blynk.virtualWrite(V20, Litres); 
  Blynk.virtualWrite(V33, MoistAvg1);
  Blynk.virtualWrite(V34, MoistAvg2);  
  Blynk.virtualWrite(V37, currentTime); //Send Hardware Time (RTC DS1307) to App
  //Blynk.virtualWrite(V35, lux); 
  //Blynk.virtualWrite(V36, Pressure); 
  if (FanState == HIGH)   { Blynk.virtualWrite( V30, 2); }
                   else   { Blynk.virtualWrite( V30, 1); }
  if (PumpState == HIGH)  { Blynk.virtualWrite( V31, 2); }
                   else   { Blynk.virtualWrite( V31, 1); }
  if (LightState == HIGH) { Blynk.virtualWrite( V32, 2); }
                    else  { Blynk.virtualWrite( V32, 1); }                                       
  }
  else { 
    Serial.println("Working Offline!");  
  }
 }



//*********************************************************************************//
//*********** Blynk Notification***************************************************//
//*********************************************************************************//
/*
void BlynkNotify()    {

  delay(16000);
   if ( Fan == HIGH || Pump == HIGH || Light == HIGH )  {
   if (AirQuality > 150)   {
   Blynk.notify (" Air Quality Unhealthy! Index Over: 150!");
  }
  delay(16000);
  
    if ( Litres < 10 )   {
      Blynk.notify (" Water Level Low!");
      //delay(3000);
      //digitalWrite(PumpRelay, LOW); 
          
  } /*
  delay(16000);
  
    if ( MoistLv_1 < 40 || MoistLv_2 < 40 || MoistLv_3 < 40 || MoistLv_4 < 40)  {
      Blynk.notify(" Low Moisture! Water Me Please!");
  }   
  delay(16000);

  if (lux >= 15000)   {
    Blynk.notify(" Light off due to high Lux Level!");
    delay(3000);
    //digitalWrite(LightRelay, LOW);
  }
 }
}   */
void loop() { 
  ArduinoOTA.handle();
  if(Blynk.connected()){
  Blynk.run();
  }
  Moistloop();
  PushButton();
  PrintValues();
  timer.run();     // Initiates BlynkTimer
}

void CheckConnection(){    // check every 11s if connected to Blynk server
  if(!Blynk.connected()){
      online = 0;
      yield();
  if (WiFi.status() != WL_CONNECTED)    {
      Serial.println("Not connected to Wifi! Connect...");
      WiFi.begin(ssid, pass);
      ArduinoOTA.begin();
      delay(400); //give it some time to connect
    if (WiFi.status() != WL_CONNECTED)    {
      Serial.println("Cannot connect to WIFI!");
      online = 0;
      }
    else  {
      Serial.println("Connected to wifi!");
      }
    }
    
  if ( WiFi.status() == WL_CONNECTED && !Blynk.connected() )   {
      Serial.println("Not connected to Blynk Server! Connecting..."); 
      Blynk.connect();  // // It has 3 attempts of the defined BLYNK_TIMEOUT_MS to connect to the server, otherwise it goes to the enxt line 
    if(!Blynk.connected()){
      Serial.println("Connection failed!"); 
      online = 0;
      }
    else  {
      online = 1;
      }
    }
  }
  else  {
     Serial.println("Connected to Blynk server!"); 
     online = 1;    
  }
}
