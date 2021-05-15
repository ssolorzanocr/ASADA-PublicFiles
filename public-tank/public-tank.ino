/**
 * @file tank.ino
 * @author Sergio Solórzano Alfaro 
 * @coauthor Juan José Rojas
 * @date 15 May 2021
 * @brief Main loop is a state machine firmware for tank monitoring and LoRa Gateway
 * @par Institution:
 * DeltaLab. Instituto Tecnológico de Costa Rica.
 * @par mail:
 * solorzanos117@gmail.com
 * juan.rojas@tec.ac.cr
 * @par Git repository:
 * https://github.com/ssolorzanocr/ASADA-PublicFiles
 */
// JR: VERY Important ADC2 pins cannot be used when the WiFi is being used, and some of the ADC1 pins are also blocked for some reason, 32, 33 and 39 are ok...
/*SS: IMPORTANT: When using GPIO pins to receive PWM signals while using WiFi, I recommend using pin 2 and 17. 
*Any other may have internal hardware interference
*IMPORTANT >> Both High Level and Low Level sensors must be High in Air!*/

#include "heltec.h" 
#include <WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h" 
#include "soc/rtc.h"
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <time.h>

#define BAND    915E6   //you can set band here directly,e.g. 868E6 (Europe),915E6 (North America),433E6 (Asia)

#define WLAN_SSID       //name of your wifi                      
#define WLAN_PASS       //your wifi password              
#define AIO_SERVER      //adafruit server 
#define AIO_SERVERPORT  //the server number                 
#define AIO_USERNAME    //your adafruit IO username
#define AIO_KEY         //your adafruit IO key

//Micro TF card pins to ESP32 GPIOs via this connecthin:
#define SD_CS   22        //  SD_CS   -- GPIO22
#define SD_SCK  2         //  SD_SCK  -- GPIO2
#define SD_MOSI 23        //  SD_MOSI -- GPIO23
#define SD_MISO 17        //  SD_MISO -- GPIO17

#define DIGITAL_PULSE_1 13  //for hidric measures  

#define ADC_BATTERY     39  //battery voltage input
#define ADC_INPUT       38  //foreseen input     

#define DIGITAL_INPUT_1 36  //for low liquid level sensor 
#define DIGITAL_INPUT_2 37  //for high liquid level sensor 

#define RP      0.001          //IMPORTANT: Rp stands for "Pulse Resolution". Depends on flow meter parameters. 1 pulse = Rp cubic meters of water that have passed.
#define UNITS   1000000000.0  //calculate flow: (1000000*Rp/(sum_interval/pulses))[m3/s]; (3600000000*Rp/(sum_interval/pulses))[m3/h]; (1000000000*Rp/(sum_interval/pulses))[L/s]

/*-------- Core temperature variables -------*/
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();
/*------------------------------*/


WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

//IoT sistem water data
Adafruit_MQTT_Publish statusauto = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/statusauto");       
Adafruit_MQTT_Publish status15hp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pumps.status15hp");
Adafruit_MQTT_Publish status10hp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pumps.status10hp");
Adafruit_MQTT_Publish dmndflow = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/water.dmndflow");
Adafruit_MQTT_Publish dmndvol = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/water.dmndvol");
Adafruit_MQTT_Publish prodflow = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/water.prodflow");
Adafruit_MQTT_Publish prodvol = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/water.prodvol");
Adafruit_MQTT_Publish alerts = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/alerts");
Adafruit_MQTT_Publish tanklvl = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tanklvl");
Adafruit_MQTT_Publish lowsensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/lowsensor");
Adafruit_MQTT_Publish highsensor = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/highsensor");
Adafruit_MQTT_Publish mainrealwaterloses = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/mainrealwaterloses");

//IoT sistem LoRa and power funtion parameters data
Adafruit_MQTT_Publish volt2b = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sistemaiot.volt2b"); 
Adafruit_MQTT_Publish volt3b = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sistemaiot.volt3b"); 
Adafruit_MQTT_Publish volt4b = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sistemaiot.volt4b"); 
Adafruit_MQTT_Publish rssimeter = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sistemaiot.rssimeter"); 
Adafruit_MQTT_Publish rssipump = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sistemaiot.rssipump");
Adafruit_MQTT_Publish tankcoretemp = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sistemaiot.tankcoretemp"); 

Adafruit_MQTT_Subscribe control10hp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumps.control10hp");
Adafruit_MQTT_Subscribe control15hp = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumps.control15hp");
Adafruit_MQTT_Subscribe automatic = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/automatic");

char *strtok(char *str1, const char *str2); char *part; 
char charpacket [99]; char delimiter [2] = "e"; char ppack[8] = "start";
String State="1"; String tank; String sdelimiter="e";

/*----Local variables----*/
String pump15hp_status="OFF", pump10hp_status="OFF";  //pump On/Off changing variable, start with everything OFF
String mem_pump15hp, mem_pump10hp;        //pump On/Off last value

int8_t attempt = 0; int packetSize; 
uint32_t interval, lastinterval, sum_interval, init_stamp;
bool meter_flag, sd_flag; bool pump10hp_flag=1; bool first_flag = 0; bool message_flag = 0; bool callback_flag, firstflow_flag; 
bool reset_flag; //for volumes daily reset  
 
byte localAddress;          // address of this device
byte meter_destination;      // Node 1 destination address
byte pump_destination;      // Node 2 destination address
String incoming; 

/*----AIO publish variables----*/
char alert_message [70]; String mem_alert_message, route;
double pump15hp_flow, pump15hp_vol, pump10hp_flow, pump10hp_vol;  //water production variables
double rdemand_flow, rdemand_vol, sumrdemand_flow, rdemand_av, demand_flow, demand_vol, product_publish, demand_publish;   //water demand variables (volume is accumulative)         
double tank_lvl, mrwl, upmrwl, product_total, demand_total, rdemand_memvol;   //for tank level calculation**
double tank_battery_voltage, tank_battery_life, prod_battery_voltage, prod_battery_life, demand_battery_voltage, demand_battery_life; 

/*----AIO subscription variables----*/
String IoTsystem, mem_IoTsystem, pump15hp_IO, pump10hp_IO; 

/* create a hardware timer */
hw_timer_t * timer = NULL; 

bool upload_flag, alert_flag, losscalc_flag; int8_t data_flag=0;

/*-------------------------------------LoRa variables----------------------------------------*/
String packet;
int16_t airtime = 400;     //LoRa message estimated time in air in miliseconds
int8_t txPower = 20;       /*txPower -- 0 ~ 20 dBm*/
/** LoRa.setTxPower(txPower,RFOUT_pin);
  * txPower -- 0 ~ 20
  * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
  *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
  *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
----------------------------------------------------------------------------------------------*/ 

// The sd card can also use a virtual SPI bus
SPIClass sd_spi(HSPI);
long timezone = 1;
byte daysavetime = 1;

//Getting actual time and date from server  
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -25200;
const int   daylightOffset_sec = 3600; char today[3]= {'\0'}; char newday [3]= {'9'};
//char csvfirstrow [150] = "TimeStamp;P_RSSI;M_RSSI;P15_Flw;P15_Vol[m3];P10_Flw;P10_Vol[m3];D_Flw;D_Vol[m3];RD_Flw;RD_Vol[m3];Tank[m3];D_Bat;P_Bat;T_Bat;MCT[C];PCT[C];TCT[C]\n"
char buff[230] = {'\0'}; char value[15]; const char semicolon[] = ";"; char t_stamp[30] = {'\0'}; const char newline[] = "\n";
char pump_rssi[10] = {'\0'}; char meter_rssi[10] = {'\0'};

TaskHandle_t measureTask;     //create a water measure task in core 0.

void IRAM_ATTR reed_isr(){ 
  meter_flag = 1;
}

void IRAM_ATTR timer_isr(){
  upload_flag = 1;
}

// funtion to get time for sd data write ------------------------------------------------------------------------
void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  time_t rawtime;
  struct tm * tmstruct;
  time (&rawtime);
  tmstruct = localtime (&rawtime);
  printf ("Current local time and date: %s", asctime(tmstruct));
  sprintf(t_stamp,"%d-%02d-%02d %02d:%02d:%02d",(tmstruct->tm_year)+1900,( tmstruct->tm_mon)+1, tmstruct->tm_mday,tmstruct->tm_hour, tmstruct->tm_min, tmstruct->tm_sec);
  sprintf(today, "%d", (tmstruct->tm_wday));
  if (strcmp(today,newday)){
    sprintf(newday, "%d", (tmstruct->tm_wday));
    Serial.println("New Day Updated");
    strcpy(alert_message,"VOLUMENES REINICIADOS");
    alerts.publish(alert_message);
    reset_flag=1; //demand_vol=0; rdemand_vol=0; pump15hp_vol=0; pump10hp_vol=0;  //every new day reset acumulate volumes to cero
    if (!pump10hp_flag){                                          //every new day test if 10hp pump has worked. If not, turn on until tank is full.
      pump10hp_status="ON";
      strcpy(alert_message,"BOMBA 10HP ENCENDIDA POR INACTIVIDAD"); alert_flag=1;
      sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
      Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(80, 30, ppack); Heltec.display->display();
      for(int i = 0; i <= 5; i++){
        LoRaSend(ppack, pump_destination);
        listencallback();
        Serial.print("callback flag: "); Serial.println(callback_flag);
        if (callback_flag){
          callback_flag=0;
          break;
        }
      }
    }else{
      pump10hp_flag=0;
    }
  }
  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.print("Day of the week: "); Serial.println(today);
  Serial.print("NewDay of the week: "); Serial.println(newday);
  Serial.println(t_stamp);
  Serial.print("Pump10HP flag: ");Serial.println(pump10hp_flag);
  
}
// funtion to get time for sd data write ------------------------------------------------------------------------

/*-------------------------------------------------------------------------------------------*/
                            /*@brief This is the tank measuring setup of the program.*/   
void setup() {
  noInterrupts();     //Disable interrupts while initiating 
  Serial.begin(115200);
  delay(100);
  pinMode(DIGITAL_INPUT_2,INPUT); //HighSensor /*IMPORTANT High Level sensor must be High in Air!*/
  pinMode(DIGITAL_INPUT_1,INPUT); //LowSensor /*IMPORTANT Low Level sensor must be High in Air!*/
  pinMode(DIGITAL_PULSE_1, INPUT_PULLUP);
  attachInterrupt(DIGITAL_PULSE_1,&reed_isr,FALLING);
  
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach timer_isr function to our timer */
  timerAttachInterrupt(timer, &timer_isr, true);
  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 120000000, true);          //2 minutes timer for data request and upload 
  /* Start an alarm */
  timerAlarmEnable(timer);

  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->drawString(0, 20, "Beginning TANK program!");
  Heltec.display->display();

  Serial.print("setup running on core "); Serial.println(xPortGetCoreID());
  //create a task that will be executed in the measureTaskcode() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    measureTaskcode,  /* Task function. */
                    "measure",        /* name of task. */
                    10000,            /* Stack size of task in bites */
                    NULL,             /* parameter of the task */
                    1,                /* priority of the task */
                    &measureTask,     /* Task handle to keep track of created task */
                    0);               /* pin task to core */                  
  delay(200);
  
  // Setup MQTT subscription.
  mqtt.subscribe(&control10hp);
  mqtt.subscribe(&control15hp);
  mqtt.subscribe(&automatic);

/* SD Card Set up --------------------------------------------------------------*/
   // SD Card SamaraData.csv file initiation
  sd_spi.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, sd_spi)){
      Serial.println("SD Card: mounting failed.");
  } else {
      Serial.println("SD Card: mounted.");
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
      Serial.println("No SD card attached");
      sd_flag=0;
      return;   //exit setup
  }else{
    sd_flag=1;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
      Serial.println("MMC");
  } else if(cardType == CARD_SD){
      Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
      Serial.println("SDHC");
  } else {
      Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  //For new SD Cart use only
  //deleteFile(SD, "/SamaraData.csv");
  readFile(SD, "/SamaraData.csv");
  delay(100);
}

void tankdisplay(){
  route += " "; route += State;
  Heltec.display->clear();
  Heltec.display->drawString(0 , 0 , "IoT"); Heltec.display->drawString(20 , 0 , IoTsystem); Heltec.display->drawString(50 , 0 , "Tank:"); Heltec.display->drawString(80 , 0 , tank);
  Heltec.display->drawString(0 , 10 , String (rdemand_flow,2)); Heltec.display->drawString(30 , 10 , "L/s"); Heltec.display->drawString(60 , 10 , String (rdemand_vol,2)); Heltec.display->drawString(110 , 10 , "m3");
  Heltec.display->drawString(0, 50, "ST:"); Heltec.display->drawString(20, 50, route); if (sd_flag){ Heltec.display->drawString(100, 50, "SD: OK");}
  Heltec.display->display();
}

      /*-------------------------------------------------------------------------------------------*/
                            /*@brief This is the tank measuring loop of the program.*/                              
void measureTaskcode( void * pvParameters ){
  delay(7000);        //waits for everything else to start
  Serial.print("measureTask() running on core "); Serial.println(xPortGetCoreID());
  interrupts();       //Enable interrupts

  for(;;){
    //After 71.6 minutes (2^32 micro seconds), the value you get from micros() will overflow (reset to zero).
    //Acording to Octave –After 1 minute of no flow, the meter will flash a series of letters and numbers in place of the rate of flow to show the checksum of the software version. 
    if(first_flag){                         //Check only when measuring
      if ((micros()-init_stamp)>120000000){  //Stop and reset measure process after 2 minutes of no PULSES. **Min flow posibly measured = RP*1000/120 [L/s]
        first_flag=0; 
        rdemand_flow = 0;
        sum_interval+=(micros()-init_stamp);    //Add last large time interval for average calc 
        
        if(firstflow_flag){
          rdemand_av = UNITS*(rdemand_vol-RP)/sum_interval;   //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          rdemand_av = UNITS*(rdemand_vol)/sum_interval;
        }
        Serial.println("Stop");                 //Measuring process stops in order to save memory
        //printing>> 
        //time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage
        Serial.print(",");Serial.print(lastinterval);Serial.print(","); Serial.print (rdemand_vol,3); Serial.print(","); 
        Serial.print (rdemand_flow,4); Serial.print(","); Serial.print(rdemand_av,4); Serial.println(",");
      }
    }
    if(meter_flag && !first_flag){  //Start measure process 
      first_flag=1; meter_flag=0; firstflow_flag=1; Serial.println("Start");
      init_stamp=micros();          //First time stamp
      rdemand_vol+= RP;                     //First volume measure
    }
    
    if(meter_flag && first_flag ){
      interval=abs(micros()-init_stamp);  //Time interval between pulses
      if (interval<10000){       //IGNORE SMALL INTERFERENCES, THAT CAUSES HUGE FLOW MEASURES < 0.3 sec (Pulse width is 6000 microseconds)
        //IMPORTANT: THIS PARAMETER MUST BE A LITTLE BIT LARGER THAN PULSE WIDTH ---> SMALL PULSE WIDTHS HELPS AT MEASURING
        //pulse width measured for 4in Octave is 450ms and for 2in Octave is 300ms
        meter_flag=0;
      }else{
        init_stamp=micros();        //Time stamp for next measure
        sum_interval+=interval;   //For average flow calc
        
        rdemand_flow = (UNITS*RP)/((double)interval); 
        
        rdemand_vol+= RP;                                 //plus Rp cubic meters per interruption

        if(firstflow_flag){
          rdemand_av = UNITS*(rdemand_vol-RP)/sum_interval;   //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          rdemand_av = UNITS*(rdemand_vol)/sum_interval;
        }
        
        meter_flag=0; lastinterval= interval;
        
        //printing>> time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage 
        Serial.print(",");Serial.print(lastinterval);Serial.print(","); Serial.print (rdemand_vol,3); Serial.print(","); 
        Serial.print (rdemand_flow,4); Serial.print(","); Serial.print(rdemand_av,4); Serial.println(",");
      } 
    }
    delay(10); //SMALL DELAY PREVENTS PROCESSOR FROM CRASHING WHEN NO MEASURING
  }
}

          /*-------------------------------------------------------------------------------------------*/
                              /*@brief This is the main loop of the program.*/
                              
void loop(){
  Serial.print("loopTask() running on core "); Serial.println(xPortGetCoreID());
  delay(2000); attempt=0; route="";
  
  Serial.print("Temperature: ");
  // Convert raw core temperature in F to Celsius degrees
  double coretemp = ((temprature_sens_read() - 32) / 1.8);
  Serial.print(coretemp,2); Serial.println(" C");

  Serial.print("Tank Level: "); Serial.println(tank_lvl,2); 
  
  while (State=="1"){ /*---------------------------------INITIAL STATE TO VERIFY CONNECTION TO THE INTERNET AND MQTT*/
    Serial.println("State 1"); tankdisplay();
    WiFi_connect();
    MQTT_connect();

    //init and get the time
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    printLocalTime();
    
    State="2";
  }
  
  while (State=="2"){
    Serial.println("State 2"); tankdisplay();
    AIOstate();
    
    if (mem_IoTsystem!=IoTsystem){
      mem_IoTsystem=IoTsystem;
      if (mem_IoTsystem=="ON"){
        statusauto.publish("ON");
      }else{
        statusauto.publish("OFF");
      }
    }
    if (IoTsystem=="ON"){ // Checks ON state of automatic system in adafruit IO
      Serial.print("Low Level Sensor = "); Serial.println(digitalRead(DIGITAL_INPUT_1)); Serial.print("High Level Sensor = "); Serial.println(digitalRead(DIGITAL_INPUT_2));
      //Now checks level sensors in main tank...
      if (!digitalRead(DIGITAL_INPUT_1)&&digitalRead(DIGITAL_INPUT_2)){ 
        /*Low Sensor inactive! and High Sensor Active! Full >= tank_LVL >= empty*/
        State="A1"; tank="Good";
        highsensor.publish("1");
        lowsensor.publish("0");
      }
      if (digitalRead(DIGITAL_INPUT_1)&&digitalRead(DIGITAL_INPUT_2)){ 
        /*Low Sensor Active! and High Sensor Active! tank_LVL <= empty*/
        State="B"; tank="Empty";
        highsensor.publish("1");
        lowsensor.publish("1");
      }
      if (!digitalRead(DIGITAL_INPUT_1)&&!digitalRead(DIGITAL_INPUT_2)){ 
        /*Low Sensor inactive! and High Sensor inactive! tank_LVL >= Full*/
        State="C"; tank="Full";
        highsensor.publish("0");
        lowsensor.publish("0");
      }
      if (digitalRead(DIGITAL_INPUT_1)&&!digitalRead(DIGITAL_INPUT_2)){ 
        /*Low Sensor Active! and High Sensor inactive! ERROR*/
        State="D"; tank="ERROR";
        highsensor.publish("0");
        lowsensor.publish("1");
      }                    
    }
    if (IoTsystem=="OFF"){        // Checks OFF state of automatic system in adafruit IO
      State="3"; tank="";
    }
  delay(1000);
  }

  while (State=="3"){ //IoT Pumps manual control by AIO
    Serial.println("State 3"); tankdisplay();
    //Heltec.display->drawString(40, 50, State); Heltec.display->display();
    AIOstate();
    if((pump15hp_status!=pump15hp_IO)||(pump10hp_status!=pump10hp_IO)){
      pump15hp_status=pump15hp_IO;
      pump10hp_status=pump10hp_IO;
    }
    sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
    LoRaSend(ppack, pump_destination);
    listencallback();
    delay(2000); //waits for error solution
  }

  /*-----------------------------------------------------------------------------------------------------*/  
  while (State=="A1"){ //Requesting total water demand
    Serial.println("State A1"); tankdisplay();
    strcpy(alert_message,"TANQUE BIEN");
    requestdemand();

    if ((demand_flow > pump15hp_flow)&&(pump15hp_flow)){       //TURNS ON BOTH PUMPS TO SUSTAIN HIGH DEMAND
      pump15hp_status="ON";
      pump10hp_status="ON";
      strcpy(alert_message,"AMBAS BOMBAS ENCENDIDAS POR ALTA DEMANDA"); alert_flag=1;
      sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
      Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(80, 30, ppack); Heltec.display->display();
      for(int i = 0; i <= 5; i++){
        LoRaSend(ppack, pump_destination);
        listencallback();
        Serial.print("callback flag: "); Serial.println(callback_flag);
        if (callback_flag){
          callback_flag=0;
          break;
        }
      } 
    }else{
      Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(80, 30, "SP"); Heltec.display->display();
      for(int i = 0; i <= 5; i++){
        LoRaSend("SP", pump_destination);   //REQUEST PUMP STATE WITHOUT COMANDING ANY ACTION
        listencallback();
        Serial.print("callback flag: "); Serial.println(callback_flag);
        if (callback_flag){
          callback_flag=0;
          break; 
        }
      }
    } 
  }
  
  while (State=="A2"){ //Checks if 15min have past since last data upload to AIO and updates AIO pumps status
    Serial.println("State A2"); tankdisplay();
    //Heltec.display->drawString(60, 50, State); Heltec.display->display();
    /*Serial.print("pump15hp_status =");Serial.println(pump15hp_status);
    Serial.print("mem_pump15hp =");Serial.println(mem_pump15hp);
    Serial.print("pump10hp_status");Serial.println(pump10hp_status);
    Serial.print("mem_pump10hp");Serial.println(mem_pump10hp);*/
    if (mem_pump15hp!=pump15hp_status){
      mem_pump15hp=pump15hp_status;
      if (mem_pump15hp=="ON"){
        status15hp.publish("ON");
      }else{
        status15hp.publish("OFF");
      }
    }
    if (mem_pump10hp!=pump10hp_status){
      mem_pump10hp=pump10hp_status;
      if (mem_pump10hp=="ON"){
        status10hp.publish("ON");
      }else{
        status10hp.publish("OFF");
      }
    }
    Serial.print("------- Upload flag = "); Serial.println(upload_flag);
    if (upload_flag){
      upload_flag = 0;
      State="A3";
    }else{
      State="1";
    }
  }
  while (State=="A3"){ //Requests all water data to upload to AIO
    Serial.print("State A3"); tankdisplay();
    data_flag=0;
    for (int i = 0; i < 10; i++){
      if (data_flag==0){
        requestdemand();
      }
      if (data_flag==1){
        requestproduction();
      }
      if (data_flag==2){
        State="4";
      }
    }
    if (data_flag!=2){
      State="1"; alert_flag=1;
      sprintf(alert_message, "%s%s",alert_message, "/ ERROR EN SOLICITUD");  //save an alert message to publish
    }
  }

  while (State=="B"){ //Tank Empty
    Serial.println("State B"); tankdisplay();
    strcpy(alert_message,"TANQUE VACIO"); alert_flag=1;
    mrwl= tank_lvl-100; //mrwl stands for Main Real Water Loses. In this case, is an approximate of real water loses in tank while discharging.
    tank_lvl=100; //aproximate volume of water when tank is empty (m^3)

    if (mrwl){    //only make loses calc one time, once is empty
      upmrwl=mrwl;
      losscalc_flag=1;
    }
    
    pump15hp_status = "ON";
    pump10hp_status = "OFF";
    sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
    Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(50, 30, ppack); Heltec.display->display();
    LoRaSend(ppack, pump_destination);
    listencallback();
    }
  
  while (State=="C"){ //Tank Full
    strcpy(alert_message,"TANQUE LLENO"); alert_flag=1;
    Serial.println("State C"); tankdisplay();
    
    mrwl= tank_lvl-200; //mrwl stands for Main Real Water Loses. In this case, is an approximate of real water loses in tank while is been filled.
    tank_lvl=200; //aproximate volume of water when tank is Full(m^3)
    
    if (mrwl){    //only make loses calc one time, once is full
      upmrwl=mrwl;
      losscalc_flag=1;
    }
    
    pump15hp_status = "OFF";
    pump10hp_status = "OFF";
    sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
    Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(50, 30, ppack); Heltec.display->display();
    LoRaSend(ppack, pump_destination);
    listencallback();
    }
  
  while (State=="D"){ //Low Sensor ERROR!
    Serial.println("State D"); tankdisplay();

    pump15hp_status = "OFF";
    pump10hp_status = "OFF";
    sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
    Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(50, 30, ppack); Heltec.display->display();
    LoRaSend(ppack, pump_destination);
    listencallback();
    State="1"; alert_flag=1;
    strcpy(alert_message, "SISTEMA AUTOMATICO APAGADO: ERROR EN SENSORES");  //save an alert message to publish  
  }
/*-----------------------------------------------------------------------------------------------------*/  

  
  while (State=="4"){ /*------------------------------------STATE FOR DATA UPLOAD TO IOT PLATFORM*/
    Serial.println("State 4");
    Heltec.display->clear();
    delay (100);
    demand_total += (rdemand_vol - rdemand_memvol);
    rdemand_memvol = rdemand_vol;
    sum_interval=0; //reset average flow calc and acumulated volume
    demand_publish= demand_vol + rdemand_vol;
    product_publish= pump15hp_vol + pump10hp_vol;
    tank_lvl+= (product_total - demand_total);   //aproximate calculation of water in tank
    product_total, demand_total = 0;
    delay (100);
    
    batteryVoltage();
    WiFi_connect();
    MQTT_connect();
    printLocalTime();

    Serial.println("t_stamp, pump_rssi, meter_rssi, pump15hp_flow, pump15hp_vol, pump10hp_flow, pump10hp_vol, demand_flow, demand_vol, rdemand_flow, rdemand_vol, tank_lvl, demand battery, prod battery, tank battery, tank core temp C");
    Serial.print(",");Serial.print(t_stamp);Serial.print(","); Serial.print (pump_rssi); Serial.print(","); 
    Serial.print (meter_rssi); Serial.print(","); Serial.print(pump15hp_flow,4); Serial.print(","); Serial.print(pump15hp_vol,2); Serial.print(",");
    Serial.print(pump10hp_flow,4); Serial.print(","); Serial.print(pump10hp_vol,2); Serial.print(","); Serial.print(demand_flow,4); Serial.print(","); Serial.print(demand_vol,2); Serial.print(",");
    Serial.print(rdemand_flow,4); Serial.print(","); Serial.print(rdemand_vol,2); Serial.print(","); Serial.print(tank_lvl,2); Serial.print(","); Serial.print(demand_battery_voltage,2); Serial.print(",");
    Serial.print(prod_battery_voltage,2); Serial.print(","); Serial.print(tank_battery_voltage,2); Serial.print(","); Serial.print(coretemp,2); Serial.println(",");
    
    if (sd_flag){ //when a SD Card is attached
      Serial.println(F("Saving in SD Card: "));
      // Now save values on SD Card Data file
      strcat(buff,t_stamp);             //time stamp char is 25 characters long 
      strcat(buff,semicolon);

      strcat(buff,pump_rssi);           //last rssi value from pump station is 10 characters long 
      strcat(buff,semicolon);

      strcat(buff,meter_rssi);          //last rssi value from meter station is 10 characters long
      strcat(buff,semicolon);
      
      dtostrf(pump15hp_flow,4,2,value); //double to char value with number of characters and decimal places
      strcat(buff,value);               //value with 6 characters >> 4 flow values = 6 characters *4 = 24 characters
      strcat(buff,semicolon);
      dtostrf(pump15hp_vol,10,2,value); //value with 12 characters >> 4 vol values = 12 characters *4 = 48 characters
      strcat(buff,value);
      strcat(buff,semicolon);
      
      dtostrf(pump10hp_flow,4,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(pump10hp_vol,10,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      
      dtostrf(demand_flow,4,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(demand_vol,10,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      
      dtostrf(rdemand_flow,4,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(rdemand_vol,10,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
  
      dtostrf(tank_lvl,5,2,value);    //value with 7 characters >> 1 tank level values = 1 characters *7 = 7 characters
      strcat(buff,value);
      strcat(buff,semicolon);
      
      dtostrf(demand_battery_voltage,4,2,value);
      strcat(buff,value);             //value with 7 characters >> 3 battery voltage values = 5 characters *3 = 21 characters
      strcat(buff,semicolon);
      
      dtostrf(prod_battery_voltage,4,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      
      dtostrf(tank_battery_voltage,4,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);

      dtostrf(coretemp,5,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      
      strcat(buff,newline);           //required buff size = 25 + 10 + 10 + 24 + 48 + 7 + 21 + 13 semicolon characters = 158 characters (in theory), however buff size is larger
      delay(100);
      appendFile(SD, "/SamaraData.csv", buff);
      memset(buff,0,sizeof(buff));

      State="2";
    }//else{
        
    Serial.println(F("\nUploading values: "));
    /*data upload to adafruit IO feeds - each feed has 3 tries for data upload */ 
    for(int i = 0; i < 3; i++){
      if (!dmndflow.publish(demand_flow+rdemand_av)) {                      //publish water total demand flow. Write feedname.publish(data) 
        Serial.println(F("dmndflow - Failed"));
        Heltec.display->drawString(30 , 0 , "dmndflow-Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      }else{
        Serial.println(F("dmndflow - OK!"));
        Heltec.display->drawString(30 , 0 , "dmndflow-OK!"); Heltec.display->display();
        break;
      }
    }

    for(int i = 0; i < 3; i++){
      if (!dmndvol.publish(demand_publish)) {                               //publish water total demand volume. Write feedname.publish(data)
        Serial.println(F("dmndvol - Failed"));
        Heltec.display->drawString(0 , 10 , "dmndvol-Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      }else{
        Serial.println(F("dmndvol - OK!"));
        Heltec.display->drawString(0 , 10 , "dmndvol-OK!"); Heltec.display->display();
        break;
      }
    }
    
    for(int i = 0; i < 3; i++){
      if (!prodflow.publish(pump15hp_flow + pump10hp_flow)) {               //publish water total production flow. Write feedname.publish(data)  
        Serial.println(F("prodflow - Failed"));
        Heltec.display->drawString(0 , 20 , "prodflow-Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      }else{
        Serial.println(F("prodflow - OK!"));
        Heltec.display->drawString(0 , 20 , "prodflow-OK!"); Heltec.display->display();
        break;
      }
    }
      
    for(int i = 0; i < 3; i++){
      if (!prodvol.publish(product_publish)) {                              //publish water total production volume. Write feedname.publish(data)
        Serial.println(F("prodvol - Failed"));
        Heltec.display->drawString(0 , 30 , "prodvol-Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      }else{
        Serial.println(F("prodvol - OK!"));
        Heltec.display->drawString(0 , 30 , "prodvol-OK!"); Heltec.display->display();
        break;
      }
    }

    for(int i = 0; i < 3; i++){
      if (!tanklvl.publish(tank_lvl)) {                                     //publish real water tank level data. Write feedname.publish(data)
        Serial.println(F("Failed-tank"));
        Heltec.display->drawString(0 , 40 , "tank-Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      }else{
        Serial.println(F("OK!-tank"));
        Heltec.display->drawString(0 , 40 , "tank-OK!"); Heltec.display->display();
        break;
      }
    }
    
    if(losscalc_flag){
      losscalc_flag=0;
      for(int i = 0; i < 3; i++){
        if (!mainrealwaterloses.publish(upmrwl)) {                            //publish main real water loses calculation. Write feedname.publish(data)
          Serial.println(F("Failed-mrwl"));
          Heltec.display->drawString(60 , 0 , "MRWL-Fail"); Heltec.display->display();
          mqtt.disconnect();
          WiFi_connect(); MQTT_connect();
        }else{
          Serial.println(F("OK!-tank"));
          Heltec.display->drawString(60 , 0 , "MRWL-OK!"); Heltec.display->display();
          break;
        }
      }   
    }

    for(int i = 0; i < 3; i++){ 
      if (!volt2b.publish(demand_battery_voltage)) {                        //publish demand_battery_voltage. Write feedname.publish(data)
        Serial.println(F("Meter baterry voltage - Failed"));
        Heltec.display->drawString(60 , 10 , "MBat-Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      }else{
        Serial.println(F("Meter baterry voltage - OK!"));
        Heltec.display->drawString(60 , 10 , "MBat-OK!"); Heltec.display->display();
        break;
      }
    }
    
    for(int i = 0; i < 3; i++){
      if (!volt3b.publish(prod_battery_voltage)) {                      //publish production_battery_voltage. Write feedname.publish(data)
        Serial.println(F("Pumps baterry voltage - Failed"));
        Heltec.display->drawString(60 , 20 , "PBat - Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      } 
      else {
        Serial.println(F("Pumps baterry voltage - OK!"));
        Heltec.display->drawString(60 , 20 , "PBat - OK!"); Heltec.display->display();
        break;
      }
    }

    for(int i = 0; i < 3; i++){
      if (!volt4b.publish(tank_battery_voltage)) {                      //publish tank_battery_voltage. Write feedname.publish(data)
        Serial.println(F("Tank baterry voltage - Failed"));
        Heltec.display->drawString(60 , 30 , "TBat - Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      } 
      else {
        Serial.println(F("Tank baterry voltage - OK!"));
        Heltec.display->drawString(60 , 30 , "TBat - OK!"); Heltec.display->display();
        break;
      }
    }

    for(int i = 0; i < 3; i++){
      if (!rssimeter.publish(meter_rssi)) {                      //publish rssi indicator for meter LoRa communication. Write feedname.publish(data)
        Serial.println(F("PUMP RSSI - Failed"));
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      } 
      else {
        Serial.println(F("PUMP RSSI - OK!"));
        break;
      }
    }

    for(int i = 0; i < 3; i++){
      if (!rssipump.publish(pump_rssi)) {                      //publish rssi indicator for pump LoRa communication. Write feedname.publish(data)
        Serial.println(F("PUMP RSSI - Failed"));
        Heltec.display->drawString(60 , 40 , "RSSI - Fail"); Heltec.display->display();
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      } 
      else {
        Serial.println(F("PUMP RSSI - OK!"));
        Heltec.display->drawString(60 , 40 , "RSSI - OK!"); Heltec.display->display();
        break;
      }
    }

    for(int i = 0; i < 3; i++){
      if (!tankcoretemp.publish(coretemp)) {                      //publish rssi indicator for pump LoRa communication. Write feedname.publish(data)
        Serial.println(F("Coretemp - Failed"));
        mqtt.disconnect();
        WiFi_connect(); MQTT_connect();
      } 
      else {
        Serial.println(F("Coretemp - OK!"));
        break;
      }
    }
    mqtt.processPackets(1000);
    if (reset_flag){
      demand_vol=0; rdemand_vol=0; pump15hp_vol=0; pump10hp_vol=0;  //every new day reset acumulate volumes to cero
      reset_flag=0;
    }
    firstflow_flag=0;   //reset first flow flag after data upload
    rdemand_av=0;       //reset average secondary demand data after data upload
    State="1";
  //}
  }
}
//-------------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------------------------------
void listencallback(){ /*LISTENING STATE*/
  LoRa.receive();                   //back into receive mode
  for(int i = 0; i < 20; i++){ // listen for 4 seconds in total)
    packetSize = LoRa.parsePacket();
    if (packetSize){
      break; 
    }
    delay(200);
  }
  onReceive(packetSize, pump_destination);
  
  if (message_flag){
    Serial.println(incoming);
    tankdisplay();
    Heltec.display->drawString(35, 30, incoming); Heltec.display->display();
    message_flag=0; attempt=0; callback_flag=1;
    if ((incoming=="ON"+sdelimiter+"OFF")||(incoming=="OFF"+sdelimiter+"ON")||(incoming=="ON"+sdelimiter+"ON")||(incoming=="OFF"+sdelimiter+"OFF")){ //update each pump received state
      int8_t p = 1;
      incoming.toCharArray(charpacket,80);
      part = strtok(charpacket, delimiter);
      while (part!=NULL) {    //message separation process
        if (p==1){
          pump15hp_status = part;
        }
        if (p==2){
          pump10hp_status = part;
        }
        p++;
        part=strtok(NULL, delimiter);
      }
      if (pump10hp_status=="ON"){
        pump10hp_flag=1;                        //flag to indicate if 10 hp pump has worked today
      }
      if ((incoming==ppack)||(!strcmp(ppack,"start"))){
        State="A2";       
      }else{
        alert_flag=1;
        Serial.println("SISTEMA EN MODO MANUAL O ERROR TABLERO EN BOMBEO");
        strcpy(alert_message, "SISTEMA EN MODO MANUAL O ERROR TABLERO EN BOMBEO");  //save an alert message to publish later
        State="A2";
      }
    }
  }
  Serial.print("pump15hp_status"); Serial.println(pump15hp_status);
  Serial.print("pump10hp_status"); Serial.println(pump10hp_status);
}

//------------------------------------------------------------------------------------------------------------------------------------------------
/*LoRa data requesting functions*/
void requestdemand(){
  Serial.println("Requesting Demand Data");
  Serial.print("State "); Serial.println(State);
  
  Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(50, 30, "D");  Heltec.display->display();
  LoRaSend("D", meter_destination);

  LoRa.receive();                   //back into receive mode
  for(int i = 0; i < 10; i++){ // listen for 2 seconds in total)
    packetSize = LoRa.parsePacket();
    if (packetSize){
      break;
    }
    delay(200);
  }
  onReceive(packetSize, meter_destination);

  if (message_flag){
    tankdisplay(); Heltec.display->drawString(0, 40, incoming); Heltec.display->display();
    incoming.toCharArray(charpacket,17);   
    int8_t p = 1; message_flag=0; attempt=0;
    part = strtok(charpacket, delimiter);
    while (part!=NULL) {
      if (p==1){
        demand_total= atof(part);    // FOR TANK LEVEL CALC
        if (State=="A3"){
          demand_vol += atof(part);    // RECEIVED PACKET TO FLOAT NUMBER
        }
      }
      if (p==2){
        demand_flow = atof(part);   // RECEIVED PACKET TO FLOAT NUMBER
      }
      if (p==3){
        demand_battery_voltage = atof(part);   // RECEIVED PACKET TO FLOAT NUMBER
      }
      p++;
      part=strtok(NULL, delimiter);
    }
    if (State=="A3"){
      delay(4000);
      Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(80, 30, "OKD");   Heltec.display->display();
      LoRaSend("OKD", meter_destination);
      data_flag=1;
    }else{
      State="A2";
    }
    Serial.print("demand_vol = "); Serial.println(demand_vol);
    Serial.print("demand_flow = "); Serial.println(demand_flow);
    demand_battery_life=100*((demand_battery_voltage-2.8)/1.37); //percentage 
    if (demand_battery_life>100){ demand_battery_life=100;}
    if (demand_battery_life<0){ demand_battery_life=0;}
    if (demand_battery_voltage < 3){
      alert_flag=1;
      strcpy(alert_message,"ALERTA: BATERÍA BAJA EN MACROMEDIDOR DEMANDA");
      Serial.println("ALERTA: BATERÍA BAJA EN MACROMEDIDOR DEMANDA");
    }
    Serial.print("\tDemand Battery: "); Serial.print(demand_battery_voltage,4); Serial.print("\t"); Serial.println(demand_battery_life);
  }  
}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void requestproduction(){
  Serial.print("State "); Serial.println(State);
  Serial.println("Requesting Production Data");
  Heltec.display->drawString(0, 30, "Sending: "); Heltec.display->drawString(50, 30, "P");  Heltec.display->display();                      
  LoRaSend("P", pump_destination);
  
  LoRa.receive();                   //back into receive mode
  for(int i = 0; i < 10; i++){ // listen for 2 seconds in total)
    packetSize = LoRa.parsePacket();
    if (packetSize){
      break;
    }
    delay(200);
  }
  onReceive(packetSize, pump_destination);
  
  if (message_flag){
    tankdisplay(); Heltec.display->drawString(0, 40, incoming); Heltec.display->display();
    incoming.toCharArray(charpacket,40);   
    int8_t p = 1; message_flag=0; attempt=0;
    part = strtok(charpacket, delimiter);
    while (part!=NULL) {
      if (p==1){
        pump15hp_flow = atof(part);    // RECEIVED PACKET TO FLOAT NUMBER
      }
      if (p==2){
        product_total = atof(part);    // FOR TANK LEVEL CALC
        pump15hp_vol += atof(part);   // RECEIVED PACKET TO FLOAT NUMBER
      }
      if (p==3){
        pump10hp_flow = atof(part);    // RECEIVED PACKET TO FLOAT NUMBER
      }
      if (p==4){
        product_total += atof(part);    // FOR TANK LEVEL CALC
        pump10hp_vol += atof(part);   // RECEIVED PACKET TO FLOAT NUMBER
      }
      if (p==5){
        prod_battery_voltage = atof(part);   // RECEIVED PACKET TO FLOAT NUMBER
      } 
      p++;
      part=strtok(NULL, delimiter);
    }
    Serial.print("pump15hp_flow = "); Serial.println(pump15hp_flow);
    Serial.print("pump15hp_vol = "); Serial.println(pump15hp_vol);
    Serial.print("pump10hp_flow = "); Serial.println(pump10hp_flow);
    Serial.print("pump10hp_vol = "); Serial.println(pump10hp_vol);
    Serial.print("Battery Voltage = "); Serial.println(prod_battery_voltage,4);
    prod_battery_life=100*((prod_battery_voltage-2.8)/1.37); //percentage 
    if (prod_battery_life>100){ prod_battery_life=100;}
    if (prod_battery_life<0){ prod_battery_life=0;}
    if (prod_battery_voltage < 3){
      alert_flag=1;
      strcpy(alert_message,"ALERTA: BATERÍA BAJA EN PRODUCCION");
      Serial.println("ALERTA: BATERÍA BAJA EN PRODUCCION");
    }
    
    if (State=="A3"){
      delay(4000);
      Heltec.display->drawString(0, 30, "Sending: ");  Heltec.display->drawString(80, 30, "OKP"); Heltec.display->display();
      LoRaSend("OKP", pump_destination);
      data_flag=2;
    }
    Serial.print("\tProduction Battery: "); Serial.print(prod_battery_voltage,4); Serial.print("\t"); Serial.println(prod_battery_life);
  }
}

//--------------------------------------------------------------------------------------------------------------------------------------------
void batteryVoltage(){
  /*full battery voltage = 4.17V, which corresponds to a reading of 2.085V
   *empty battery voltage = 2.80V, which corresponds to a reading of 1.4V
   */
  //Serial.println(analogRead(ADC_BATTERY)); 
  tank_battery_voltage=2*((analogRead(ADC_BATTERY)+200.01)/1263.4); // Aproximate ADC vs DC voltage function from Heltec LoRa 32 V2 real measures 
  tank_battery_life=100*((tank_battery_voltage-2.8)/1.37); //percentage 
  if (tank_battery_life>100){ tank_battery_life=100;}
  if (tank_battery_life<0){ tank_battery_life=0;}
  if (tank_battery_voltage<3){
    alert_flag=1;
    strcpy(alert_message,"ALERTA: BATERÍA BAJA EN TANQUE");
    Serial.println("ALERTA: BATERÍA BAJA EN TANQUE");
  }
  Serial.print("Tank Battery: "); Serial.print(tank_battery_voltage,4); Serial.print("\t"); Serial.println(tank_battery_life);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------
/*----------------------------LoRa Functions---------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------*/
void LoRaSend(String outgoing, byte destination) {
  delay(airtime);                   
  for(int i = 0; i < 4; i++){ // send same packet 4 times (1 second in total)
    LoRa.beginPacket();
    LoRa.setTxPower(txPower,RF_PACONFIG_PASELECT_PABOOST);
    // Packet header bytes:
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(outgoing.length());        // add payload length
    // add payload
    LoRa.print(outgoing);                 
    LoRa.endPacket();
    Serial.print("Sent"); Serial.println(outgoing);
    delay(250);
  }
  delay(airtime);                      
}

/*---------------------------------------------------------------------------------------------------*/
void onReceive(int packetSize, byte expected_sender){
  message_flag=0; 
  if (!packetSize) {
    Serial.println("No order");
    Heltec.display->drawString(0, 40, "No Data received"); Heltec.display->display();
    attempt++;
    if ((attempt > 4) && (expected_sender == meter_destination)){
      attempt=0;
      State="A2"; alert_flag=1;
      strcpy(alert_message,"ALERTA: ERROR EN COMUNICACION LoRa CON MACROMEDIDOR DE DEMANDA");
      Serial.println("ALERTA: ERROR EN COMUNICACION LoRa CON MACROMEDIDOR DE DEMANDA"); 
    }

    if ((attempt > 4) && (expected_sender == pump_destination)){
      attempt=0;
      State="A2"; alert_flag=1;
      strcpy(alert_message,"ALERTA: ERROR EN COMUNICACION LoRa CON PRODUCCION");
      Serial.println("ALERTA: ERROR EN COMUNICACION LoRa CON PRODUCCION"); 
    }
    return;                             // if there's no packet, return
  }

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingLength = LoRa.read();    // incoming msg length

  incoming ="";
  while (LoRa.available()){
    incoming += (char)LoRa.read();
  }
  
  if (incomingLength != incoming.length()){   // check length for error
      Heltec.display->drawString(0, 40,"error: length does not match"); Heltec.display->display();
      Serial.println("error: message length does not match length");
      return;                             // skip rest of function
    }
  
  // if the recipient isn't this device or sender isn't gateway,
  if ((recipient != localAddress) || (sender != expected_sender)) {
      Heltec.display->drawString(0, 40,"error: message not for me"); Heltec.display->display();
      Serial.println("Message not for me");
      return;                             // skip rest of function
    }
  if ((recipient == localAddress) && (sender == expected_sender) && (incomingLength == incoming.length())) {
    message_flag=1; attempt=0;
    Heltec.display->drawString(0, 40, "RSSI: " + String(LoRa.packetRssi())); Heltec.display->drawString(50, 40, incoming); Heltec.display->display();
    if (expected_sender == meter_destination){
      sprintf(meter_rssi, "%d", LoRa.packetRssi());
    }
    if (expected_sender == pump_destination){
      sprintf(pump_rssi, "%d", LoRa.packetRssi());
    }
    //Received Signal Strength Indicator (RSSI) - closer the value is to 0, the stronger the received signal has been. 
    // if message is for this device, or broadcast, print details:
    Serial.println("----------------------------------------");
    Serial.println("Received from: 0x" + String(sender, HEX));
    Serial.println("Sent to: 0x" + String(recipient, HEX));
    Serial.println("Message length: " + String(incomingLength));
    Serial.println("Message: " + incoming);
    Serial.println("RSSI: " + String(LoRa.packetRssi()));
    Serial.println("Snr: " + String(LoRa.packetSnr()));
    Serial.println("----------------------------------------");
  }
}
/*-------------------------------------------------------------------------------------------*/


/*----------------------------------AIO Subscriptions Check-----------------------------------------*/
void AIOstate(){
  Serial.println("Checking subscriptions in Adafruit IO... ");
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    // Check if its the onoff automatic system feed
    if (subscription == &automatic) {
      IoTsystem = (char *)automatic.lastread;
    }
    // check if its the pumps controlers feed
    if (subscription == &control15hp) {
      pump15hp_IO = (char *)control15hp.lastread;
    }
    if (subscription == &control10hp) {
      pump10hp_IO = (char *)control10hp.lastread;
    }
  }
  Serial.print("On-Off Auto: "); Serial.println(IoTsystem);
  Serial.print("On-Off 15Hp: "); Serial.println(pump15hp_IO);
  Serial.print("On-Off 10Hp: "); Serial.println(pump10hp_IO);
  //Serial.println(strlen((char *)automatic.lastread));
  if (strlen((char *)automatic.lastread) == 0) {
      Serial.println("ALERTA: Subcripción principal vacía");
      //Publishing process
      int8_t tries=0;
      while (!alerts.publish("ALERTA: Sistema reiniciado")){
        delay(1000);
        tries++;
        if (tries > 3){
          Serial.println("Important Publishing error");
        }
      }
    }  
  Serial.print("Alert flag: "); Serial.println(alert_flag);
  if (alert_flag){
    alert_flag=0;   
    if (mem_alert_message!=alert_message){
      mem_alert_message=alert_message;
      //Publish errors and alerts to AIO
      int8_t tries=0; 
      while (!alerts.publish(alert_message)){
        tries++;
        if (tries > 3){
          Heltec.display->clear(); Heltec.display->drawString(0, 25, "ALERTA! ERROR EN PUBLICACIÓN"); Heltec.display->display();
          Serial.println("Alert Publishing error"); delay(2000);
          break;
        }
        delay(1000);
      } 
    }else{
      Serial.println("Same Alert message");
    }
  } 
}
/*-------------------------------------------------------------------------------------------*/

/*---------------------------------WiFi & MQTT Conection.-------------------------------------*/
void WiFi_connect(){
  if (WiFi.status() == WL_CONNECTED){
    return;
  }
  Heltec.display->clear();
  Heltec.display->drawString(0, 20, "Connecting to ");  Heltec.display->drawString(0, 30, WLAN_SSID); Heltec.display->drawString(0, 50, "ST:"); Heltec.display->drawString(20, 50, State);
  Heltec.display->display();
  Serial.print("Connecting to ");  Serial.println(WLAN_SSID);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
    Serial.println();
  }
  Serial.println();
  Serial.println("WiFi connected");
  Heltec.display->drawString(0, 40,"WiFi connected");
  Heltec.display->display();
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
}
void MQTT_connect(){
  int8_t ret;
  if (mqtt.connected()){
    return;
  }
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Connecting to MQTT...");
  Heltec.display->drawString(0, 50, "ST:"); Heltec.display->drawString(20, 50, State);
  Heltec.display->display();
  Serial.print("Connecting to MQTT...");
  uint8_t retries = 5;
  while ((ret = mqtt.connect()) != 0){ // connect will return 0 for connected
    Heltec.display->drawString(0, 20, mqtt.connectErrorString(ret));Serial.println(mqtt.connectErrorString(ret));
    Heltec.display->drawString(0, 30, "Retrying MQTT connection"); Serial.println("Retrying MQTT connection in 5 seconds...");
    Heltec.display->display();
    mqtt.disconnect();
    delay(1000);
    retries--;
    if (retries == 0){
      Heltec.display->drawString(0, 40, "MQTT connection failed"); Serial.println("MQTT connection failed");
      Heltec.display->drawString(70, 40, "Please reboot"); Serial.println("Please reboot");
      Heltec.display->display();
      delay (2000);
      return;
    }    
  }
  Heltec.display->drawString(0, 40, "MQTT Connected!"); Serial.println("MQTT Connected!");
  Heltec.display->display();
}

/*--------------------------------------------------------------------------------------------*/
/*-------------------------Micro SD Card Data log functions-----------------------------------*/

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
      Serial.println("Failed to open file for reading");
      deleteFile(SD, "/SamaraData.csv");                
      //writeFile(SD, "/SamaraData.csv", csvfirstrow);
      writeFile(SD, "/SamaraData.csv", "TimeStamp;P_RSSI;M_RSSI;P15_Flw;P15_Vol[m3];P10_Flw;P10_Vol[m3];D_Flw;D_Vol[m3];RD_Flw;RD_Vol[m3];Tank[m3];D_Bat;P_Bat;T_Bat;MCT[C];PCT[C];TCT[C]\n");
      return; //if there is no file to read, create one
  }

  Serial.print("Read from file: ");
  while(file.available()){
      Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
      Serial.println("Failed to open file for writing");
      return;
  }
  if(file.print(message)){
      Serial.println("File written");
  } else {
      Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
      Serial.println("Failed to open file for appending");
      return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
      Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
      Serial.println("File renamed");
  } else {
      Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
      Serial.println("File deleted");
  } else {
      Serial.println("Delete failed");
  }
}
/*-------------------------------------------------------------------------------------------*/
