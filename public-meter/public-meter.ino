/**
 * @file meter.ino
 * @author Sergio Solórzano Alfaro
 * @coauthor Juan J. Rojas 
 * @date 15 May 2021
 * @brief Main loop for Arad Octave watermeter controller
 * @par Institution:
 * DeltaLab. Instituto Tecnológico de Costa Rica.
 * @par mail:
 * solorzanos117@gmail.com
 * juan.rojas@tec.ac.cr
 * @par Git repository:
 * https://github.com/ssolorzanocr/ASADA-PublicFiles
 */

/*SS: IMPORTANT: When using GPIO pins to receive PWM signals and sending by LoRa, I recommend using TOUCH pins 12 and 13.
*Any other may have internal hardware interference*/
 
#include "heltec.h"
#include "soc/rtc.h"
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <time.h>

#define BAND    915E6         //you can set band here directly,e.g. 868E6 (Europe),915E6 (North America),433E6 (Asia)

//Micro TF card pins to ESP32 GPIOs via this connecthin:
#define SD_CS   22        //  SD_CS   -- GPIO22
#define SD_SCK  2         //  SD_SCK  -- GPIO2
#define SD_MOSI 23        //  SD_MOSI -- GPIO23
#define SD_MISO 17        //  SD_MISO -- GPIO17

#define DIGITAL_PULSE_1 13  //for hidric measures  
#define DIGITAL_PULSE_2 12  //for hidric measures
#define ADC_BATTERY     38  //battery voltage input
#define ADC_INPUT       39  //foreseen input         

#define RP      0.01          //IMPORTANT: Rp stands for "Pulse Resolution". Depends on flow meter parameters. 1 pulse = Rp cubic meters of water that have passed.
#define UNITS   1000000000.0  //calculate flow: (1000000000*Rp/(sum_interval/pulses))[m3/s]; (3600000000*Rp/(sum_interval/pulses))[m3/h]; (1000000000*Rp/(sum_interval/pulses))[L/s]

// The sd card can also use a virtual SPI bus
SPIClass sd_spi(HSPI);
long timezone = 1;
byte daysavetime = 1;
 
char buff[99] = {'\0'}; char value[12]; const char semicolon[] = ";"; char t_stamp[10] = {'\0'}; const char newline[] = "\n"; char tank_rssi[10] = {'\0'};

TaskHandle_t measureTask;
TaskHandle_t loopTask;

int8_t txPower = 20;       /*txPower -- 0 ~ 20 dBm*/
/** LoRa.setTxPower(txPower,RFOUT_pin);
  * txPower -- 0 ~ 20
  * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
  *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
  *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */
  
/*----Local variables----*/            
byte localAddress;       // add your address of this device
byte destination;       // add the destination to send to (Gateway)
String incoming;

double demand_vol=0; double demand_sent; double demand_flow, demand_av; double sumdemand_flow=0; 
double battery_voltage, battery_life; 
char mpack[30] = {'\0'};
char *s; bool mark=0; bool chk; char delimiter [2] = "e";
uint32_t interval, lastinterval, sum_interval, init_stamp;
bool meter_flag, sd_flag, write_flag, first_flag, message_flag, resetdata_flag = 0; bool firstflow_flag;

/*-- create a hardware timer --*/
//hw_timer_t * timer = NULL;
//bool timer_flag;
/*-----------------------------*/

void IRAM_ATTR reed_isr() {
  meter_flag = 1;
}

//void IRAM_ATTR timer_isr(){
//  timer_flag = 1;
//}

void setup() {
  Serial.begin(115200);
  delay(10);
  noInterrupts();      //Disable interrupts while initiating 
  pinMode(DIGITAL_PULSE_1, INPUT_PULLUP);
  pinMode(ADC_BATTERY,INPUT); 
  attachInterrupt(DIGITAL_PULSE_1,&reed_isr,FALLING);      

  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  //timer = timerBegin(0, 80, true);
  /* Attach timer_isr function to our timer */
  //timerAttachInterrupt(timer, &timer_isr, true);
  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  //timerAlarmWrite(timer, 5000000, true);    //5 seconds timer
  /* Start an alarm */
  //timerAlarmEnable(timer);
  
  Serial.print("setup() running on core "); Serial.println(xPortGetCoreID());

  //create a task that will be executed in the measureTaskcode() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    measureTaskcode,    /* Task function. */
                    "measure",          /* name of task. */
                    10000,              /* Stack size of task */
                    NULL,               /* parameter of the task */
                    1,                  /* priority of the task */
                    &measureTask,       /* Task handle to keep track of created task */
                    1);                 /* pin task to core */                  
  delay(200);
  
  //create a task that will be executed in the loopTaskcode() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                      loopTaskcode,       /* Task function. */
                      "loop",             /* name of task. */
                      10000,              /* Stack size of task */
                      NULL,               /* parameter of the task */
                      1,                  /* priority of the task */
                      &loopTask,          /* Task handle to keep track of created task */
                      0);                 /* pin task to core */                  
  delay(200);

  // SD Card demandData.csv file initiation
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
  
  //-----For new SD Cart data use only----------------------
  //deleteFile(SD, "/demandData.csv"); sd_flag=0; //for deleting file 
  //resetdata_flag=1;             //This flag deletes old file to create a new one and start from scratch
  readFile(SD, "/demandData.csv");
  
  delay(500);
}

void measureTaskcode( void * pvParameters ){
  delay(6000);        //waits for everything else to start
  Serial.print("measureTask() running on core "); Serial.println(xPortGetCoreID());
  interrupts();       //Enable interrupts
  Serial.println("time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage"); 
  
  for(;;){
    //After 71.6 minutes (2^32 micro seconds), the value you get from micros() will overflow (reset to zero).
    //Acording to Octave –After 1 minute of no flow, the meter will flash a series of letters and numbers in place of the rate of flow to show the checksum of the software version. 
    if(first_flag){                         //Check only when measuring
      if ((micros()-init_stamp)>120000000){  //Stop and reset measure process after 2 minutes of no PULSES. **Min flow posibly measured = RP*1000/120 [L/s]
        first_flag=0; 
        demand_flow = 0;
        sum_interval+=(micros()-init_stamp);    //Add last large time interval for average calc
         
        if(firstflow_flag){
          demand_av = UNITS*(demand_vol-RP)/sum_interval;   //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          demand_av = UNITS*(demand_vol)/sum_interval;
        }
        
        Serial.println("Stop");                 //Measuring process stops in order to save memory
        
        //printing>> 
        //time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage
        Serial.print(",");Serial.print(lastinterval);Serial.print(","); Serial.print (demand_vol,3); Serial.print(","); 
        Serial.print (demand_flow,4); Serial.print(","); Serial.print(demand_av,4); Serial.print(","); Serial.println(battery_voltage,4);
      }
    }
    if(meter_flag && !first_flag){  //Start measure process 
      first_flag=1; meter_flag=0; firstflow_flag=1; Serial.println("Start");
      init_stamp=micros();          //First time stamp
      demand_vol+= RP;                     //First volume measure
    }
    
    if(meter_flag && first_flag ){
      interval=abs(micros()-init_stamp);  //Time interval between pulses
      if (interval<110000){       //IGNORE SMALL INTERFERENCES, THAT CAUSES HUGE FLOW MEASURES < 0.3 sec 
        //IMPORTANT: THIS PARAMETER MUST BE A LITTLE BIT LARGER THAN PULSE WIDTH ---> SMALL PULSE WIDTHS HELPS AT MEASURING
        //pulse width measured for 4in Octave is 450ms and for 2in Octave is 300ms
        //pulse width programed for 4in Octave is 90ms and for 2in Octave is 280ms
        meter_flag=0;
      }else{
        init_stamp=micros();        //Time stamp for next measure
        sum_interval+=interval;   //For average flow calc
        
        demand_flow = (UNITS*RP)/((double)interval); 
        
        demand_vol+= RP;                                 //plus Rp cubic meters per interruption
        if(firstflow_flag){
          demand_av = UNITS*(demand_vol-RP)/sum_interval;   //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          demand_av = UNITS*(demand_vol)/sum_interval;
        }
        meter_flag=0; lastinterval= interval;
        if (first_flag){ write_flag=1; } // write flag gives the order to write measures in sd card when measuring
        
        //printing>> time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage 
        Serial.print(",");Serial.print(lastinterval);Serial.print(","); Serial.print (demand_vol,3); Serial.print(","); 
        Serial.print (demand_flow,4); Serial.print(","); Serial.print(demand_av,4); Serial.print(","); Serial.println(battery_voltage,4);
      } 
    }
  }
}

void loopTaskcode( void * pvParameters ){
  delay(4000); 

  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->flipScreenVertically();  
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->drawString(0, 20, "Beginning flow");
  Heltec.display->drawString(0, 30, "measurement process!"); Heltec.display->display();
  Serial.print("loop() running on core "); Serial.println(xPortGetCoreID());
  write_flag=0; //just in case don't write at start

  for (;;){
    delay(1000);          /*procesor needs a second to rest, other wise will past out*/
    if (listencommand()){ /*-------------------------------------------MEASURE SENDING STATE */
      sprintf(mpack, "%.2f%s%.2f%s%.2f", demand_vol, delimiter, demand_av, delimiter, battery_voltage);
      demand_sent=demand_vol;
      Serial.print("Packet to send: "); Serial.println(mpack);
      LoRaSend(mpack, destination);                  
      mark = 0;
    }
    if ((sd_flag) && (write_flag)){ //when a SD Card is attached and measure process is been done
      write_flag=0;
      // Now save values on SD Card Data file
      strcat(buff,tank_rssi);
      strcat(buff,semicolon);
      dtostrf(demand_flow,5,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(demand_av,5,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(lastinterval,10,0,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(demand_vol,10,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      dtostrf(battery_voltage,4,2,value);
      strcat(buff,value);
      strcat(buff,semicolon);
      strcat(buff,newline);
      //Serial.print("buff: ");Serial.println(buff);
      appendFile(SD, "/demandData.csv", buff);
      memset(buff,0,sizeof(buff));
    }       
  }
}

void loop(){
  delay(1000);          /*core 1 needs one second to rest, other wise will past out*/
}
/*--------------------------------------------------------------------------------------------*/

void batteryVoltage(){
  /*full battery voltage = 4.17V, which corresponds to a reading of 2.085V
   *empty battery voltage = 2.80V, which corresponds to a reading of 1.4V
   */ 
  battery_voltage=2*((analogRead(ADC_BATTERY)+200.01)/1263.4);  // Aproximate ADC vs DC voltage function from Heltec LoRa 32 V2 real measures
  battery_life=100*((battery_voltage-2.8)/1.37);                  //battery life percentage 
  if (battery_life>100){ battery_life=100;}
  if (battery_life<0){ battery_life=0;}
  //Serial.print("\tBattery: "); Serial.print(battery_voltage,4); Serial.print("\t"); Serial.println(battery_life);
}

/*-------------------------------------------------------------------------------------------*/
bool listencommand(){ /*LISTENING STATE*/
  batteryVoltage();
  Heltec.display->clear();
  Heltec.display->drawString(0, 0,"Water flow: "); Heltec.display->drawString(60, 0, String(demand_flow)); Heltec.display->drawString(100, 0, "L/s");
  Heltec.display->drawString(0, 10,"Average: "); Heltec.display->drawString(60, 10, String(demand_av)); Heltec.display->drawString(100, 10, "L/s");
  Heltec.display->drawString(0, 20,"Volume: ");  Heltec.display->drawString(50, 20, String(demand_vol)); Heltec.display->drawString(110, 20, "m3");
  Heltec.display->drawString(0, 30, "Bat:"); Heltec.display->drawString(25, 30, String(battery_life,1)+"%"); Heltec.display->drawString(70, 30, "SD:"); if (sd_flag){Heltec.display->drawString(90, 30, "OK");}
  Heltec.display->display();
  //Serial.println("Listening...");

  LoRa.receive();
  onReceive(LoRa.parsePacket());  
  if (message_flag){
    message_flag=0;
    if (incoming=="D"){
      mark=1;        
    }
    Serial.println("demand_vol: ");Serial.println(demand_vol);
    Serial.println("demand_sent: ");Serial.println(demand_sent);
    if (incoming=="OKD"){ //reset average flow calc and acumulated volume
      sum_interval=0; demand_av=0; firstflow_flag=0;
      demand_vol -= demand_sent; 
      demand_sent=0;
      delay(2000);  //wait for repeated messages to pass
      Serial.println("Demand flow---------------------------------------");
      Serial.println(demand_av);   
    }
  }
  return mark;
}

/*----------------------------LoRa Functions----------------------------------------------*/
void LoRaSend(String outgoing, byte destination) {
  Heltec.display->drawString(0, 50, "Sending: ");
  Heltec.display->drawString(50, 50, outgoing);
  Heltec.display->display();
  delay(1000);                   
  for(int i = 0; i < 5; i++){ // send same packet 5 times (2 seconds in total)
    LoRa.beginPacket();
    LoRa.setTxPower(txPower,RF_PACONFIG_PASELECT_PABOOST);
    // Packet header bytes:
    LoRa.write(destination);              // add destination address
    LoRa.write(localAddress);             // add sender address
    LoRa.write(outgoing.length());        // add payload length
    // add payload
    LoRa.print(outgoing);                 
    LoRa.endPacket();
    Serial.print("Sent  "); Serial.println(outgoing);
    delay(400);
  }                      
}

void onReceive(int packetSize){
  message_flag=0;
  if (!packetSize) {
    //Serial.println("No order"); 
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
  if ((recipient != localAddress) && (sender != 0xAA)) {
    Heltec.display->drawString(0, 40,"error: message not for me"); Heltec.display->display();
    Serial.println("Message not for me");
    return;                             // skip rest of function
  }
  if ((recipient == localAddress) && (sender == 0xAA) && (incomingLength == incoming.length())) {
    message_flag=1; 
    Heltec.display->drawString(0, 40, "Got: "); Heltec.display->drawString(20, 40, incoming);
    Heltec.display->drawString(70, 40, "RSSI: " + String(LoRa.packetRssi())); Heltec.display->display();
    sprintf(tank_rssi, "%d", LoRa.packetRssi());
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

/*------------------------Micro SD Card Data log functions-----------------------------------*/

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);
  if (resetdata_flag){
    deleteFile(SD, "/demandData.csv");
  }
  File file = fs.open(path);
  if(!file){
      Serial.println("Failed to open file for reading");
      writeFile(SD, "/demandData.csv", "tank_rssi;Flw;Flw_Prom;Last_Interval;Vol;VBat\n");   //if there is no file to read, create one
      return;
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
/*--END--*/
