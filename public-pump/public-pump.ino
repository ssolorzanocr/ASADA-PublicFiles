/**
 * @file pump.ino
 * @author Sergio Solórzano Alfaro
 * @author Juan J. Rojas 
 * @date 15 May 2021
 * @brief Main loop for pump controller and 2 water meter data extraction
 * @par Institution:
 * DeltaLab. Instituto Tecnológico de Costa Rica.
 * @par mail:
 * solorzanos117@gmail.com
 * juan.rojas@tec.ac.cr
 * @par Git repository:
 * https://github.com/ssolorzanocr/ASADA-PublicFiles
 */

/*SS: IMPORTANT: When using GPIO pins to receive PWM signals and sending by LoRa, I recommend using pins 12 and 13. 
*Any other may have internal hardware interference*/

#include "heltec.h"
#include "soc/rtc.h"
#include "FS.h"
#include "SD.h"
#include <SPI.h>
#include <time.h>

#define BAND    915E6   //you can set band here directly,e.g. 868E6 (Europe),915E6 (North America),433E6 (Asia)

//Micro TF card pins to ESP32 GPIOs via this connecthin:
#define SD_CS   22        //  SD_CS   -- GPIO22
#define SD_SCK  2         //  SD_SCK  -- GPIO2
#define SD_MOSI 23        //  SD_MOSI -- GPIO23
#define SD_MISO 17        //  SD_MISO -- GPIO17

#define DIGITAL_PULSE_1 13  //for hidric measures  
#define DIGITAL_PULSE_2 12  //for hidric measures
#define ADC_BATTERY     38  //battery voltage input
#define ADC_INPUT       39  //foreseen input

#define DIGITAL_INPUT_1 36  //for 15hp pump contactor relay state
#define DIGITAL_INPUT_2 37  //for 10hp pump contactor relay state 

#define DIGITAL_SSROUT_1 32  //to activate 15hp pump contactor relay 
#define DIGITAL_SSROUT_2 33  //to activate 10hp pump contactor relay 

#define RP1     0.01          //IMPORTANT: Rp stands for "Pulse Resolution". Depends on flow meter parameters. 1 pulse = Rp cubic meters of water that have passed.
#define RP2     0.01
#define UNITS   1000000000.0  //calculate flow: (1000000000*Rp/(sum_interval/pulses))[m3/s]; (3600000000*Rp/(sum_interval/pulses))[m3/h]; (1000000000*Rp/(sum_interval/pulses))[L/s]

// The sd card can also use a virtual SPI bus
SPIClass sd_spi(HSPI);
long timezone = 1;
byte daysavetime = 1;

char t_stamp[20]; char buff[70] = {'\0'}; char value[20]; char semicolon[] = ";"; char newline[] = "\n"; char tank_rssi[10] = {'\0'};


TaskHandle_t measureTask;
TaskHandle_t loopTask;

/*----Local variables----*/    
byte localAddress;    // address of this device
byte destination;     // destination to send to (Gateway)
String incoming;

char *strtok(char *str1, const char *str2); char *part; char charpacket [80]; char delimiter [2] = "e";
String packSize = "--"; String sdelimiter="e";
char *s; int8_t packflag; 
char mpack[] = {'\0'};
char ppack[8] = {'\0'};

String pump15hp_status, pump1 = "OFF";
String pump10hp_status, pump2 = "OFF";
double battery_voltage, battery_life;
double pump15hp_flow, pump15hp_vol, pump15hp_vol_sent, pump10hp_flow, pump10hp_vol, pump10hp_vol_sent, av15hp_flow, av10hp_flow;

uint32_t t1_interval, t2_interval, sum1_interval, init_1stamp, sum2_interval, init_2stamp, last1interval, last2interval, lastorder;
bool first_1flag = 0; bool first_2flag = 0; bool firstflow1_flag, firstflow2_flag;
bool meter_1flag, meter_2flag;
/* create a hardware timer */
hw_timer_t * timer = NULL;
bool timer_flag, sd_flag; bool message_flag = 0;


int8_t txPower = 20;       /*txPower -- 0 ~ 20 dBm*/
/** LoRa.setTxPower(txPower,RFOUT_pin);
  * txPower -- 0 ~ 20
  * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
  *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
  *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
  */ 

void IRAM_ATTR pulsecountA(){ 
  meter_1flag = 1;
}
void IRAM_ATTR pulsecountB(){ 
  meter_2flag = 1;
}

void IRAM_ATTR timer_isr(){
  timer_flag = 1;
}

void setup() {
  Serial.begin(115200);
  delay(10);
  noInterrupts();     //Disable interrupts while initiating 
  pinMode(DIGITAL_PULSE_1, INPUT_PULLUP); pinMode(DIGITAL_PULSE_2, INPUT_PULLUP); 
  pinMode(DIGITAL_SSROUT_1,OUTPUT); pinMode(DIGITAL_SSROUT_2,OUTPUT);
  pinMode(DIGITAL_INPUT_1,INPUT); pinMode(DIGITAL_INPUT_2,INPUT);
  attachInterrupt(DIGITAL_PULSE_1,&pulsecountA,RISING);      //(Interrupción (GPIO13),función,Flanco de subida)
  attachInterrupt(DIGITAL_PULSE_2,&pulsecountB,RISING);
  
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach timer_isr function to our timer */
  timerAttachInterrupt(timer, &timer_isr, true);
  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 5000000, true);
  /* Start an alarm */
  timerAlarmEnable(timer);

  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);
  Heltec.display->init();
  Heltec.display->setTextAlignment(TEXT_ALIGN_LEFT);
  Heltec.display->setFont(ArialMT_Plain_10); Heltec.display->flipScreenVertically();  
  Heltec.display->clear();
  Heltec.display->drawString(0, 0, "Heltec.LoRa Initial success!");
  Heltec.display->drawString(0, 20, "Beginning PUMP program!");
  Heltec.display->display();
  Serial.print("setup running on core "); Serial.println(xPortGetCoreID());
  
  //create a task that will be executed in the measureTaskcode() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    measureTaskcode,  /* Task function. */
                    "measure",        /* name of task. */
                    10000,            /* Stack size of task */
                    NULL,             /* parameter of the task */
                    1,                /* priority of the task */
                    &measureTask,     /* Task handle to keep track of created task */
                    1);               /* pin task to core */                  
  delay(200);

  //create a task that will be executed in the loopTaskcode() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    loopTaskcode,   /* Task function. */
                    "loop",         /* name of task. */
                    10000,          /* Stack size of task */
                    NULL,           /* parameter of the task */
                    1,              /* priority of the task */
                    &loopTask,      /* Task handle to keep track of created task */
                    0);             /* pin task to core */                  
  delay(200);

  // SD Card productionData.csv file initiation
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
  //deleteFile(SD, "/productionData.csv");
  //writeFile(SD, "/productionData.csv", "time;Flw1;Vol1;Flw2;Vol2;VBat\n");
  
  readFile(SD, "/productionData.csv");
  delay(500);
}


void measureTaskcode( void * pvParameters ){
  delay(6000);        //waits for everything else to start
  Serial.print("measureTask() running on core "); Serial.println(xPortGetCoreID());
  interrupts();       //Enable interrupts
  
  for(;;){            //measure loop for 2 simultaneous Octave water meters
    //After 71.6 minutes (2^32 micro seconds), the value you get from micros() will overflow (reset to zero).
    //Acording to Octave –After 1 minute of no flow, the meter will flash a series of letters and numbers in place of the rate of flow to show the checksum of the software version. 
    
    if(first_1flag){                         //Check only when measuring
      if ((micros()-init_1stamp)>120000000){  //Stop and reset measure process after 2 minutes of no PULSES. **Min flow posibly measured = RP*1000/120 [L/s]
        first_1flag=0; 
        pump15hp_flow = 0;
        sum1_interval+=(micros()-init_1stamp);    //Add last large time interval for average calc 
        if (firstflow1_flag){
          av15hp_flow = UNITS*(pump15hp_vol-RP1)/(double)sum1_interval; //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          av15hp_flow = UNITS*(pump15hp_vol)/(double)sum1_interval;
        }
        Serial.println("Stop 15hp");              //Measuring process stops in order to save memory

        //printing >>>
        //time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage
        Serial.print(",");Serial.print(last1interval);Serial.print(","); Serial.print (pump15hp_vol,3); Serial.print(","); 
        Serial.print (pump15hp_flow,4); Serial.print(","); Serial.print(av15hp_flow,4); Serial.print(", , , , ,"); 
        Serial.println(battery_voltage,4);
        Serial.println(sum1_interval);
      }
    }
    if(first_2flag){                         //Check only when measuring
      if ((micros()-init_2stamp)>120000000){  //Stop and reset measure process after 2 minutes of no PULSES. **Min flow posibly measured = RP*1000/120 [L/s]
        first_2flag=0; 
        pump10hp_flow = 0;
        sum2_interval+=(micros()-init_2stamp);    //Add last large time interval for average calc 
        if (firstflow2_flag){
          av10hp_flow = UNITS*(pump10hp_vol-RP2)/(double)sum2_interval; //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          av10hp_flow = UNITS*(pump10hp_vol)/(double)sum2_interval;
        }
        Serial.println("Stop 10hp");              //Measuring process stops in order to save memory

        //printing >>>
        //time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage
        Serial.print(", , , , ,"); 
        Serial.print(last2interval);Serial.print(","); Serial.print (pump10hp_vol,3); Serial.print(","); 
        Serial.print (pump10hp_flow,4); Serial.print(","); Serial.print(av10hp_flow,4); Serial.print(",");
        Serial.println(battery_voltage,4);
        Serial.println(sum2_interval);
      }
    }
    
    //Start measures
    if(meter_1flag && !first_1flag){  //Start 15hp measure process 
      first_1flag=1; meter_1flag=0; firstflow1_flag=1; Serial.println("Start 15hp");
      init_1stamp=micros();          //First time stamp
      pump15hp_vol+= RP1;                     //First volume measure
    }
    if(meter_2flag && !first_2flag){  //Start 10hp measure process 
      first_2flag=1; meter_2flag=0; firstflow2_flag=1; Serial.println("Start 10hp");
      init_2stamp=micros();          //First time stamp
      pump10hp_vol+= RP2;                     //First volume measure
    }
    
    //Measuring volume and flow process for 15hp pump
    if(meter_1flag && first_1flag ){
      t1_interval=abs(micros()-init_1stamp);  //Time interval between pulses
      if (t1_interval<110000){       //IGNORE SMALL INTERFERENCES, THAT CAUSES HUGE FLOW MEASURES < 0.3 sec 
        //IMPORTANT: THIS PARAMETER MUST BE A LITTLE BIT LARGER THAN PULSE WIDTH ---> SMALL PULSE WIDTHS HELPS AT MEASURING
        //pulse width measured for 4in Octave is 450ms and for 2in Octave is 300ms
        //pulse width programed for 4in Octave is 90ms and for 2in Octave is 280ms
        meter_1flag=0;
      }else{
        init_1stamp=micros();        //Time stamp for next measure
        sum1_interval+=t1_interval;   //For average flow calc
        
        pump15hp_flow = (UNITS*RP1)/((double)t1_interval); 
        
        pump15hp_vol+= RP1;                                 //plus Rp cubic meters per interruption

        if (firstflow1_flag){
          av15hp_flow = UNITS*(pump15hp_vol-RP1)/(double)sum1_interval; //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          av15hp_flow = UNITS*(pump15hp_vol)/(double)sum1_interval;
        }
        meter_1flag=0; last1interval= t1_interval;
        
        //printing >>>
        //time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage
        Serial.print(",");Serial.print(last1interval);Serial.print(","); Serial.print (pump15hp_vol,3); Serial.print(","); 
        Serial.print (pump15hp_flow,4); Serial.print(","); Serial.print(av15hp_flow,4); Serial.print(", , , , ,"); 
        Serial.println(battery_voltage,4);
        Serial.println(sum1_interval);
      } 
    }

    //Measuring volume and flow process for 10hp pump
    if(meter_2flag && first_2flag ){
      t2_interval=abs(micros()-init_2stamp);  //Time interval between pulses
      if (t2_interval<110000){       //IGNORE SMALL INTERFERENCES, THAT CAUSES HUGE FLOW MEASURES < 0.3 sec 
        //IMPORTANT: THIS PARAMETER MUST BE A LITTLE BIT LARGER THAN PULSE WIDTH ---> SMALL PULSE WIDTHS HELPS AT MEASURING
        //pulse width measured for 4in Octave is 450ms and for 2in Octave is 300ms
        //pulse width programed for 4in Octave is 90ms and for 2in Octave is 280ms
        meter_2flag=0;
      }else{
        init_2stamp=micros();        //Time stamp for next measure
        sum2_interval+=t2_interval;   //For average flow calc
        
        pump10hp_flow = (UNITS*RP2)/((double)t2_interval); 
        
        pump10hp_vol+= RP2;                                 //plus Rp cubic meters per interruption
        if (firstflow2_flag){
          av10hp_flow = UNITS*(pump10hp_vol-RP2)/(double)sum2_interval; //minus Rp cause first pulse measure does not count for first flow calc
        }else{
          av10hp_flow = UNITS*(pump10hp_vol)/(double)sum2_interval;
        }
        meter_2flag=0; last2interval= t2_interval;
        
        //printing >>>
        //time,interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),interval between pulses (us),vol(m3),flow(L/s),promflow(L/s),battery voltage
        Serial.print(", , , , ,"); 
        Serial.print(last2interval);Serial.print(","); Serial.print (pump10hp_vol,3); Serial.print(","); 
        Serial.print (pump10hp_flow,4); Serial.print(","); Serial.print(av10hp_flow,4); Serial.print(",");
        Serial.println(battery_voltage,4);
        Serial.println(sum1_interval);
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
  Serial.print("loopTask() running on core "); Serial.println(xPortGetCoreID());
  
  for(;;){
    delay(1000);          /*procesor needs half second to rest, other wise will past out*/
    Heltec.display->clear();
    Heltec.display->drawString(0, 0, "M1:"); Heltec.display->drawString(20, 0, String((pump15hp_flow),2)); Heltec.display->drawString(50, 0,"L/s"); Heltec.display->drawString(80, 0, String((pump15hp_vol),2));Heltec.display->drawString(115, 0,"m3");
    Heltec.display->drawString(0, 10, "M2:"); Heltec.display->drawString(20, 10, String((pump10hp_flow),2)); Heltec.display->drawString(50, 10,"L/s"); Heltec.display->drawString(80, 10, String((pump10hp_vol),2));Heltec.display->drawString(115, 10,"m3");
    Heltec.display->drawString(0, 20, "Bat:"); Heltec.display->drawString(25, 20, String(battery_life,1)+"%"); Heltec.display->drawString(70, 20, "SD:"); if (sd_flag){Heltec.display->drawString(90, 20, "OK");}
    Heltec.display->drawString(0, 50, "15HP"); Heltec.display->drawString(30, 50, pump15hp_status); Heltec.display->drawString(60, 50, "10HP"); Heltec.display->drawString(90, 50, pump10hp_status); 
    Heltec.display->display();
    incoming=""; packflag=0;
    pumpcheck();
    batteryVoltage();
    listencommand();
    //Serial.print("Pack flag: "); Serial.println(packflag);
    
    if (packflag == 1){ 
      /*--------------------------------CALC AND MEASURE SENDING STATE */
      Serial.print("av15hp_flow: "); Serial.println(av15hp_flow);
      Serial.print("pump15hp_vol: "); Serial.println(pump15hp_vol);
      Serial.print("av10hp_flow: "); Serial.println(av10hp_flow);
      Serial.print("pump10hp_vol: "); Serial.println(pump10hp_vol);
      sprintf(mpack, "%.2f%s%.2f%s%.2f%s%.2f%s%.2f", av15hp_flow, delimiter, pump15hp_vol, delimiter, av10hp_flow, delimiter, pump10hp_vol, delimiter, battery_voltage);
      pump15hp_vol_sent = pump15hp_vol; //to take count how much volume has been reported to tank master
      pump10hp_vol_sent = pump10hp_vol;
      Serial.print("Packet to send: "); Serial.println(mpack);
      LoRaSend(mpack, destination);                        //aprox 10 bites

      if (sd_flag){ //when a SD Card is attached
        // Now save values on SD Card Data file
        strcat(buff,tank_rssi);
        strcat(buff,semicolon);
        dtostrf(av15hp_flow,5,2,value);
        strcat(buff,value);
        strcat(buff,semicolon);
        dtostrf(pump15hp_vol,10,2,value);
        strcat(buff,value);
        strcat(buff,semicolon);
        dtostrf(av10hp_flow,5,2,value);
        strcat(buff,value);
        strcat(buff,semicolon);
        dtostrf(pump10hp_vol,10,2,value);
        strcat(buff,value);
        strcat(buff,semicolon);
        dtostrf(battery_voltage,4,2,value);
        strcat(buff,value);
        strcat(buff,semicolon);
        strcat(buff,newline);
        appendFile(SD, "/productionData.csv", buff);
        memset(buff,0,sizeof(buff));
      }  
    }
    if (packflag == 2){ 
      /*--------------------------------PUMP STATUS SENDING STATE */
      lastorder = millis();
      if (incoming!="SP"){
        int8_t p = 1;
        incoming.toCharArray(charpacket,80);
        part = strtok(charpacket, delimiter);
        while (part!=NULL) {    //message separation process
          if (p==1){
            pump1 = part;
          }
          if (p==2){
            pump2 = part;
          }
          p++;
          part=strtok(NULL, delimiter);
        }
        //Serial.println(pump1); Serial.println(pump2);
        
        if (pump1=="ON"){
          digitalWrite(DIGITAL_SSROUT_1, HIGH); Serial.println("PUMP 1: ON");
        }
        if (pump1=="OFF"){
          digitalWrite(DIGITAL_SSROUT_1, LOW);
        }
        if (pump2=="ON"){
          digitalWrite(DIGITAL_SSROUT_2, HIGH);
        }
        if (pump2=="OFF"){
          digitalWrite(DIGITAL_SSROUT_2, LOW);
        }
      }
      delay(2000); //wait for pump relay contactors 
      pumpcheck();  
      sprintf(ppack, "%s%s%s", pump15hp_status, delimiter, pump10hp_status);
      Serial.print("Packet to send: "); Serial.println(ppack);
      LoRaSend(ppack, destination);                        //aprox 10 bites     
    }
  }                      
}

void loop () {
  delay(1000);
}
/*--------------------------------------------------------------------------------------------*/

/*-------------------------------------LoRa functions----------------------------------------*/

void LoRaSend(String outgoing, byte destination) {
  Heltec.display->drawString(0, 30, outgoing);
  Heltec.display->display();
  delay(1000);                   
  for(int i = 0; i < 4; i++){ // send same packet 4 times (2 seconds in total)
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
    delay(500);
  }
  delay(500); //rest half second                 
}

void listencommand(){ /*LISTENING STATE*/
  packflag = 0;
  
  LoRa.receive();
  onReceive(LoRa.parsePacket());
  
  //Serial.print("incoming  "); Serial.println(incoming);
 
  if (message_flag){
    message_flag=0; 
    Heltec.display->drawString(0, 40, incoming); Heltec.display->display();
    //Serial.print("Incoming length: "); Serial.println(incoming.length());
    if (incoming.length()<=3){
      if (incoming=="P"){
        packflag = 1;        
      }
      if (incoming=="SP"){
        packflag = 2;        
      }
      if (incoming=="OKP"){ //reset average flow calc and acumulated volume
        sum1_interval=0; sum2_interval=0; av15hp_flow=0; av10hp_flow=0;
        pump15hp_vol-=pump15hp_vol_sent; pump10hp_vol-=pump10hp_vol_sent;  
        delay(2000);  //wait for repeated messages to pass
        pump15hp_vol_sent=0; pump10hp_vol_sent=0; firstflow1_flag=0; firstflow2_flag=0;      
      }
    }else{
      packflag = 2;
    }
  }
  //Serial.print("packflag  "); Serial.println(packflag);
}

void onReceive(int packetSize){
  if (!packetSize) {
    //Serial.println("No order");
    if ((millis()-lastorder)>600000){
      //10 minutes since no order from gateway master -> turn off all pumps
      if ((pump1!="OFF")||(pump2!="OFF")){
        digitalWrite(DIGITAL_SSROUT_1, LOW);
        digitalWrite(DIGITAL_SSROUT_2, LOW);
        pump1= "OFF"; pump2 = "OFF";
      }
      Heltec.display->drawString(0, 40,"error: No order in 10min"); Heltec.display->display();
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
    }
  
  // if the recipient isn't this device or sender isn't gateway,
  if (recipient != localAddress && sender != destination) {
      Heltec.display->drawString(0, 40,"error: message not for me"); Heltec.display->display();
      Serial.println("Message not for me");
    }
  if (recipient == localAddress && sender == destination && incomingLength == incoming.length()) {
    message_flag=1; 
    Heltec.display->drawString(60, 40, "RSSI: " + String(LoRa.packetRssi())); Heltec.display->display();
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

void pumpcheck(){
  if (digitalRead(DIGITAL_INPUT_1)){        //checks main pump contactor state
    pump15hp_status = "ON";
  }else{
    pump15hp_status = "OFF";
  }
  if (digitalRead(DIGITAL_INPUT_2)){        //checks secondary pump contactor state
    pump10hp_status = "ON";
  }else{
    pump10hp_status = "OFF";
  }
  if (pump15hp_status != pump1 || pump10hp_status != pump2){
    Serial.println("ERROR en BOMBAS o modo manual activo");
    Heltec.display->drawString(0, 40, "ALERTA! MODO MANUAL");
    Heltec.display->display();
  }
}

void batteryVoltage(){
  /*full battery voltage = 4.17V, which corresponds to a reading of 2.085V
   *empty battery voltage = 2.90V, which corresponds to a reading of 1.4V
   */ 
  battery_voltage=2*((analogRead(ADC_BATTERY)+200.01)/1263.4); // Aproximate ADC vs DC voltage function from Heltec LoRa 32 V2 real measures   
  battery_life=100*((battery_voltage-2.9)/1.27); //percentage 
  if (battery_life>100){ battery_life=100;}
  if (battery_life<0){ battery_life=0;}
  //Serial.print("\tBattery: "); Serial.print(battery_voltage,4); Serial.print("\t"); Serial.println(battery_life);
}

/*--------------------------------------------------------------------------------------------*/

/*-------------------------Micro SD Card Data log functions-----------------------------------*/

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
      Serial.println("Failed to open file for reading");
      deleteFile(SD, "/productionData.csv");
      writeFile(SD, "/productionData.csv", "time;Flw1;Vol1;Flw2;Vol2;VBat\n");  //if there is no file to read, create one
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
/*-------------------------------------------------------------------------------------------*/
