#include <cc2500_REG.h>
#include <cc2500_VAL.h>

//#include "C:\Users\Owner\Documents\GitHub\WSN-Teensy\cc2500\SPIN-2\Node\packet.h"
#include <packet.h>

#include <time.h> 
#include <SPI.h>

#define CC2500_IDLE    0x36      // Exit RX / TX, turn
#define CC2500_TX      0x35      // Enable TX. If in RX state, only enable TX if CCA passes
#define CC2500_RX      0x34      // Enable RX. Perform calibration if enabled
#define CC2500_FTX     0x3B      // Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states
#define CC2500_FRX     0x3A      // Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states
#define CC2500_TXFIFO  0x3F
#define CC2500_RXFIFO  0x3F

#define ADV 0x00                   //Mode calls - might or might not be useful for the full Node.m
#define REQ 0x01
#define DAT 0x02
#define ACK 0x03

#define Size_of_Meta   3      // Standard meta-data packet length 
#define Size_of_Full   4 

unsigned char data_memory[100];  // Node's memory of data passed through
int pktlen             = 4;     // Max Packet Length 

// Known structure of Packets:
// [Length] [LN-ID of Origin Node] [MODE] [DATA]

//------------------------NODE NAME--------------------------------------------
int nodeID             = 4;      // ID of node 04-"Whiskey", 05-"Vodka", 06-"Lager", 07-"Cider"
//------------------------DATA SETUP-------------------------------------------
boolean eventFlag       = false;
// Reference State Machine Diagram on the Whiteboard in VL E361 As Of 4/23/16 1:00 AM 
int S1TO                =  100;  // State 1 Iteration Time Out
int S2TO                =  100;  // State 2 Iteration Time Out
int S3TO                =  100;  // State 3 Iteration Time Out
int S4TO                =  100;  // State 4 Iteration Time Out
// There is an event 5 where it sees if there's any interesting data to flag as an event
int limiter             =  0;  // General counter used in all state time-outs
int state               =  1;  // Current state
int TimeCycle           =  10; // Full Time Cycle of TX and RX in ms
//-------------------------PIN SETUP-------------------------------------------
const int sensorPin     = 14;          // Pin of Temperature sensor 
const int GDO0_PIN      = 16;           // the number of the GDO0_PIN pin
  int GDO0_State        = 0;           // variable for reading the pushbutton status
const int LED           = 5;           // LED pin 
const int GATEOUT       = 0;           // Gate set read-out
const int GATEIN        = 1;           // Gate state read-in
boolean isGate          = false;       // Gate boolean 
//--------------------------END SETUP------------------------------------------

void setup(){
  delay(2000); 
  Serial.begin(9600);
  pinMode(SS,OUTPUT);
  SPI.begin();
  digitalWrite(SS,HIGH);
  pinMode(GDO0_PIN, INPUT);     
  pinMode(sensorPin, INPUT); 
  pinMode(LED,OUTPUT); 
  
  //--------------Determine if Gate
  pinMode(GATEOUT,OUTPUT); 
  pinMode(GATEIN,INPUT); 
  digitalWrite(GATEOUT,HIGH); 
  if (digitalRead(GATEIN)==HIGH){
    isGate = true; 
    Serial.println("Gate Activated - Listening for Data..."); 
  }
  //--------------
  
  //Read_Config_Regs(); //Initialize and check status of certain registers. Decomment if concenered 
}

void loop(){
  Packet gPacket; //General use packet   
  //STATE S1 - TX: n/a, RX: ADV ---------------------------------------------------------------o S1
  state = 1; 
  limiter = 0; 
  while(!GDO0_State){
     GDO0_State = digitalRead(GDO0_PIN); 
     limiter++; 
     if(limiter>S1TO){
      state = 5;  
      break;
     }
     delay(10); 
  }
  while(GDO0_State){ //Packet Receive detected, waiting for full packet receive
    GDO0_State = digitalRead(GDO0_PIN); 
    delay(10); 
  }
  
  gPacket = RX(); //updates the global variable pkt[4]; 
  if(gPacket.type()==ADV){ 
    //Listening for Advertisement
    if(isInteresting(gPacket)){
      state = 2; //Advance to next state knowing we heard a packet that's interesting
    } 
  }
  else{
    state = 1; //Heard a non-Advertisement, so keep at it. 
  }
  
  //STATE S2 - TX: REQ, RX: DAT---------------------------------------------------------------o S2
  if(state==2){
    limiter = 0;  
     
    // INSERT CHECK MODE CODE 
    gPacket.setType(REQ); 
    TX(gPacket); //TX Req

    while(!GDO0_State){
     GDO0_State = digitalRead(GDO0_PIN); 
     limiter++; 
     if(limiter>S2TO){
      state = 1;  
      break;
     }
     delay(10); 
    }
    while(GDO0_State){ //Packet Receive detected, waiting for full packet receive
      GDO0_State = digitalRead(GDO0_PIN); 
      delay(10); 
    }
    
    gPacket = RX(); //updates the global variable pkt[4];  
    if(gPacket.type()==DAT){
      gPacket.setType(ACK); 
      TX(gPacket); 
      gPacket.setType(ADV); 
      state = 3; 
    }
    else{
      state = 2; //Try again
    }
      
    }
   
  if(!isGate){ //States S3 and S4 are locked only to nodes. Gates only do states S1 and S2 and report all data through serial through puts. 
  //STATE S3 - TX: ADV, RX: REQ---------------------------------------------------------------o S3
  if(state==3){
    limiter = 0; 
    
    TX(gPacket); 

    
    while(!GDO0_State){
     GDO0_State = digitalRead(GDO0_PIN); 
     limiter++; 
     if(limiter>S3TO){
      state = 1;  
      break;
     }
     delay(10); 
    }
    while(GDO0_State){ //Packet Receive detected, waiting for full packet receive
      GDO0_State = digitalRead(GDO0_PIN); 
      delay(10); 
    }

    gPacket = RX(); 
    if(gPacket.type()==REQ){
      state = 4; 
      gPacket.setType(DAT); 
    }
    else{
      state = 1; 
    }
    
  }
  //STATE S4 - TX: DAT, RX: ACK---------------------------------------------------------------o S4
  if(state==4){
    limiter = 0; 
    TX(gPacket); 

    while(!GDO0_State){
     GDO0_State = digitalRead(GDO0_PIN); 
     limiter++; 
     if(limiter>S4TO){
      state = 1;  
      break;
     }
     delay(10); 
    }
    while(GDO0_State){ //Packet Receive detected, waiting for full packet receive
      GDO0_State = digitalRead(GDO0_PIN); 
      delay(10); 
    }

    gPacket = RX(); 
    if(gPacket.type()==ACK){
      state = 1; 
      gPacket.setType(DAT); 
    }
    else{
      state = 4; //Retry sending Data
    }
    
    
  }
  //STATE S5 - EVENT--------------------------------------------------------------------------o S5
  if(state==5){
    int eventDat = readTemp(); 
    state = 3; //Let the world know you have something interesting to say.  
  }
  } //!isGate
}

Packet RX(void){
  
  char packet; 
  SendStrobe(CC2500_RX); //RX State enabled on CC2500

  int packetLength = ReadReg(CC2500_RXFIFO); 
  char tempPack[packetLength];
  tempPack[0] = packetLength; 
  for(int itt = 1; itt <= packetLength; itt++){
    tempPack[itt] = ReadReg(CC2500_RXFIFO); 
  }
  if(packetLength<4){
    tempPack[3] = 0x00; 
  }
  SendStrobe(CC2500_IDLE); 
  SendStrobe(CC2500_FRX); 
  Packet pk(tempPack[0],tempPack[1],tempPack[2],tempPack[3]); 
  return pk; 
  //return tempPack; 
}

void TX(Packet pk){
  char packetLen = pk.length(); 
  char mode = pk.type(); 
  //char txpack[4];
  //txpack = pk.full(); 
  
  WriteReg(CC2500_TXFIFO,pk.length());
  WriteReg(CC2500_TXFIFO,pk.nodeName());
  WriteReg(CC2500_TXFIFO,pk.type());  
  if(mode == DAT){
    WriteReg(CC2500_TXFIFO,pk.data()); 
  }
  /*for(int i = 0; i < packetLen; i++){
    WriteReg(CC2500_TXFIFO,txpack[i]); 
  }
  */
  SendStrobe(CC2500_TX); 
  while (!GDO0_State)
      {
          // read the state of the GDO0_PIN value:
          GDO0_State = digitalRead(GDO0_PIN);
       }
       // Wait for GDO0 to be cleared -> end of packet
       while (GDO0_State)
       {
           // read the state of the GDO0_PIN value:
           GDO0_State = digitalRead(GDO0_PIN);
       }
  SendStrobe(CC2500_IDLE);
  SendStrobe(CC2500_FTX);  
}

boolean isInteresting(Packet tempPacket){
  //compares if data packet has passed through before and/or if the packet is from this node 
  return true; //Everything is interesting when you've drunk enough 
}

void WriteReg(char addr, char value)
{
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  SPI.transfer(addr);
  delay(10);
  SPI.transfer(value);
  digitalWrite(SS,HIGH);
}

char ReadReg(char addr)
{
  addr = addr + 0x80;
  digitalWrite(SS,LOW);
  while (digitalRead(MISO) == HIGH) {
    };
  SPI.transfer(addr);
  delay(10);
  char y = SPI.transfer(0);
  digitalWrite(SS,HIGH);
  return y;  
}

char SendStrobe(char strobe)
{
  digitalWrite(SS,LOW);
  
  while (digitalRead(MISO) == HIGH) {
    };
  char result =  SPI.transfer(strobe);
  digitalWrite(SS,HIGH);
  delay(10);
  return result;
}

int readTemp(){ //getting the voltage reading from the temperature sensor
 int reading = analogRead(sensorPin);  
 
 // converting that reading to voltage, for 3.3v arduino use 3.3
 float voltage = reading * 3.3;
 voltage /= 1024.0; 
 float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                              //to degrees ((voltage - 500mV) times 100)
 return ceil(temperatureC); //Just round up for now 
}

void init_CC2500()
{
  WriteReg(REG_IOCFG2,VAL_IOCFG2);
  WriteReg(REG_IOCFG1,VAL_IOCFG1);
  WriteReg(REG_IOCFG0,VAL_IOCFG0);

  WriteReg(REG_FIFOTHR,VAL_FIFOTHR);
  WriteReg(REG_SYNC1,VAL_SYNC1);
  WriteReg(REG_SYNC0,VAL_SYNC0);
  WriteReg(REG_PKTLEN,VAL_PKTLEN);
  WriteReg(REG_PKTCTRL1,VAL_PKTCTRL1);
  WriteReg(REG_PKTCTRL0,VAL_PKTCTRL0);
  WriteReg(REG_ADDR,VAL_ADDR);
  WriteReg(REG_CHANNR,VAL_CHANNR);
  WriteReg(REG_FSCTRL1,VAL_FSCTRL1);
  WriteReg(REG_FSCTRL0,VAL_FSCTRL0);
  WriteReg(REG_FREQ2,VAL_FREQ2);
  WriteReg(REG_FREQ1,VAL_FREQ1);
  WriteReg(REG_FREQ0,VAL_FREQ0);
  WriteReg(REG_MDMCFG4,VAL_MDMCFG4);
  WriteReg(REG_MDMCFG3,VAL_MDMCFG3);
  WriteReg(REG_MDMCFG2,VAL_MDMCFG2);
  WriteReg(REG_MDMCFG1,VAL_MDMCFG1);
  WriteReg(REG_MDMCFG0,VAL_MDMCFG0);
  WriteReg(REG_DEVIATN,VAL_DEVIATN);
  WriteReg(REG_MCSM2,VAL_MCSM2);
  WriteReg(REG_MCSM1,VAL_MCSM1);
  WriteReg(REG_MCSM0,VAL_MCSM0);
  WriteReg(REG_FOCCFG,VAL_FOCCFG);

  WriteReg(REG_BSCFG,VAL_BSCFG);
  WriteReg(REG_AGCCTRL2,VAL_AGCCTRL2);
  WriteReg(REG_AGCCTRL1,VAL_AGCCTRL1);
  WriteReg(REG_AGCCTRL0,VAL_AGCCTRL0);
  WriteReg(REG_WOREVT1,VAL_WOREVT1);
  WriteReg(REG_WOREVT0,VAL_WOREVT0);
  WriteReg(REG_WORCTRL,VAL_WORCTRL);
  WriteReg(REG_FREND1,VAL_FREND1);
  WriteReg(REG_FREND0,VAL_FREND0);
  WriteReg(REG_FSCAL3,VAL_FSCAL3);
  WriteReg(REG_FSCAL2,VAL_FSCAL2);
  WriteReg(REG_FSCAL1,VAL_FSCAL1);
  WriteReg(REG_FSCAL0,VAL_FSCAL0);
  WriteReg(REG_RCCTRL1,VAL_RCCTRL1);
  WriteReg(REG_RCCTRL0,VAL_RCCTRL0);
  WriteReg(REG_FSTEST,VAL_FSTEST);
  WriteReg(REG_PTEST,VAL_PTEST);
  WriteReg(REG_AGCTEST,VAL_AGCTEST);
  WriteReg(REG_TEST2,VAL_TEST2);
  WriteReg(REG_TEST1,VAL_TEST1);
  WriteReg(REG_TEST0,VAL_TEST0);
/*  
  WriteReg(REG_PARTNUM,VAL_PARTNUM);
  WriteReg(REG_VERSION,VAL_VERSION);
  WriteReg(REG_FREQEST,VAL_FREQEST);
  WriteReg(REG_LQI,VAL_LQI);
  WriteReg(REG_RSSI,VAL_RSSI);
  WriteReg(REG_MARCSTATE,VAL_MARCSTATE);
  WriteReg(REG_WORTIME1,VAL_WORTIME1);
  WriteReg(REG_WORTIME0,VAL_WORTIME0);
  WriteReg(REG_PKTSTATUS,VAL_PKTSTATUS);
  WriteReg(REG_VCO_VC_DAC,VAL_VCO_VC_DAC);
  WriteReg(REG_TXBYTES,VAL_TXBYTES);
  WriteReg(REG_RXBYTES,VAL_RXBYTES);
  WriteReg(REG_RCCTRL1_STATUS,VAL_RCCTRL1_STATUS);
  WriteReg(REG_RCCTRL0_STATUS,VAL_RCCTRL0_STATUS);
  */
}

void Read_Config_Regs(void)
{ 
  Serial.println(ReadReg(REG_IOCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_IOCFG0),HEX);
   delay(10);
/* Serial.println(ReadReg(REG_FIFOTHR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_SYNC0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTLEN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PKTCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_ADDR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_CHANNR),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREQ0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG4),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG3),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MDMCFG0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_DEVIATN),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_MCSM0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FOCCFG),HEX);
   delay(10);

  Serial.println(ReadReg(REG_BSCFG),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WOREVT1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WOREVT0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_WORCTRL),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREND1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FREND0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL3),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSCAL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_RCCTRL1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_RCCTRL0),HEX);
   delay(10);
  Serial.println(ReadReg(REG_FSTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_PTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_AGCTEST),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST2),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST1),HEX);
   delay(10);
  Serial.println(ReadReg(REG_TEST0),HEX);
   delay(10);

  Serial.println(ReadReg(REG_PARTNUM),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_VERSION),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_FREQEST),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_LQI),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RSSI),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_MARCSTATE),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_WORTIME1),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_WORTIME0),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_PKTSTATUS),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_VCO_VC_DAC),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_TXBYTES),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RXBYTES),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RCCTRL1_STATUS),HEX);
   delay(1000);
  Serial.println(ReadReg(REG_RCCTRL0_STATUS),HEX);
   delay(1000);
*/  

/*
 * WREK-Atlanta 91.1 FM
 */

}

