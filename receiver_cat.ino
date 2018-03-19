/*  Developed in Arduino IDE 1.8.3
 *  Running on the UNO
 *  By: Daniel Ekman SA2KNG <knegge@gmail.com>
 *  Created in February 2018
 *
 *  code from OE1CGS Christoph Schwärzler http://qrp-labs.com/synth/oe1cgs.html
 *  and G3ZIL Gwyn Griffiths gxgriffiths@virginmedia.com http://qrp-labs.com/images/uarduino/g3zil/G3ZIL_WSPR_Tx-Rx_V5.ino
 *  
 *  This project uses the Arduino shield http://qrp-labs.com/uarduino.html
 *  with the synth module http://qrp-labs.com/synth
 *  to make a cat controlled receiver http://qrp-labs.com/receiver.html
 *  preferably with the polyphase to produce ssb output http://qrp-labs.com/polyphase.html
 *  and optionally with relay board http://qrp-labs.com/ultimatelpf.html
 */


// pin summary for reference
// d0     Reserved: USB SERIAL IN and GPS data in
// d1     Reserved: USB SERIAL OUT 
// d2
// d3     Reserved: WCLK to Si5351A
// d4
// d5     Reserved: CLK0 from Si5351A daughter board.
// d6
// d7     Relay 0 (onboard shield)
// d8 
// d9     Reserved: GPS 1PPS, not used
// d10    Relay 2
// d11    Relay 3
// d12    Relay 4
// d13
// A0     Relay 1
// A1
// A2
// A3     Relay 5
// A4     Reserved: TWI - SDA - reserved [Used to talk to the Si5351A and the DS3231]
// A5     Reserved: TWI - SCL - reserved  [Used to talk to the Si5351A and the DS3231]

// this is the 10pin header that fits directly to the relay board
int rl0 = 7; // onboard relay
int rl1 = A0;
int rl2 = 10;
int rl3 = 11;
int rl4 = 12;
int rl5 = A3;

#include <Wire.h>

#define F_XTAL 27003680;               // This needs to be measured and calibrated based on your synth board, this sketch does not support gps 1pps as of now
#define c 1048574;                     // "c" part of Feedback-Multiplier from XTAL to PLL
#define BUFFLEN 16                     // Serial and asciiZ buffer length
#define RIG_ID F("ID020;")             // Answer to rig identification, hamlib TS480 -m 228

// Frequency table for relays, lower relay# has priority, adjust for your filter span
// Relay 0 not used
#define rl0_lo 0
#define rl0_hi 0
// Relay 1 80m LPF
#define rl1_lo 0
#define rl1_hi 3800000
// Relay 2 40m BPF
#define rl2_lo 7000000
#define rl2_hi 7200000
// Relay 3 30m BPF
#define rl3_lo 10100000
#define rl3_hi 10150000
// Relay 4 20m BPF
#define rl4_lo 14000000
#define rl4_hi 14350000
// Relay 5 10m LPF (or catch rest)
#define rl5_lo 0
#define rl5_hi 30000000

int clk = 1;  
unsigned long TX_MSNB_P2;              // Si5351 register MSNB_P2  PLLB for Tx
unsigned long TX_P2;                   // Variable values for MSNB_P2 which defines the frequencies for the data

unsigned long vfoa;                    // Current frequency
unsigned char rxbuf[BUFFLEN], rigcat[BUFFLEN];
char vfostr[BUFFLEN];
unsigned int rxpos, rxread, rxs, opmode=2;
bool rxcomplete;
unsigned long curTime;
const long timeout = 1000; // 1s timeout on serial communication

void setup()
{
  Serial.begin(9600);                  //open a serial port - for debugging, commented out normally
  Wire.begin();                        // Initialize I2C-communication as master: SDA on pin ADC04, SCL on pin ADC05

  pinMode(rl0, OUTPUT);
  pinMode(rl1, OUTPUT);
  pinMode(rl2, OUTPUT);
  pinMode(rl3, OUTPUT);
  pinMode(rl4, OUTPUT);
  pinMode(rl5, OUTPUT);
  digitalWrite(rl0, HIGH);
  digitalWrite(rl1, HIGH);
  digitalWrite(rl2, HIGH);
  digitalWrite(rl3, HIGH);
  digitalWrite(rl4, HIGH);
  digitalWrite(rl5, HIGH);

  vfoa=10000000ul;
  Set_RX_LO_Frequency (vfoa*4);        // Set TX-Frequency default at 10MHz
  SetPower (4);
  RX_LO_ON();
}

void loop()
{
  int i;
  if(rxpos==rxread) curTime = millis();
  if(millis() > curTime + timeout){
    Serial.print("E;");
    rxread=rxpos;
  }

  if(rxavail()){ // parsing of the cat commands
    rxs=255; // status 255, unhandled packet
    if(rigcat[0] == 'A'){
      if(rigcat[1]=='I'){ // AI, auto information
        rxs=0;
        if(rigcat[2]==';'){ // get
          rxs=0; // no further processing
          Serial.print(F("AI0;"));
        }else if(rigcat[3]==';'){
          rxs=0;
        }
      }
    }else if(rigcat[0] == 'F'){
      if(rigcat[1] == 'A'){ // FA, get/set VFO A
        rxs=254;
        if(rigcat[2] == ';'){ // get
          rxs=0;
          sprintf(vfostr, "FA%011lu;", vfoa);
          Serial.print(vfostr);
        }else if(rigcat[13] == ';'){ // set
          rxs=0;
          memset(vfostr,0,sizeof(vfostr));
          for(i=0;i<11;i++){
            vfostr[i]=rigcat[i+2];
          }
          sscanf(vfostr,"%lu",&vfoa);
          Set_RX_LO_Frequency (vfoa*4);
          SetPower (4);
          RX_LO_ON();
        }
      }else if(rigcat[1] == 'B'){ // FB, get/set VFO B (DOES NOT DIFFER FROM A!!)
        rxs=254;
        if(rigcat[2] == ';'){ // get
          rxs=0;
          sprintf(vfostr, "FB%011lu;", vfoa);
          Serial.print(vfostr);
        }else if(rigcat[13] == ';'){ // set
          rxs=0;
          memset(vfostr,0,sizeof(vfostr));
          for(i=0;i<11;i++){
            vfostr[i]=rigcat[i+2];
          }
          sscanf(vfostr,"%lu",&vfoa);
          Set_RX_LO_Frequency (vfoa*4);
          SetPower (4);
          RX_LO_ON();
        }
      }else if(rigcat[1] == 'L'){ // FL, filter
        rxs=254;
        if(rigcat[2] == ';'){ // get
          rxs=0;
          Serial.print(F("FL000000;")); // FIXME
        }
      }else if(rigcat[1] == 'R'){ // FR, get/set receiver VFO A/B
        rxs=254;
        if(rigcat[2] == ';'){ // get
          rxs=0;
          Serial.print(F("FR0;"));
        }else if(rigcat[3] == ';'){ // set
          rxs=0;
        }
      }else if(rigcat[1] == 'T'){ // FT, get/set transmitter VFO A/B
        rxs=254;
        if(rigcat[2] == ';'){ // get
          rxs=0;
          Serial.print(F("FT0;"));
        }else if(rigcat[3] == ';'){ // set
          rxs=0;
        }
      }else if(rigcat[1] == 'V' && rigcat[2] == ';'){ // FV, get firmware version
        rxs=0;
        Serial.print(F("FV1.00;"));
      }
    }else if(rigcat[0] == 'I'){
      if(rigcat[1] == 'D' && rigcat[2] == ';'){ // ID, read transceiver identity
        rxs=0;
        Serial.print(RIG_ID); // some programs autodetect/verify the model ID!
      }else if(rigcat[1] == 'F' && rigcat[2] == ';'){ // IF, read transceiver status
        rxs=0;
        sprintf(vfostr, "IF%011lu", vfoa);
        Serial.print(vfostr);
        Serial.print(F("     ")); // P2 five spaces
        Serial.print(F("00000000000")); // P3-P8
        Serial.print(opmode); // P9 opmode
        Serial.print(F("000000 ;"));
      }
    }else if(rigcat[0] == 'M'){
      if(rigcat[1] == 'D' && rigcat[2] == ';'){ // MD, operating mode
        rxs=0;
        Serial.print(F("MD"));
        Serial.print(opmode);
        Serial.print(F(";"));
      }else if(rigcat[1] == 'D' && rigcat[3] == ';'){ // set
        rxs=254;
        if(rigcat[2] >= '0' && rigcat[2] <= '9'){
          rxs=0;
          opmode = rigcat[2]-'0';
        }
      }
    }else if(rigcat[0] == 'N'){
      if(rigcat[1] == 'A' && rigcat[2] == ';'){ // NA, radio name
        rxs=0;
        Serial.print(F("NASA2KNG;"));
      }
    }else if(rigcat[0] == 'P'){
      if(rigcat[1] == 'S' && rigcat[2] == ';'){ // PS, power status
        rxs=0;
        Serial.print(F("PS1;"));
      }else if(rigcat[1] == 'T' && rigcat[2] == ';'){ // PT, pitch ?
        rxs=0;
        Serial.print(F("PT00;"));
      }
    }else if(rigcat[0] == 'R'){
      if(rigcat[1] == 'C' && rigcat[2] == ';'){ // RC, clear rit
        rxs=0;
      }else if(rigcat[1] == 'S' && rigcat[2] == ';'){ // RS
        rxs=0;
        Serial.print(F("RS0;"));
      }else if(rigcat[1] == 'T' && rigcat[2] == ';'){ // RT
        rxs=0;
        Serial.print(F("RT0;"));
      }else if(rigcat[1] == 'X' /*&& rigcat[2] == ';'*/){ // RX, receive
        rxs=0;
      }
    }else if(rigcat[0] == 'S'){
      if(rigcat[1] == 'H' && rigcat[2] == ';'){ // SH high pass freq
        rxs=0;
        Serial.print(F("SH09;"));
      }else if(rigcat[1] == 'H' && rigcat[4] == ';'){ // SH set
        rxs=0;
      }else if(rigcat[1] == 'L' && rigcat[2] == ';'){ // SL, low pass freq
        rxs=0;
        Serial.print(F("SL00;"));
      }else if(rigcat[1] == 'L' && rigcat[4] == ';'){ // SL set
        rxs=0;
      }else if(rigcat[1] == 'M' && rigcat[3] == ';'){ // SM, signal meter
        rxs=0;
        Serial.print(F("SM00000;"));
      }
    }else if(rigcat[0] == 'T'){
      if(rigcat[1] == 'O' && rigcat[2] == ';'){ // TO, tone status
        rxs=0;
        Serial.print(F("TO0;"));
      }else if(rigcat[1] == 'X' /*&& rigcat[2] == ';'*/){ // TX, transmit
        rxs=0;
      }
    }

    if(rxs==255){
      rxs=0;
      Serial.print(F("?;")); // bad command syntax | unknown command or busy
    }else if(rxs==254){
      rxs=0;
      Serial.print(F("O;")); // processing not completed | overflow
    }else if(rxs==253){
      rxs=0;
      Serial.print(F("E;")); // error
    }
  }

  // Relay filter control
  if(vfoa >= rl0_lo && vfoa <= rl0_hi){
    //digitalWrite(rl0, LOW);
    digitalWrite(rl1, HIGH);
    digitalWrite(rl2, HIGH);
    digitalWrite(rl3, HIGH);
    digitalWrite(rl4, HIGH);
    digitalWrite(rl5, HIGH);
  }else if(vfoa >= rl1_lo && vfoa <= rl1_hi){
    //digitalWrite(rl0, HIGH);
    digitalWrite(rl1, LOW);
    digitalWrite(rl2, HIGH);
    digitalWrite(rl3, HIGH);
    digitalWrite(rl4, HIGH);
    digitalWrite(rl5, HIGH);
  }else if(vfoa >= rl2_lo && vfoa <= rl2_hi){
    //digitalWrite(rl0, HIGH);
    digitalWrite(rl1, HIGH);
    digitalWrite(rl2, LOW);
    digitalWrite(rl3, HIGH);
    digitalWrite(rl4, HIGH);
    digitalWrite(rl5, HIGH);
  }else if(vfoa >= rl3_lo && vfoa <= rl3_hi){
    //digitalWrite(rl0, HIGH);
    digitalWrite(rl1, HIGH);
    digitalWrite(rl2, HIGH);
    digitalWrite(rl3, LOW);
    digitalWrite(rl4, HIGH);
    digitalWrite(rl5, HIGH);
  }else if(vfoa >= rl4_lo && vfoa <= rl4_hi){
    //digitalWrite(rl0, HIGH);
    digitalWrite(rl1, HIGH);
    digitalWrite(rl2, HIGH);
    digitalWrite(rl3, HIGH);
    digitalWrite(rl4, LOW);
    digitalWrite(rl5, HIGH);
  }else if(vfoa >= rl5_lo && vfoa <= rl5_hi){
    //digitalWrite(rl0, HIGH);
    digitalWrite(rl1, HIGH);
    digitalWrite(rl2, HIGH);
    digitalWrite(rl3, HIGH);
    digitalWrite(rl4, HIGH);
    digitalWrite(rl5, LOW);
  }else {
    //digitalWrite(rl0, HIGH);
    digitalWrite(rl1, HIGH);
    digitalWrite(rl2, HIGH);
    digitalWrite(rl3, HIGH);
    digitalWrite(rl4, HIGH);
    digitalWrite(rl5, HIGH);
  }
  if(opmode==1) { // opmode: 1=LSB, 2=USB (and rest). relay changes from USB to LSB when activated (LOW)
    digitalWrite(rl0, LOW);
  }else{
    digitalWrite(rl0, HIGH);
  }
}

/* handle incoming serial data and put it into the buffer */
void serialEvent(){
  int i;
  while(Serial.available()){
    i = Serial.read();
    if(i != '\r' && i != '\n'){
      rxbuf[rxpos]=(char)i;
      rxpos = (rxpos + 1) % BUFFLEN;
    }
  }
}

/* This function checks if there's a completed rigcac command in the receive buffer and copies it into a separate buffer */
bool rxavail(void){
  int i, len;
  bool com;
  if(rxpos==rxread) return false; // no new data, or complete overrun
  com = false;
  len = (BUFFLEN - rxread + rxpos) % BUFFLEN; // total length of new data
  for(i=0;i<len;i++){ // find next ';'
    if(rxbuf[(rxread+i)%BUFFLEN]==';'){
      com=true;
      len=i; // length to first ';'
      exit;
    }
  }
  if(com){
    for(i=0;i<BUFFLEN;i++){
      rigcat[i]=0;
    }
    for(i=0;i<=len;i++){
      rigcat[i]=rxbuf[(rxread+i)%BUFFLEN];
    }
    rxread = (rxread+i)%BUFFLEN;
    return true;
  }
  return false;
}


//--------------------------------------------------------------------
// Now for the Si5351a RECEIVER set-up functions Using Clock 1, Multisynth1 and PLLA
//--------------------------------------------------------------------
//
void RX_LO_ON () 
{                        
  Si5351a_Write_Reg (17, 79);          // Enable output CLK1, set MS1 as source, Integer Mode on PLLA and max current
}

void RX_LO_OFF () {                       
  Si5351a_Write_Reg (17, 128);         // Disable output CLK1
}

void Set_RX_LO_Frequency (unsigned long frequency) { // Frequency in Hz; must be within [7810 Hz to 200 Mhz]
  unsigned long fvco;                  // VCO frequency (600-900 MHz) of PLL
  unsigned long outdivider;            // Output divider in range [4,6,8-900], even numbers preferred
  byte R = 1;                          // Additional Output Divider in range [1,2,4,...128]
  byte a;                              // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  unsigned long b;                     // "b" part of Feedback-Multiplier from XTAL to PLL
  float f;                             // floating variable, needed in calculation
  unsigned long MS0_P1;                // Si5351a Output Divider register MS0_P1, P2 and P3 are hardcoded below
  unsigned long MSNA_P1;               // Si5351a Feedback Multisynth register MSNA_P1
  unsigned long MSNA_P2;               // Si5351a Feedback Multisynth register MSNA_P2
  unsigned long MSNA_P3;               // Si5351a Feedback Multisynth register MSNA_P3

  outdivider = 900000000 / frequency;  // With 900 MHz beeing the maximum internal PLL-Frequency
  
  while (outdivider > 900){            // If output divider out of range (>900) use additional Output divider
    R = R * 2;
    outdivider = outdivider / 2;
  }
  if (outdivider % 2) outdivider--;    // finds the even divider which delivers the intended Frequency

  fvco = outdivider * R * frequency;   // Calculate the PLL-Frequency (given the even divider)

  switch (R){                          // Convert the Output Divider to the bit-setting required in register 44
    case 1: R = 0; break;              // Bits [6:4] = 000
    case 2: R = 16; break;             // Bits [6:4] = 001
    case 4: R = 32; break;             // Bits [6:4] = 010
    case 8: R = 48; break;             // Bits [6:4] = 011
    case 16: R = 64; break;            // Bits [6:4] = 100
    case 32: R = 80; break;            // Bits [6:4] = 101
    case 64: R = 96; break;            // Bits [6:4] = 110
    case 128: R = 112; break;          // Bits [6:4] = 111
  }

  a = fvco / F_XTAL;                   // Multiplier to get from Quartz-Oscillator Freq. to PLL-Freq.
  f = fvco - a * F_XTAL;               // Multiplier = a+b/c
  f = f * c;                           // this is just "int" and "float" mathematics
  f = f / F_XTAL;
  b = f;

  MS0_P1 = 128 * outdivider - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                       // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below

  f = 128 * b / c;                     // Calculation of Feedback Multisynth registers MSNA_P1 to MSNA_P3
  MSNA_P1 = 128 * a + f - 512;
  MSNA_P2 = f;
  MSNA_P2 = 128 * b - MSNA_P2 * c; 
  MSNA_P3 = c;

  Si5351a_Write_Reg (17, 128);                      // Disable output during the following register settings 
  Si5351a_Write_Reg (26, (MSNA_P3 & 65280) >> 8);   // Bits [15:8] of MSNA_P3 in register 26
  Si5351a_Write_Reg (27, MSNA_P3 & 255);            // Bits [7:0]  of MSNA_P3 in register 27
  Si5351a_Write_Reg (28, (MSNA_P1 & 196608) >> 16); // Bits [17:16] of MSNA_P1 in bits [1:0] of register 28
  Si5351a_Write_Reg (29, (MSNA_P1 & 65280) >> 8);   // Bits [15:8]  of MSNA_P1 in register 29
  Si5351a_Write_Reg (30, MSNA_P1 & 255);            // Bits [7:0]  of MSNA_P1 in register 30
  Si5351a_Write_Reg (31, ((MSNA_P3 & 983040) >> 12) | ((MSNA_P2 & 983040) >> 16)); // Parts of MSNA_P3 und MSNA_P1
  Si5351a_Write_Reg (32, (MSNA_P2 & 65280) >> 8);   // Bits [15:8]  of MSNA_P2 in register 32
  Si5351a_Write_Reg (33, MSNA_P2 & 255);            // Bits [7:0]  of MSNA_P2 in register 33
  Si5351a_Write_Reg (50, 0);                        // Bits [15:8] of MS1_P3 (always 0) in register 50
  Si5351a_Write_Reg (51, 1);                        // Bits [7:0]  of MS1_P3 (always 1) in register 51
  Si5351a_Write_Reg (52, ((MS0_P1 & 196608) >> 16) | R);  // Bits [17:16] of MS1_P1 in bits [1:0] and R in [7:4]
  Si5351a_Write_Reg (53, (MS0_P1 & 65280) >> 8);    // Bits [15:8]  of MS1_P1 in register 53
  Si5351a_Write_Reg (54, MS0_P1 & 255);             // Bits [7:0]  of MS1_P1 in register 54
  Si5351a_Write_Reg (55, 0);                        // Bits [19:16] of MS1_P2 and MS0_P3 are always 0
  Si5351a_Write_Reg (56, 0);                        // Bits [15:8]  of MS1_P2 are always 0
  Si5351a_Write_Reg (57, 0);                        // Bits [7:0]   of MS1_P2 are always 0
  if (outdivider == 4){
    Si5351a_Write_Reg (52, 12 | R);                 // Special settings for R = 4 (see datasheet)
    Si5351a_Write_Reg (53, 0);                      // Bits [15:8]  of MS1_P1 must be 0
    Si5351a_Write_Reg (54, 0);                      // Bits [7:0]  of MS1_P1 must be 0
  } 
  Si5351a_Write_Reg (177, 32);                      // This resets PLL A
}

void SetPower (byte power){                         // Sets the output power level
  if (power == 0 || power > 4){power = 0;}          // valid power values are 0 (25%), 1 (50%), 2 (75%) or 3 (100%)
  switch (power){
    case 1:
      Si5351a_Write_Reg (17, 76);                   // CLK1 drive strength = 2mA; power level ~ -8dB
      break;
    case 2:
      Si5351a_Write_Reg (17, 77);                   // CLK1 drive strength = 4mA; power level ~ -3dB
      break;
    case 3:
      Si5351a_Write_Reg (17, 78);                   // CLK1 drive strength = 6mA; power level ~ -1dB
      break;
    case 4:
      Si5351a_Write_Reg (17, 79);                   // CLK1 drive strength = 8mA; power level := 0dB
      break;
  }
}

/*
//--------------------------------------------------------------------
// Now for the Si5351a TRANSMIT set-up functions Using Clock 0, Multisynth0 and PLLB
//--------------------------------------------------------------------
//

void TX (unsigned long P2)                      // Changes TX frequency according to channel
  {                   
  Si5351a_Write_Reg (40, (P2 & 65280) >> 8);   // Bits [15:8]  of MSNB_P2 in register 40
  Si5351a_Write_Reg (41, P2 & 255);            // Bits [7:0]  of MSNB_P2 in register 41
  }

void TX_ENB () 
{                                              // Enables output on CLK0
  Si5351a_Write_Reg (16, 111);                 // Enable output CLK0, Integer Mode, PLLB, MS0 as source
}

void TX_OFF () 
{                                              // Disables output on CLK0
  Si5351a_Write_Reg (16, 128);
}

unsigned long Set_TX_Frequency (unsigned long TX_frequency)  // Frequency in Hz; must be within [7810 Hz to 200 Mhz]
{ 
  unsigned long TX_fvco;                       // VCO frequency (600-900 MHz) of PLL
  unsigned long TX_outdivider;                 // Output divider in range [4,6,8-900], even numbers preferred
  byte TX_R = 1;                               // Additional Output Divider in range [1,2,4,...128]
  byte TX_a;                                   // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  unsigned long TX_b;                          // "b" part of Feedback-Multiplier from XTAL to PLL
  float TX_f;                                  // floating variable, needed in calculation
  unsigned long MS0_P1;                        // Si5351a Output Divider register MS0_P1, P2 and P3 are hardcoded below
  unsigned long MSNB_P1;                       // Si5351a Feedback Multisynth register MSNB_P1
  unsigned long MSNB_P2;                       // Si5351a Feedback Multisynth register MSNB_P2
  unsigned long MSNB_P3;                       // Si5351a Feedback Multisynth register MSNB_P3

  TX_outdivider = 900000000 / TX_frequency;       // With 900 MHz beeing the maximum internal PLL-Frequency
  
  while (TX_outdivider > 900){                    // If output divider out of range (>900) use additional Output divider
    TX_R = TX_R * 2;
    TX_outdivider = TX_outdivider / 2;
  }
  if (TX_outdivider % 2) TX_outdivider--;         // finds the even divider which delivers the intended Frequency

  TX_fvco = TX_outdivider * TX_R * TX_frequency;  // Calculate the PLL-Frequency (given the even divider)

  switch (TX_R){                          // Convert the Output Divider to the bit-setting required in register 44
    case 1: TX_R = 0; break;              // Bits [6:4] = 000
    case 2: TX_R = 16; break;             // Bits [6:4] = 001
    case 4: TX_R = 32; break;             // Bits [6:4] = 010
    case 8: TX_R = 48; break;             // Bits [6:4] = 011
    case 16: TX_R = 64; break;            // Bits [6:4] = 100
    case 32: TX_R = 80; break;            // Bits [6:4] = 101
    case 64: TX_R = 96; break;            // Bits [6:4] = 110
    case 128: TX_R = 112; break;          // Bits [6:4] = 111
  }

  TX_a = TX_fvco / F_XTAL;      // Multiplier to get from Quartz-Oscillator Freq. to PLL-Freq.
  TX_f = TX_fvco - TX_a * F_XTAL;         // Multiplier = a+b/c
  TX_f = TX_f * c;                        // this is just "int" and "float" mathematics
  TX_f = TX_f / F_XTAL;
  TX_b = TX_f;

  MS0_P1 = 128 * TX_outdivider - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                          // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below

  TX_f = 128 * TX_b / c;                  // Calculation of Feedback Multisynth registers MSNB_P1 to MSNB_P3
  MSNB_P1 = 128 * TX_a + TX_f - 512;
  MSNB_P2 = TX_f;
  MSNB_P2 = 128 * TX_b - MSNB_P2 * c; 
  MSNB_P3 = c;

  Si5351a_Write_Reg (16, 128);                      // Disable output during the following register settings 
  Si5351a_Write_Reg (34, (MSNB_P3 & 65280) >> 8);   // Bits [15:8] of MSNB_P3 in register 34
  Si5351a_Write_Reg (35, MSNB_P3 & 255);            // Bits [7:0]  of MSNB_P3 in register 35
  Si5351a_Write_Reg (36, (MSNB_P1 & 196608) >> 10); // Bits [17:16] of MSNB_P1 in bits [7:6] of register 36  so only 10 bits to right shift
  Si5351a_Write_Reg (37, (MSNB_P1 & 65280) >> 8);   // Bits [15:8]  of MSNB_P1 in register 37
  Si5351a_Write_Reg (38, MSNB_P1 & 255);            // Bits [7:0]  of MSNB_P1 in register 38
  Si5351a_Write_Reg (39, ((MSNB_P3 & 983040) >> 12) | ((MSNB_P2 & 983040) >> 16)); // Parts of MSNB_P3 and MSNB_P1
  Si5351a_Write_Reg (40, (MSNB_P2 & 65280) >> 8);   // Bits [15:8]  of MSNB_P2 in register 40
  Si5351a_Write_Reg (41, MSNB_P2 & 255);            // Bits [7:0]  of MSNB_P2 in register 41
  Si5351a_Write_Reg (42, 0);                        // Bits [15:8] of MS0_P3 (always 0) in register 42
  Si5351a_Write_Reg (43, 1);                        // Bits [7:0]  of MS0_P3 (always 1) in register 43
  Si5351a_Write_Reg (44, ((MS0_P1 & 196608) >> 16) | TX_R);  // Bits [17:16] of MS0_P1 in bits [1:0] and R in [7:4]
  Si5351a_Write_Reg (45, (MS0_P1 & 65280) >> 8);    // Bits [15:8]  of MS0_P1 in register 45
  Si5351a_Write_Reg (46, MS0_P1 & 255);             // Bits [7:0]  of MS0_P1 in register 46
  Si5351a_Write_Reg (47, 0);                        // Bits [19:16] of MS0_P2 and MS0_P3 are always 0
  Si5351a_Write_Reg (48, 0);                        // Bits [15:8]  of MS0_P2 are always 0
  Si5351a_Write_Reg (49, 0);                        // Bits [7:0]   of MS0_P2 are always 0
  if (TX_outdivider == 4){
    Si5351a_Write_Reg (44, 12 | TX_R);              // Special settings for R = 4 (see datasheet)
    Si5351a_Write_Reg (45, 0);                      // Bits [15:8]  of MS0_P1 must be 0
    Si5351a_Write_Reg (46, 0);                      // Bits [7:0]  of MS0_P1 must be 0
  } 
  Si5351a_Write_Reg (177, 128);                     // This resets PLL B
  return MSNB_P2;
}
*/

//--------------------------------------------------------------------
// Now for the Si5351a multipurpose function
//--------------------------------------------------------------------
//
void Si5351a_Write_Reg (byte regist, byte value){   // Writes "byte" into "regist" of Si5351a via I2C
  Wire.beginTransmission(96);                       // Starts transmission as master to slave 96, which is the
                                                    // I2C address of the Si5351a (see Si5351a datasheet)
  Wire.write(regist);                               // Writes a byte containing the number of the register
  Wire.write(value);                                // Writes a byte containing the value to be written in the register
  Wire.endTransmission();                           // Sends the data and ends the transmission
}

