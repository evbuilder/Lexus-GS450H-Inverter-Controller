/* Project to get Prius Inverter/Converter going
 * 
 * Derived from GS450H V7 code:
 * Copyright 2019 T.Darby , D.Maguire
 * openinverter.org
 * evbmw.com
 * 
 */

#include <Metro.h>
//#include <due_wire.h> //https://github.com/collin80/due_wire
#include <DueTimer.h>  //https://github.com/collin80/DueTimer
//#include <Wire_EEPROM.h> //https://github.com/collin80/Wire_EEPROM
//#include <ISA.h>  //isa can shunt library

#define pin_inv_req 22

#define PARK 0
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE 3

#define OilPumpPower  33
#define OilPumpPWM  2
#define InvPower    34
#define Out1  50
#define TransSL1  47
#define TransSL2  44
#define TransSP   45

#define IN1   6
#define IN2   7
#define Brake_In   62

#define TransPB1    40
#define TransPB2    43
#define TransPB3    42

#define OilpumpTemp A7
#define TransTemp A4
#define MG1Temp A5
#define MG2Temp A6

////////////////Global variables ////////////////////////////////////////////////////////////
byte gear;
unsigned short ThrottleCountdown = 1000;
////////////////////////////////////////////////////////////////////////////////////////////////////

/*
byte get_gear()
{
  if(!digitalRead(IN1))
  {
  return(DRIVE);
  }
  else
  {
  return(REVERSE); 
  }
}
*/

Metro timer_htm = Metro(10); 
Metro timer_diag = Metro(1100);

byte mth_data[120];
byte htm_data_setup[100]={0,   30,  0,   0,   0,   0,   0,   55,  0,   126,
                          255, 0,   0,   255, 0,   97,  4,   0,   0,   0,
                          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                          0,   0,   49,  0,   0,   0,   0,   0,   255, 0,
                          3,   4,   0,   0,   0,   0,   0,   0,   0,   0,
                          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                          0,   0,   0,   0,   0,   0,   0,   4,   0,   0,
                          0,   0,   255, 4,   61,  25,  60,  246, 52,  8,
                          0,   0,   255, 0,   0,   0,   137, 0, /*226*/59, 0,
                          8,   0,   0,   0,   27,  23,  0,   0, /*220*/53, 9
};

 byte htm_data[100]={     0,   30,  0,  2,    0,   0,   0,   55,  0,   126,
                          255, 0,   0,   31,  0,   97, 0,    0,   0,   0,
                          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                          0,   0,   49,  0,   0,   0,   0,   0,   31,  0,
                          3,  128,  0,   0,   0,   0,   0,   0,   0,   0,
                          0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
                          0,   0,   0,   0,   0,   0,   0,   4,   0,   0,
                          0,   0,   31, 0,   2,    33,  60,  246, 52,  8,
                          0,   0,   31,  0,   0,   0,   137, 0,   0,   0,
                          8,   0,   0,   0,   38,  30,  0,   0,   0,   0 };
/* creep in reverse
0   30  0   2   0   0  0   55  0   126
255 0   0   40  0   97 0   0   0   0
0   0   248 254 8   1  0   0   0   0
0   0   49  0   0   0  0   0   40  0
3   128 0   0   0   0  0   0   0   0 
0   0   0   0   0   0  0   0   0   0 
0   0   0   0   0   0  0   4   0   0
0   0   40  0   2   33 59  246 241 7
0   0   40  0   0   0  201 0   224 0
16  0   0   0   151 40 0   0   0   0*/

unsigned short since_last_packet=4000;

unsigned long  last_packet=0;

float dc_bus_voltage=0,temp_inv_water=0,temp_inv_inductor=0; //just used for diagnostic output

short mg1_torque=0,
      mg2_torque=0,
      mg1_speed=-1,
      mg2_speed=-1;
       
byte inv_status=1;
     //gear=get_gear(); //get this from your transmission
     
bool htm_sent=0, 
     mth_good=0;

int oil_power=135; //oil pump pwm value

int Throt1Pin = A0; //throttle pedal analog inputs
int Throt2Pin = A1;
int ThrotVal=0; //value read from throttle pedal analog input

short get_torque()
{
  //accelerator pedal mapping to torque values here
  ThrotVal=analogRead(Throt1Pin); //75 to 370
  if (ThrotVal<80) ThrotVal=75;//dead zone at start of throttle travel
 if(gear==DRIVE) ThrotVal = map(ThrotVal, 75, 370, 0, 3500);
 if(gear==REVERSE) ThrotVal = map(ThrotVal, 75, 370, 0, -3500);
 if(gear==PARK) ThrotVal = 0;  //no torque in park or neutral
 if(gear==NEUTRAL) ThrotVal = 0;  //no torque in park or neutral
   return ThrotVal; //return torque
}

#define SerialDEBUG SerialUSB

void setup() {
    gear=NEUTRAL;
//////////////////////////////////////////////   
  pinMode(pin_inv_req, OUTPUT);
  digitalWrite(pin_inv_req, 1);
  pinMode(13, OUTPUT);  //led
  pinMode(OilPumpPower, OUTPUT);  //Oil pump control relay- being used to control HV precharge contactor as not required seperate oil pump power control.
  digitalWrite(OilPumpPower,LOW);  //turn off precharge
   //analogWrite(OilPumpPWM,125);  //set 50% pwm to oil pump at 1khz for testing

  pinMode(InvPower, OUTPUT);  //Inverter Relay 
  pinMode(Out1, OUTPUT);  //GP output one
  pinMode(TransSL1,OUTPUT); //Trans solenoids
  pinMode(TransSL2,OUTPUT); //Trans solenoids
  pinMode(TransSP,OUTPUT); //Trans solenoids

  digitalWrite(InvPower,LOW);  //turn off at startup
  digitalWrite(Out1,LOW);  //turn off HV main at startup
  digitalWrite(TransSL1,LOW);  //turn off at startup
  digitalWrite(TransSL2,LOW);  //turn off at startup
  digitalWrite(TransSP,LOW);  //turn off at startup
  pinMode(19,INPUT); // EHC had to do this to make receive MTH work

  pinMode(IN1,INPUT); //Input 1
  pinMode(IN2,INPUT); //Input 2
  pinMode(Brake_In,INPUT); //Brake pedal input

  pinMode(TransPB1,INPUT); //Trans inputs
  pinMode(TransPB2,INPUT); //Trans inputs
  pinMode(TransPB3,INPUT); //Trans inputs

  Serial1.begin(250000); // 500k comes out. Not sure how USCLKS affects it as this should div8

  PIOA->PIO_ABSR |= 1<<17;
  PIOA->PIO_PDR |= 1<<17;
  USART0->US_MR |= 1<<4 | 1<<8 | 1<<18;   //  4:USCLKS | 8:SYNC | 18:CLKO(utput)
//  PIOA->PIO_PDR = 1<<10; //why is this still enabled?

  htm_data[76]=(-5000)&0xFF; //was63 // regen ability of battery
  htm_data[77]=((-5000)>>8); //was64

  htm_data[78]=(27500)&0xFF; //was65  // discharge ability of battery
  htm_data[79]=((27500)>>8); //was66
 
  SerialDEBUG.begin(115200);
  SerialDEBUG.print("hello world!");
}

void control_inverter()
{
  if(timer_htm.check()) //prepare htm data
  {
    if(mth_good)
    {
      dc_bus_voltage=(((mth_data[100]|mth_data[101]<<8)-5)/2); //was82,83
      temp_inv_water=(mth_data[105]);
      temp_inv_inductor=(mth_data[104]);
      mg1_speed=mth_data[10]|mth_data[11]<<8; //was 6,7
      mg2_speed=mth_data[38]|mth_data[39]<<8; //was31,32
    }
    //gear=get_gear();
    if(ThrottleCountdown!=0)
    {
      ThrottleCountdown--;
      mg2_torque = 0;
    }
    else
    {
      short newTorque;  //EHC: reduce torque with speed to emulate auto "creep"
      if(mg2_speed > 1000)   { newTorque = 0;                 }
      else if(mg2_speed < 0) { newTorque = 200;               }
      else                   { newTorque = 200 - mg2_speed/5; }
         // Limit rate of increase in torque to 1 per millisecond (0.2s to get to 200)
      mg2_torque = newTorque < (mg2_torque + 5) ? newTorque : mg2_torque + 5;
    }
    mg1_torque=0;

    //speed feedback
    int speedSum = mg2_speed + mg1_speed;
    speedSum/=113;
//    speedSum = (mg1_speed + mg2_speed //Engine speed = (MG1+MG2*143/145)*5/18
    htm_data[88]=dc_bus_voltage; // EHC - tell it the BMS says the volts are good
    htm_data[0]=(byte)speedSum;
    htm_data[91]=(mg1_torque*4)&0xFF; //was75
    htm_data[92]=((mg1_torque*4)>>8); //was76
    
    //mg1
    htm_data[5]=(mg1_torque*-1)&0xFF;  //was5 //negative is forward
    htm_data[6]=((mg1_torque*-1)>>8);  //was6
    htm_data[11]=htm_data[5];          //was11
    htm_data[12]=htm_data[6];          //was12

    //mg2
    htm_data[36]=(mg2_torque)&0xFF; //was26//positive is forward
    htm_data[37]=((mg2_torque)>>8); //was27
    htm_data[30]=htm_data[36];      //was32
    htm_data[31]=htm_data[37];      //was33

    //checksum
    unsigned short htm_checksum=0;
    for(byte i=0;i<98;i++) { htm_checksum+=htm_data[i]; }
    htm_data[98]=htm_checksum&0xFF;
    htm_data[99]=htm_checksum>>8;
  }
  
  since_last_packet=micros()-last_packet;

  if(since_last_packet>=4000) //read mth
  {    
    htm_sent=0;
    byte mth_byte;
    unsigned short mth_checksum = 0, bytes_received = 0;
    
    for(mth_byte=0; mth_byte<120; mth_byte++)
    {
      if(Serial1.available())
      {
        mth_data[mth_byte]=Serial1.read();
        bytes_received++;
      }
      else break;
      if(mth_byte<118) mth_checksum += mth_data[mth_byte];
    }
    while(Serial1.available()){  Serial1.read(); bytes_received++;} // drop any remaining bytes in serial buffer
    while(mth_byte<120){ mth_data[mth_byte++]=0; } // zero remiander of mth array
    
    if( (mth_checksum==(mth_data[118]|(mth_data[119]<<8))) && (bytes_received==120) && (mth_checksum!=0) )
      mth_good=1;else mth_good=0;
    last_packet=micros();
    digitalWrite(pin_inv_req,0);
  }

  since_last_packet=micros()-last_packet;
  
  if(since_last_packet>=10)digitalWrite(pin_inv_req,1);

  if(since_last_packet>=1000 && !htm_sent)
  {
    if(inv_status==0)
    {
      for(int i=0;i<100;i++)
        Serial1.write(htm_data[i]);
    }
    else
    {
      for(int i=0;i<100;i++)
        Serial1.write(htm_data_setup[i]);
      if(mth_data[1]!=0)
        inv_status--;
    }
    htm_sent=1;
  }
}

void diag_mth()
{
  ///mask just hides any MTH data byte which is represented here with a 0. Useful for debug/discovering.
  bool mth_mask[120] = {
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1,};
    
  SerialDEBUG.print("\n");
  SerialDEBUG.println("\t0\t1\t2\t3\t4\t5\t6\t7\t8\t9");
  SerialDEBUG.print(PIOA->PIO_PSR,HEX);
  SerialDEBUG.print(" ");
  SerialDEBUG.print(PIOB->PIO_PSR,HEX);
  SerialDEBUG.print("\n");
  SerialDEBUG.print(PIOA->PIO_ABSR,HEX);
  SerialDEBUG.print(" ");
  SerialDEBUG.print(PIOB->PIO_ABSR,HEX);
  SerialDEBUG.print("\n");
  SerialDEBUG.println("   ------------------------------------------------------------------------------");  
  for (int j=0;j<12;j++)
  {
    if(j<10)SerialDEBUG.print(" ");
    if(j==0)SerialDEBUG.print(" ");
    SerialDEBUG.print(j*10);
    SerialDEBUG.print(" |\t");
    for (int k=0;k<10;k++)
    {
      if(mth_mask[j*10+k])SerialDEBUG.print(mth_data[j*10+k]);else SerialDEBUG.print (" ");
      SerialDEBUG.print("\t");
    }
    SerialDEBUG.print("\n");
  }
  SerialDEBUG.print("\nMTH Valid: ");if(mth_good)SerialDEBUG.print("Yes"); else SerialDEBUG.print("No");
  SerialDEBUG.print("\nDC Bus: ");if(dc_bus_voltage>=0)SerialDEBUG.print(dc_bus_voltage);else SerialDEBUG.print("----");
  SerialDEBUG.print("v\n");
 
  SerialDEBUG.print("MG1 - Speed: ");SerialDEBUG.print(mg1_speed);
  SerialDEBUG.print("rpm\tPosition: ");SerialDEBUG.print(mth_data[16]|mth_data[17]<<8);
  SerialDEBUG.print("\n");
  
  SerialDEBUG.print("MG2 - Speed: ");SerialDEBUG.print(mg2_speed);
  SerialDEBUG.print("rpm\tPosition: ");SerialDEBUG.print(mth_data[46]|mth_data[47]<<8);
  SerialDEBUG.print("\n");
  
  SerialDEBUG.print("Engine Speed:");SerialDEBUG.print(((int32_t)mg1_speed*18204 + (int32_t)mg2_speed*17953)>>16);
  SerialDEBUG.print("\n");
  
  SerialDEBUG.print("Water Temp:\t");SerialDEBUG.print(temp_inv_water);
  SerialDEBUG.print("c\nInductor Temp:\t" );SerialDEBUG.print(temp_inv_inductor);
  SerialDEBUG.print("c\nAnother Temp:\t");SerialDEBUG.print(mth_data[88]|mth_data[89]<<8);
  SerialDEBUG.print("c\nAnother Temp:\t");SerialDEBUG.print(mth_data[41]|mth_data[40]<<8);
  SerialDEBUG.print("c\n");
  
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
  SerialDEBUG.print("\n");
}

void loop()
{
  control_inverter();
  if(timer_diag.check())
  {
    diag_mth();
  }
}
