#include <SPI.h>

/* Serial rates for UART */
#define BAUDRATE        115200

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* SPI pins */
#define ENC_0            2
#define ENC_1            3
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52

#define MAX            16384.0
#define motor_A 11
#define motor_B 10

double u=0;
int Vm=0;

int prev_time=0;
int curr_time=0;
int dt=1;
double dtheta=0,dalpha=0;

//const double K[4] = {-2.2361, 90.8011, -2.9016, 12.1888}; Didnt work as well. balanced for a bit then died
//const double K[4] = {-1.9047, 123.8395, -3.1640, 18.5507}; worked well but does not fit the constraints well, rotates by large angles
//const double K[4] = {-2.4495, 127.5636, -3.3574, 18.9796}; worked better than prev and doesnt rotate as much
//const double K[4] = {-4.4721, 142.4809, -4.0785, 20.5822}; best run as of now, still not within constraints
//const double K[4] = {-7.0711, 157.9260, -4.9568, 22.5243}; best run as of now, still not within constraints
//const double K[4] = {-10, 174.5739, -5.9164, 24.6447}; Reasonable accurate results, not within constraints
//const double K[4] = {-10, 222.7088,  -7.5632, 30.8811};
//const double K[4] = {-10, 151.9896, -5.1143, 21.7746};
//const double K[4] = {-22.3607, 220.6405, -9.1089, 30.6108}; worse 

//const double K[4] = {-31.6228, 299.0244, -12.9488, 40.9459}; Q = diag([1000, 2000, 10, 100]), R = 1, offset (direct) = 4, best run with [WC spread = 75, Stable spread = 40] video1 3rd sept, 2:29

//const double K[4] = {-34.6410, 342.4425, -14.7675, 46.7231};  
//Q = diag([1200, 2000, 20, 100]), R = 1, offset (direct) = 4, best run with [WC spread = 60, Stable spread = 40] video2, video3(alpha)3rd sept, 2:35

const double K[4] = {-34.6410, 342.4425, -14.7675, 46.7231};  
//Q = diag([1200, 2000, 20, 100]), R = 1, offset (max) = 50, best run with [WC spread = 20, Stable spread = 15] video4 (alpha)3rd sept, 3:02 (best run)

double offset = 50;
double theta_prev=0.0;
double alpha_prev=0.0;
double theta=0.0;
double alpha=0.0;
double theta_r=0.0;
double alpha_r=0.0;



long i=0;

void setup()
{
  //Set the modes for the SPI IO
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_0, OUTPUT);
  pinMode(ENC_1, OUTPUT);
  pinMode(motor_A,OUTPUT);
  pinMode(motor_B,OUTPUT);
 
  //Initialize the UART serial connection for debugging
  Serial.begin(BAUDRATE);

  //Get the CS line high which is the default inactive state
  digitalWrite(ENC_0, HIGH);
  digitalWrite(ENC_1, HIGH);

  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz

 
  //start SPI bus
  SPI.begin();
}

void loop()
{
  i++;
  curr_time=millis();
  int en0 = getPositionSPI(ENC_0, RES14);
  int en1 = getPositionSPI(ENC_1, RES14);
  theta = (en0/MAX)*360;
  //theta = en0;
  alpha = (en1/MAX)*360 - 180;
  //alpha = en1;
  if(theta>180.0)
  {
    theta-=360;
  }
//  if(alpha>180.0)
//  {
//    alpha-=360;
//  }
  theta_r=theta*PI/180.0;
  alpha_r=alpha*PI/180.0;

  dtheta=(theta_r-theta_prev)/dt*1000.0;
  dalpha=(alpha_r-alpha_prev)/dt*1000.0;
  theta_prev=theta_r;
  alpha_prev=alpha_r;
    
   
  u=(K[0]*theta+K[1]*alpha+K[2]*dtheta+K[3]*dalpha);
  if(u>0)
  {
    Vm=map(u,0,12,0,255);
    Vm = max(Vm, offset);
    analogWrite(motor_A,0);
    analogWrite(motor_B,constrain(Vm,0,255));
  }
  else
  {
    Vm=map(-u,0,12,0,255);
    Vm = max(Vm, offset);
    analogWrite(motor_A,constrain(Vm,0,255));
    analogWrite(motor_B,0);
  }

  if(i%100==0)
  {
    //Serial.print("Theta is: ");
    Serial.print((theta_r*180)/PI);

    //Serial.print("   Theta_d is: ");
    //Serial.print(dtheta);

    //Serial.print("  alpha is: ");
    Serial.print(" ");
    Serial.println((alpha_r*180)/PI);

    //Serial.print("  alpha_d is: ");
    //Serial.print(dalpha);

    //Serial.print("  u:");

    //Serial.print(u/abs(u)*(abs(u)+4));

//    Serial.print("  Vm:");
//    Serial.println(constrain(Vm,0,255));
//
//    Serial.println("-------------------------------------");
}

  delay(1);
}

uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;  

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

/*
 * This function does the SPI transfer. sendByte is the byte to transmit.
 * Use releaseLine to let the spiWriteRead function know if it should release
 * the chip select line after transfer.  
 * This function takes the pin number of the desired device as an input
 * The received data is returned.
 */
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
 
  return data;
}

/*
 * This function sets the state of the SPI line. It isn't necessary but makes the code more readable than having digitalWrite everywhere
 * This function takes the pin number of the desired device as an input
 */
void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void setZeroSPI(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);
 
  spiWriteRead(AMT22_ZERO, encoder, true);
  delay(250); //250 second delay to allow the encoder to reset
}

/*
 * The AMT22 bus allows for extended commands. The first byte is 0x00 like a normal position transfer, but the
 * second byte is the command.  
 * This function takes the pin number of the desired device as an input
 */
void resetAMT22(uint8_t encoder)
{
  spiWriteRead(AMT22_NOP, encoder, false);

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);
 
  spiWriteRead(AMT22_RESET, encoder, true);
 
  delay(250); //250 second delay to allow the encoder to start back up
}
