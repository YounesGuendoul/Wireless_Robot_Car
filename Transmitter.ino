#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define PRINT_DEBUG   

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
bool dmpReady = false;  
uint8_t devStatus;      
uint16_t packetSize;    
uint8_t fifoBuffer[64];
Quaternion q;           
VectorFloat gravity;  
float ypr[3];           


const uint64_t pipeOut = 0xF9E8F0F0E1LL; 
RF24 radio(8, 9);

struct PacketData 
{
  byte xAxisValue;
  byte yAxisValue;
} data;

void setupRadioTransmitter()
{
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.openWritingPipe(pipeOut);
  radio.stopListening();

  data.xAxisValue = 127; // Center
  data.yAxisValue = 127; // Center 
}


void setupMPU()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); 
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  #ifdef PRINT_DEBUG
    Serial.begin(115200);
    while (!Serial); 
  #endif
  
  mpu.initialize();

  #ifdef PRINT_DEBUG  
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); 
    while (!Serial.available());                 
    while (Serial.available() && Serial.read());
    Serial.println(F("Initializing DMP..."));
  #endif
  
  devStatus = mpu.dmpInitialize();

  if (devStatus == 0) 
  {
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);      
      mpu.PrintActiveOffsets();
      mpu.setDMPEnabled(true);

      dmpReady = true;
  
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else 
  {
      #ifdef PRINT_DEBUG       
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      #endif
  }
}


void setup()
{
  setupRadioTransmitter();   
  setupMPU();
}

void loop() 
{
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) 
  {  
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    int xAxisValue = constrain(ypr[2] * 180/M_PI, -90, 90);
    int yAxisValue = constrain(ypr[1] * 180/M_PI, -90, 90);
    data.xAxisValue = map(xAxisValue, -90, 90, 0, 254); 
    data.yAxisValue = map(yAxisValue, -90, 90, 254, 0);

    radio.write(&data, sizeof(PacketData));

    #ifdef PRINT_DEBUG  
      Serial.println(xAxisValue);  
      Serial.println(yAxisValue);        
    #endif
  }
}
