#define Rx 0
#define Tx 1

#define Buzzer 4
#define Cal_Button 2

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];  


volatile bool Cal_MPU_Again = false;
void Calibrate_MPU()
{
  Cal_MPU_Again=true;
}


void setup() {
  pinMode(Buzzer, OUTPUT);
  pinMode(Cal_Button,INPUT);
  digitalWrite(Buzzer,LOW);
  
  attachInterrupt(digitalPinToInterrupt(Cal_Button), Calibrate_MPU, FALLING);

  Wire.begin();
  Wire.setClock(100000); // 100kHz I2C clock. Comment on this line if having compilation difficulties

  Serial.begin(9600); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  mpu.initialize();
  

  /*Verify connection*/
  if(mpu.testConnection() == false)
  {
    //MPU6050 connection failed
    while(true);
  }
  else 
  {
    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(100);
  }

  /* Initializate and configure the DMP*/
  devStatus = mpu.dmpInitialize();

  /* Supply your gyro offsets here, scaled for min sensitivity */
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
     //Turning ON DMP
    mpu.setDMPEnabled(true);

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison

    digitalWrite(Buzzer,HIGH);
    delay(100);
    digitalWrite(Buzzer,LOW);
    delay(100);
  } 
  else 
  {
    //DMP Initialization failed
   
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!DMPReady) return; // Stop the program if DMP programming fails.

  
   /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) // Get the Latest packet 
  {
     #ifdef OUTPUT_READABLE_YAWPITCHROLL
      /* Display Euler angles in degrees */
      mpu.dmpGetQuaternion(&q, FIFOBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      short pitch = ypr[1]* 180/M_PI;
      short roll  = ypr[2]* 180/M_PI;

      Serial.write(0x3c);//'<'
      Serial.write(0x70);//'p'
      Serial.write(0x3d);//'='
      //Serial.print(pitch);
      Serial.write(highByte(pitch));
      Serial.write(lowByte(pitch));
      Serial.write(0x2c);//','
      Serial.write(0x72);//'r'
      Serial.write(0x3d);//'='
      //Serial.print(roll);
      Serial.write(highByte(roll));
      Serial.write(lowByte(roll));
      Serial.write(0x3e);//'>
      Serial.write(0x0d);//'\r'
      Serial.write(0x0a);//'\n'

    #endif
  }

  if(Cal_MPU_Again)
  {
    mpu.CalibrateAccel(6);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);

    delay(2500);
    digitalWrite(Buzzer,HIGH);
    delay(500);
    digitalWrite(Buzzer,LOW);
    
    Cal_MPU_Again = false;
  }

  delay(50);
    
}
