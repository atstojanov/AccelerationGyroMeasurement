/* 
IMU Calculations Demo, by Starlino
IMU Guide: http://starlino.com/imu_guide.html
Acc_Gyro Board:  http://www.gadgetgangster.com/213

Hardware Setup:

Acc_Gyro <--->  Arduino
5V       <--->  5V  
GND      <--->  GND
AX       <--->  AN0
AY       <--->  AN1
AZ       <--->  AN2
GX4      <--->  AN3  
GY4      <--->  AN4    

*/
#include "Wire.h"

#define LED_PIN 13
#define MPU6050_ADDRESS 0x68

#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C

#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_ACCEL_FS_8 0x02
#define MPU6050_ACCEL_FS_16 0x03
// #define INPUT_COUNT 5     //number of analog inputs
// #define VDD 5000.0f       //Analog reference voltage in milivolts
// #define PI 3.14159265358979f

// int an[INPUT_COUNT];      //analog inputs  
char firstSample;	  //marks first sample
float wGyro;
// struct {					
//   char inpInvert[INPUT_COUNT];    // bits 0..5 invert input
//   int zeroLevel[INPUT_COUNT];     // 0..2 accelerometer zero level (mV) @ 0 G
//                                   // 3..5 gyro zero level (mV) @ 0 deg/s
//   int inpSens[INPUT_COUNT];       // 0..2 acceleromter input sensitivity (mv/g)
//                                   // 3..5 gyro input sensitivity (mV/deg/ms) 
//   float wGyro;		          // gyro weight/smooting factor
// } config;	

//Notation "w" stands for one of the axes, so for example RwAcc[0],RwAcc[1],RwAcc[2] means RxAcc,RyAcc,RzAcc
//Variables below must be global (their previous value is used in getEstimatedInclination)

float RwEst[3];     //Rw estimated from combining RwAcc and RwGyro
unsigned long lastMicros;  

//Variables below don't need to be global but we expose them for debug purposes

float accVector;
long ax, ay, az, maxax, maxay, maxaz;
float accX, accY, accZ;
long gx, gy, gz, maxgx, maxgy, maxgz;
float gyroX, gyroY, gyroZ;

long temperature;
long gyroOffsetX, gyroOffsetY, gyroOffsetZ;
long loopTimer;
float pitch, roll, yaw;

float angleXZgyro = 0;
float angleYZgyro = 0;
float angleXYgyro = 0;

long pitchBuffer, rollBuffer, yawBuffer;
float rollAcc, pitchAcc, yawAcc;
float pitchOutpu, rollOutput;

unsigned long interval; //interval since previous samples
float RwAcc[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float rGyro[3];         //gyroReadings in deg/s.
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)

int accRange = 0;
int gyroRange = 0;

void setup() {
  static int i;
  
  Wire.begin();

  Serial.begin(38400); 

  pinMode(LED_PIN, OUTPUT); // LED пина конфигуриран, като изход.

  setup_mpu_6050_registers(); //Конфигуриране на mpu_6050.

  // //Setup parameters for Acc_Gyro board, see http://www.gadgetgangster.com/213
  // for(i=0;i<=2;i++){                  // X,Y,Z axis
  //   config.zeroLevel[i] = 1650;       // Accelerometer zero level (mV) @ 0 G
  //   config.inpSens[i] = 478;          // Accelerometer Sensisitivity mV/g
  // }        
  
  // for(i=3;i<=4;i++){
  //   config.inpSens[i] = 2000;	    // Gyro Sensitivity mV/deg/ms    
  //   config.zeroLevel[i] = 1230;     // Gyro Zero Level (mV) @ 0 deg/s  
  // }

  // config.inpInvert[0] = 1;  //Acc X
  // config.inpInvert[1] = 1;  //Acc Y
  // config.inpInvert[2] = 1;  //Acc Z

  // //Gyro readings are inverted according to accelerometer coordonate system
  // //see http://starlino.com/imu_guide.html for discussion
  // //also see http://www.gadgetgangster.com/213 for graphical diagrams
  // config.inpInvert[3] = 1;  //Gyro X  
  // config.inpInvert[4] = 1;  //Gyro Y

  // config.wGyro = 10;
    wGyro = 10;
   firstSample = 1;
}

void loop() {
  read_mpu_6050_data();
  getEstimatedInclination();
  //!!! Please note that printing more data will increase interval between samples. Try to keep it under 10ms (10,0000 us)   
  Serial.print(interval);  //microseconds since last sample, monitor this value to be < 10000, increase bitrate to print faster
  Serial.print(F(","));    
  Serial.print(RwAcc[0]);  //Inclination X axis (as measured by accelerometer)
  Serial.print(F(","));
  Serial.print(RwEst[0]);  //Inclination X axis (estimated / filtered)
  Serial.print(F(","));    
  Serial.print(RwAcc[1]);  //Inclination Y axis (as measured by accelerometer)
  Serial.print(F(","));
  Serial.print(RwEst[1]);  //Inclination Y axis (estimated / filtered)
  Serial.print(F(","));   
  Serial.print(RwAcc[2]);  //Inclination Z axis (as measured by accelerometer)
  Serial.print(F(","));
  Serial.print(RwEst[2]);  //Inclination Z axis (estimated / filtered)  
  Serial.println(F(""));

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void getEstimatedInclination(){
  static int i,w;
  static float tmpf,tmpf2;  
  static unsigned long newMicros; //new timestamp
  static char signRzGyro;  

  //Serial.println("Calculate inclination");

  //get raw adc readings
  newMicros = micros();       //save the time when sample is taken
  
  //for(i=0;i<INPUT_COUNT;i++) an[i]= analogRead(i);
  
  //compute interval since last sampling time 
  interval = newMicros - lastMicros;    //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2 
  lastMicros = newMicros;               //save for next loop, please note interval will be invalid in first sample but we don't use it
  
  //get accelerometer readings in g, gives us RwAcc vector
  // for(w=0;w<=2;w++) RwAcc[w] = getInput(w);
  
  RwAcc[0] = rawToRealAcc(ax);
  RwAcc[1] = rawToRealAcc(ay);
  RwAcc[2] = rawToRealAcc(az);
  
  rGyro[0] = rawToRealGyro(gx);
  rGyro[1] = rawToRealGyro(gy);
  rGyro[2] = rawToRealGyro(gz);

  //normalize vector (convert to a vector with same direction and with length 1)
  normalize3DVector(RwAcc);
  
  if (firstSample){
    for(w=0;w<=2;w++) RwEst[w] = RwAcc[w];    //initialize with accelerometer readings
  }else{
    //evaluate RwGyro vector
    if(abs(RwEst[2]) < 0.1){
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for(w=0;w<=2;w++) RwGyro[w] = RwEst[w];
    }else{
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
      for(w=0;w<=1;w++){
        tmpf = rGyro[w] / 1000.0;                         //get current gyro rate in deg/ms
        tmpf *= interval / 1000.0f;                     //get angle change in deg
        Awz[w] = atan2(RwEst[w],RwEst[2]) * 180 / PI;   //get angle and convert to degrees        
        Awz[w] += tmpf;                                 //get updated angle according to gyro movement
      }
      
      //estimate sign of RzGyro by looking in what qudrant the angle Axz is, 
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
      signRzGyro = ( cos(Awz[0] * PI / 180) >=0 ) ? 1 : -1;
      
      //reverse calculation of RwGyro from Awz angles, for formula deductions see  http://starlino.com/imu_guide.html
      for(w=0;w<=1;w++){
        RwGyro[w] = sin(Awz[w] * PI / 180);
        RwGyro[w] /= sqrt( 1 + squared(cos(Awz[w] * PI / 180)) * squared(tan(Awz[1-w] * PI / 180)) );
      }
      RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
    }
    
    //combine Accelerometer and gyro readings
    for(w=0;w<=2;w++) RwEst[w] = (RwAcc[w] + wGyro* RwGyro[w]) / (1 + wGyro);

    normalize3DVector(RwEst);  
   
  }
  
  firstSample = 0;
}

void normalize3DVector(float* vector){
  static float R;  
  R = sqrt(vector[0]*vector[0] + vector[1]*vector[1] + vector[2]*vector[2]);
  vector[0] /= R;
  vector[1] /= R;  
  vector[2] /= R;  
}

float squared(float x){
  return x*x;
}

//Convert ADC value for to physical units see http://starlino.com/imu_guide.html for explanation.
//For accelerometer it will return  g  (acceleration) , applies when  i = 0..2
//For gyro it will return  deg/ms (rate of rotation)  , applies when i = 3..5
// float getInput(char i){
//   static float tmpf;	        //temporary variable
//   tmpf = an[i] * VDD / 1023.0f;  //voltage (mV)
//   tmpf -= config.zeroLevel[i];  //voltage relative to zero level (mV)
//   tmpf /= config.inpSens[i];    //input sensitivity in mV/G(acc) or mV/deg/ms(gyro)
//   tmpf *= config.inpInvert[i];  //invert axis value according to configuration 
//   return tmpf;		
// }


void read_mpu_6050_data()
{
  //Serial.println("Reading data from 6050.");
                                            //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(MPU6050_ADDRESS); //Start communicating with the MPU-6050
  Wire.write(0x3B);                        //Send the requested starting register
  Wire.endTransmission();                  //End the transmission
  Wire.requestFrom(MPU6050_ADDRESS, 14);   //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14)
  {
  };                                            //Wait until all the bytes are received
  ax = Wire.read() << 8 | Wire.read();          //Add the low and high byte to the acc_x variable
  ay = Wire.read() << 8 | Wire.read();          //Add the low and high byte to the acc_y variable
  az = Wire.read() << 8 | Wire.read();          //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the temperature variable
  gx = Wire.read() << 8 | Wire.read();          //Add the low and high byte to the gyro_x variable
  gy = Wire.read() << 8 | Wire.read();          //Add the low and high byte to the gyro_y variable
  gz = Wire.read() << 8 | Wire.read();          //Add the low and high byte to the gyro_z variable
}

void setFullScaleAccRange(uint8_t range)
{
  Wire.beginTransmission(MPU6050_ADDRESS); //Start communicating with the MPU-6050
  Wire.write(0x1C);                        //Send the requested starting register
  Wire.write(range << 3);                  //Set the requested starting register
  Wire.endTransmission();                  //End the transmission
}

void setFullScaleGyroRange(uint8_t range)
{
  Wire.beginTransmission(MPU6050_ADDRESS); //Start communicating with the MPU-6050
  Wire.write(0x1B);                        //Send the requested starting register
  Wire.write(range << 3);                  //Set the requested starting register
  Wire.endTransmission();                  //End the transmission
}

void setup_mpu_6050_registers()
{
  //Activate the MPU-6050
  Wire.beginTransmission(MPU6050_ADDRESS); //Start communicating with the MPU-6050
  Wire.write(0x6B);                        //Send the requested starting register
  Wire.write(0x01);                        //Set the requested starting register
  Wire.endTransmission();                  //End the transmission
  setFullScaleAccRange(0);
  setFullScaleGyroRange(0);
}

double rawToRealAcc(int16_t value)
{
  return (double)value / (16384 / pow(2, accRange));
}

double rawToRealGyro(int16_t value)
{
  return (double)value / (32768 / (250 * pow(2, gyroRange)));
}