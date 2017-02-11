#include "Wire.h"
#include "Keypad.h"
#include "OLED_I2C.h"
#include <MsTimer2.h>

#define BUTTON_1 '1'
#define BUTTON_2 '2'
#define BUTTON_3 '3'
#define BUTTON_4 '4'

#define INTERRUPT_PIN 2
#define LED_PIN 13

#define DSP_V_POS_ACC 0
#define DSP_V_POS_GYRO 56
#define ROW_HIGH 8
#define DSP_V_DATA_OFFSET 12

#define FONT_WIDTH 6
#define FONT_HEIGHT 8
#define DISPLAY_WIDTH_PIXELS 128
#define DISPLAY_HEIGHT_PIXELS 64

#define BUTTON_1 '1'
#define BUTTON_2 '2'
#define BUTTON_3 '3'
#define BUTTON_4 '4'
// Menu constants
#define MENU_HOME 1
#define MENU_HOME_MAX 2

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

#define UPPER_RANGE 24576  // 75% of int16_t
#define BOTTOM_RANGE 16384 // 50% of int16_t

#define GYRO_MEASUREMENT_INTERVAL 4 //miliseconds

#define LED_PIN 13

char *labelsXYZ[] = { "x=", "y=", "z=" };
char *labelsYPR[] = { "pitch=", "roll=", "yaw="};

const byte ROWS = 2; //Два реда
const byte COLS = 2; //Две колони

char keys[ROWS][COLS] = {
    {'4', '3'},
    {'2', '1'}};

byte rowPins[ROWS] = {4, 5}; //пинове към които са свързани редовете на клавиатурата
byte colPins[COLS] = {6, 7}; //пинове към които са свързани колоните на клавиатурата

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

int displayInterval = 500;

OLED display(SDA, SCL, 10);
extern uint8_t SmallFont[];

//float accVector;
long rawAcc[3];
long rawGyro[3];
float realGyro[3];

unsigned long interval; //interval since previous samples
float RwAcc[3];         //projection of normalized gravitation force vector on x/y/z axis, as measured by accelerometer
float RwGyro[3];        //Rw obtained from last estimated value and gyro movement
float Awz[2];           //angles between projection of R on XZ/YZ plane and Z axis (deg)

long gyroOffsetX, gyroOffsetY, gyroOffsetZ;
float pitch, roll, yaw;

long pitchBuffer, rollBuffer, yawBuffer;
float rollAcc, pitchAcc, yawAcc;
float pitchOutpu, rollOutput;

volatile bool refreshNeeded = false;
volatile byte buttonPressed = 0;
volatile bool firstSample = true;

float wGyro;

float RwEst[3]; //Rw estimated from combining RwAcc and RwGyro
unsigned long lastMicros;
int temperature;
int menuPosition = MENU_HOME;

void interrupt()
{
  refreshNeeded = true;
}

int accRange, gyroRange;

void setup()
{
  display.begin(); // Стартиране на връзката с дисплея.
  display.setFont(SmallFont);

  Wire.begin(); //Стартиране на I2C в режим мастър

  Serial.begin(115200); //Стартиране на серийната комуникация. Скорост 38400 bps
  Serial.println("Setting some things up.");

  MsTimer2::set(100, interrupt);
  MsTimer2::start();

  pinMode(LED_PIN, OUTPUT); // LED пина конфигуриран, като изход.

  setup_mpu_6050_registers(); //Конфигуриране на mpu_6050.

  // calibrateGyro();

  wGyro = 10;

  firstSample = true;
  Serial.println("Done");
}

void loop()
{

  read_mpu_6050_data();
  getEstimatedInclination();

  // buttonPressed = keypad.getKey();
  // switch (buttonPressed)
  // {
  // case BUTTON_1:
  //   reset();
  //   break;
  // case BUTTON_3:
  //   if (menuPosition == MENU_HOME)
  //     menuPosition = MENU_HOME_MAX;
  //   else
  //     menuPosition = MENU_HOME;
  //   break;
  // }

  //  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // gx -= gyroOffsetX; //Subtract the offset calibration value from the raw gyro_x value
  // gy -= gyroOffsetY; //Subtract the offset calibration value from the raw gyro_y value
  // gz -= gyroOffsetZ; //Subtract the offset calibration value from the raw gyro_z value

  // calculateAngles();

  refresh();
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  // while (micros() - loopTimer < 1000 * GYRO_MEASUREMENT_INTERVAL)
  //   ;                   //Wait until the loopTimer reaches 4000us (250Hz) before starting the next loop
  // loopTimer = micros(); //Reset the loop timer
}

void getEstimatedInclination()
{
  static int i, w;
  static float tmpf, tmpf2;
  static unsigned long newMicros; //new timestamp
  static char signRzGyro;

  //Serial.println("Calculate inclination");

  //get raw adc readings
  newMicros = micros(); //save the time when sample is taken

  //for(i=0;i<INPUT_COUNT;i++) an[i]= analogRead(i);

  //compute interval since last sampling time
  interval = newMicros - lastMicros; //please note that overflows are ok, since for example 0x0001 - 0x00FE will be equal to 2
  lastMicros = newMicros;            //save for next loop, please note interval will be invalid in first sample but we don't use it

  //get accelerometer readings in g, gives us RwAcc vector
  for (w = 0; w <= 2; w++)
  {
    RwAcc[w] = rawToRealAcc(rawAcc[w]);
    realGyro[w] = rawToRealGyro(rawGyro[w]);
  }

  //normalize vector (convert to a vector with same direction and with length 1)
  normalize3DVector(RwAcc);

  if (firstSample)
  {
    for (w = 0; w <= 2; w++)
      RwEst[w] = RwAcc[w]; //initialize with accelerometer readings
  }
  else
  {
    //evaluate RwGyro vector
    if (abs(RwEst[2]) < 0.1)
    {
      //Rz is too small and because it is used as reference for computing Axz, Ayz it's error fluctuations will amplify leading to bad results
      //in this case skip the gyro data and just use previous estimate
      for (w = 0; w <= 2; w++)
        RwGyro[w] = RwEst[w];
    }
    else
    {
      //get angles between projection of R on ZX/ZY plane and Z axis, based on last RwEst
      for (w = 0; w <= 1; w++)
      {
        tmpf = realGyro[w] / 1000.0;                   //get current gyro rate in deg/ms
        tmpf *= interval / 1000.0f;                    //get angle change in deg
        Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD_TO_DEG; //get angle and convert to degrees
        Awz[w] += tmpf;                                //get updated angle according to gyro movement
      }

      //estimate sign of RzGyro by looking in what qudrant the angle Axz is,
      //RzGyro is pozitive if  Axz in range -90 ..90 => cos(Awz) >= 0
      signRzGyro = (cos(Awz[0] * DEG_TO_RAD) >= 0) ? 1 : -1;

      //reverse calculation of RwGyro from Awz angles, for formula deductions see  http://starlino.com/imu_guide.html
      for (w = 0; w <= 1; w++)
      {
        RwGyro[w] = sin(Awz[w] * DEG_TO_RAD);
        RwGyro[w] /= sqrt(1 + squared(cos(Awz[w] * DEG_TO_RAD)) * squared(tan(Awz[1 - w] * DEG_TO_RAD)));
      }
      RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
    }

    //combine Accelerometer and gyro readings
    for (w = 0; w <= 2; w++)
      RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

    normalize3DVector(RwEst);
  }

  firstSample = false;
}

void normalize3DVector(float *vector)
{
  static float R;
  R = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  vector[0] /= R;
  vector[1] /= R;
  vector[2] /= R;
}

float squared(float x)
{
  return x * x;
}

void read_mpu_6050_data()
{                                          //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(MPU6050_ADDRESS); //Start communicating with the MPU-6050
  Wire.write(0x3B);                        //Send the requested starting register
  Wire.endTransmission();                  //End the transmission
  Wire.requestFrom(MPU6050_ADDRESS, 14);   //Request 14 bytes from the MPU-6050
  while (Wire.available() < 14)
  {
  };                                            //Wait until all the bytes are received
  rawAcc[0] = Wire.read() << 8 | Wire.read();   //Add the low and high byte to the acc_x variable
  rawAcc[1] = Wire.read() << 8 | Wire.read();   //Add the low and high byte to the acc_y variable
  rawAcc[2] = Wire.read() << 8 | Wire.read();   //Add the low and high byte to the acc_z variable
  temperature = Wire.read() << 8 | Wire.read(); //Add the low and high byte to the temperature variable
  rawGyro[0] = Wire.read() << 8 | Wire.read();  //Add the low and high byte to the gyro_x variable
  rawGyro[1] = Wire.read() << 8 | Wire.read();  //Add the low and high byte to the gyro_y variable
  rawGyro[2] = Wire.read() << 8 | Wire.read();  //Add the low and high byte to the gyro_z variable
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

void compare(int16_t current, int16_t *maxvalue)
{
  if (abs(current) > abs(*maxvalue))
    *maxvalue = current;
}

void reset()
{
  //maxax = maxay = maxaz = maxgx = maxgy = maxgz = 0;
  accRange = 0;
  gyroRange = 0;
  setFullScaleAccRange(accRange);
  setFullScaleGyroRange(gyroRange);
  //mpu.setFullScaleAccelRange(accRange);
  //mpu.setFullScaleGyroRange(gyroRange);
}

// void checkRange()
// {
//   //Check max acceleration
//   if (abs(ax) > UPPER_RANGE || abs(ay) > UPPER_RANGE || abs(az) > UPPER_RANGE)
//   {
//     if (accRange < 3)
//     {
//       accRange++;
//       maxax = maxax / 2;
//       maxay = maxay / 2;
//       maxaz = maxaz / 2;
//       setFullScaleAccRange(accRange);
//       //mpu.setFullScaleAccelRange(accRange);
//       //reset();
//     }
//   }
//   //  else if(abs(maxax) < BOTTOM_RANGE || abs(maxay) < BOTTOM_RANGE || abs(maxaz) < BOTTOM_RANGE)
//   //  {
//   //    if(accRange > 0)
//   //    {
//   //      accRange--;
//   //      mpu.setFullScaleAccelRange(accRange);
//   //      reset();
//   //    }
//   //  }

//   //Check max gyro
//   if (abs(gx) > UPPER_RANGE || abs(gy) > UPPER_RANGE || abs(gz) > UPPER_RANGE)
//   {
//     if (gyroRange < 3)
//     {
//       gyroRange++;
//       maxgx = maxgx / 2;
//       maxgy = maxgy / 2;
//       maxgz = maxgz / 2;
//       setFullScaleGyroRange(gyroRange);
//     }
//   }
//   //  else if(abs(maxgx) < BOTTOM_RANGE || abs(maxgy) < BOTTOM_RANGE || abs(maxgz) < BOTTOM_RANGE)
//   //  {
//   //    if(gyroRange > 0)
//   //    {
//   //      gyroRange--;
//   //      mpu.setFullScaleGyroRange(accRange);
//   //      reset();
//   //    }
//   //  }
// }

int16_t int_pow(int16_t base, int16_t exp)
{
  int result = base;
  for (int i = 1; i < exp; i++)
  {
    result *= base;
  }
  return result;
}

void displayReadings()
{
  //Ред 2
  display.print("Accel", colPos(0), rowPos(2));
  display.print("Angles", colPos(9), rowPos(2));

  for (int i = 0; i < 3; i++)
  {
    display.print(labelsXYZ[i], colPos(0), rowPos(3 + i));
    if(RwAcc[i] < 0.0) display.print("-", colPos(2), rowPos(3 + i));
    display.printNumF(abs(RwAcc[i]), 2, colPos(3), rowPos(3 + i));
  }

  for (int i = 0; i < 2; i++)
  {
    display.print(labelsYPR[i], colPos(9), rowPos(3 + i));
    if(Awz[i] < 0.0) display.print("-", colPos(15), rowPos(3 + i));
    display.printNumF(abs(Awz[i]), 2, colPos(16), rowPos(3 + i));
  }
}

int rowPos(int row)
{
  return row * FONT_HEIGHT;
}

int colPos(int col)
{
  return col * FONT_WIDTH;
}

void displayButtons()
{
  display.drawRect(0, 63 - 10, 127, 63);
  display.drawLine(31, 63 - 10, 31, 63);
  display.drawLine(63, 63 - 10, 63, 63);
  display.drawLine(95, 63 - 10, 95, 63);
  display.print("Menu", 2, 63 - 8);
  display.print("Rst", 97, 63 - 8);
}

// void calibrateGyro()
// {

//   display.clrScr();
//   display.print("Calibrating gyro", 0, 0);
//   display.update();

//   for (int cal_int = 0; cal_int < 2000; cal_int++)
//   { //Run this code 2000 times
//     //if(cal_int % 125 == 0) display.print();                              //Print a dot on the LCD every 125 readings
//     read_mpu_6050_data(); //Read the raw acc and gyro data from the MPU-6050
//     gyroOffsetX += gx;    //Add the gyro x-axis offset to the gyroOffsetX variable
//     gyroOffsetY += gy;    //Add the gyro y-axis offset to the gyroOffsetY variable
//     gyroOffsetZ += gz;    //Add the gyro z-axis offset to the gyroOffsetZ variable
//     delay(3);             //Delay 3us to simulate the 250Hz program loop
//   }

//   gyroOffsetX /= 2000; //Divide the gyroOffsetX variable by 2000 to get the avarage offset
//   gyroOffsetY /= 2000; //Divide the gyroOffsetY variable by 2000 to get the avarage offset
//   gyroOffsetZ /= 2000; //Divide the gyroOffsetZ variable by 2000 to get the avarage offset
// }

void refresh()
{

  if (!refreshNeeded)
    return;

  refreshNeeded = false;

  display.clrScr();

  displayButtons();

  //  display.print("+/-", DSP_V_POS_ACC, ROW_HIGH * 5);
  //  display.printNumI(int_pow(2, (accRange + 1)), DSP_V_POS_ACC + 18, ROW_HIGH * 5, 2);
  //  display.print("+/-", colPos(9), ROW_HIGH * 5);
  //  display.printNumI(250 * (gyroRange + 1), colPos(9) + 18, ROW_HIGH * 5, 4);

  switch (menuPosition)
  {
  case MENU_HOME:
    displayReadings();
    display.print("MPU6050", CENTER, 0);
    display.print("Mode", 33, 63 - 8);
    break;
  case MENU_HOME_MAX:
    displayReadings();
    display.print("ADXL345", CENTER, 0);
    display.print("Mode", 33, 63 - 8);
    break;
  }
  display.update();
}

double rawToRealAcc(int16_t value)
{
  return (double)value / (16384 / pow(2, accRange));
}

double rawToRealGyro(int16_t value)
{
  return (double)value / (32768 / (250 * pow(2, gyroRange)));
}

double gyroReadingsToDegree(int16_t value)
{
  return rawToRealGyro(value) * 0.001 * GYRO_MEASUREMENT_INTERVAL;
}

// void calculateAngles()
// {
//   //Изчисляване на Racc.
//   accVector = sqrt((ax * ax) + (ay * ay) + (az * az));

//   //The Arduino asin function is in radians
//   pitch += gyroReadingsToDegree(gx);
//   roll += gyroReadingsToDegree(gy);

//   pitch += roll * sin(gyroReadingsToDegree(gz) * DEG_TO_RAD); //If the IMU has yawed transfer the roll angle to the pitch angel
//   roll -= pitch * sin(gyroReadingsToDegree(gz) * DEG_TO_RAD);

//   pitchAcc = asin((float)ax / accVector) * RAD_TO_DEG;     //Calculate the pitch angle
//   rollAcc = -1 * asin((float)ay / accVector) * RAD_TO_DEG; //Calculate the roll angle

//   if (firstSample)
//   {
//     pitch = pitch * 0.9995 + pitchAcc * 0.0005;
//     roll = roll * 0.9995 + rollAcc * 0.0005;
//   }
//   else
//   {
//     pitch = pitchAcc;
//     roll = rollAcc;
//     firstSample = true;
//   }
// }
