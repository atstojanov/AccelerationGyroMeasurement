#include "Wire.h"
#include "Keypad.h"
#include "OLED_I2C.h"
#include <MsTimer2.h>

#define BUTTON_1 '1'
#define BUTTON_2 '2'
#define BUTTON_3 '3'
#define BUTTON_4 '4'

//
#define LED_PIN 13

const byte ROWS = 2; //Два реда
const byte COLS = 2; //Две колони

char keys[ROWS][COLS] = {
  {'4', '3'},
  {'2', '1'}
};

byte rowPins[ROWS] = {4, 5};                                                          //пинове към които са свързани редовете на клавиатурата
byte colPins[COLS] = {6, 7};                                                          //пинове към които са свързани колоните на клавиатурата

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

int displayInterval = 500;

OLED display(SDA, SCL, 10);
extern uint8_t SmallFont[];

int16_t ax, ay, az, maxax, maxay, maxaz;
int16_t gx, gy, gz, maxgx, maxgy, maxgz;
int16_t temperature;
int16_t gyro_x_cal, gyro_y_cal, gyro_z_cal;
int16_t loop_timer;
int16_t lcd_loop_counter;
float angle_pitch, angle_roll;
int16_t angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

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

#define MPU6050_ADDRESS             0x68

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define UPPER_RANGE 24576 // 75% of int16_t
#define BOTTOM_RANGE 16384 // 50% of int16_t

volatile bool refreshNeeded = false;
volatile byte buttonPressed = 0;

int menuPosition = MENU_HOME;

void interrupt() {
  refreshNeeded = true;
}

int accRange, gyroRange;

void setup()
{
  display.begin();                                                                    // Стартиране на връзката с дисплея.
  display.setFont(SmallFont);

  Wire.begin();                                                                       //Стартиране на I2C в режим мастър
  
  Serial.begin(38400);                                                                //Стартиране на серийната комуникация. Скорост 38400 bps
  Serial.println("Setting some things up.");

  MsTimer2::set(100, interrupt);
  MsTimer2::start();
  
  pinMode(LED_PIN, OUTPUT);                                                           // LED пина конфигуриран, като изход.

  setup_mpu_6050_registers();                                                         //Конфигуриране на mpu_6050.                
  calibrateGyro();

  Serial.println("Done");
}

void loop() {
    
  buttonPressed = keypad.getKey();
  switch (buttonPressed) {
    case BUTTON_1:
      reset();
      break;
    case BUTTON_3:
      if (menuPosition == MENU_HOME)
        menuPosition = MENU_HOME_MAX;
      else
        menuPosition = MENU_HOME;
      break;
  }

//  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  read_mpu_6050_data();

  gx -= gyro_x_cal;                                                //Subtract the offset calibration value from the raw gyro_x value
  gy -= gyro_y_cal;                                                //Subtract the offset calibration value from the raw gyro_y value
  gz -= gyro_z_cal;                                                //Subtract the offset calibration value from the raw gyro_z value

  compare(ax, &maxax);
  compare(ay, &maxay);
  compare(az, &maxaz);
  compare(gx, &maxgx);
  compare(gy, &maxgy);
  compare(gz, &maxgz);

  checkRange();

  refresh();
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));;
}

void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(MPU6050_ADDRESS);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(MPU6050_ADDRESS,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  ax = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_x variable
  ay = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_y variable
  az = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  gx = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gyro_x variable
  gy = Wire.read()<<8|Wire.read();                                     //Add the low and high byte to the gyro_y variable
  gz = Wire.read()<<8|Wire.read();                                    //Add the low and high byte to the gyro_z variable

}

void setFullScaleAccRange(uint8_t range){
  Wire.beginTransmission(MPU6050_ADDRESS);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(range<<3);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}

void setFullScaleGyroRange(uint8_t range){
  Wire.beginTransmission(MPU6050_ADDRESS);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(range<<3);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission 
}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(MPU6050_ADDRESS);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x01);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  setFullScaleAccRange(0);
  setFullScaleGyroRange(0);
}

void compare(int16_t current, int16_t* maxvalue)
{
  if (abs(current) > abs(*maxvalue))
    *maxvalue = current;
}

void reset() {
  maxax = maxay = maxaz = maxgx = maxgy = maxgz = 0;
  accRange = 0;
  gyroRange = 0;
  setFullScaleAccRange(accRange);
  setFullScaleGyroRange(gyroRange);
  //mpu.setFullScaleAccelRange(accRange);
  //mpu.setFullScaleGyroRange(gyroRange);
}

void checkRange()
{
  //Check max acceleration
  if (abs(ax) > UPPER_RANGE || abs(ay) > UPPER_RANGE || abs(az) > UPPER_RANGE)
  {
    if (accRange < 3)
    {
      accRange++;
      maxax = maxax / 2;
      maxay = maxay / 2;
      maxaz = maxaz / 2;
      setFullScaleAccRange(accRange);
      //mpu.setFullScaleAccelRange(accRange);
      //reset();
    }
  }
  //  else if(abs(maxax) < BOTTOM_RANGE || abs(maxay) < BOTTOM_RANGE || abs(maxaz) < BOTTOM_RANGE)
  //  {
  //    if(accRange > 0)
  //    {
  //      accRange--;
  //      mpu.setFullScaleAccelRange(accRange);
  //      reset();
  //    }
  //  }

  //Check max gyro
  if (abs(gx) > UPPER_RANGE || abs(gy) > UPPER_RANGE || abs(gz) > UPPER_RANGE)
  {
    if (gyroRange < 3)
    {
      gyroRange++;
      maxgx = maxgx / 2;
      maxgy = maxgy / 2;
      maxgz = maxgz / 2;
      setFullScaleGyroRange(gyroRange);
    }
  }
  //  else if(abs(maxgx) < BOTTOM_RANGE || abs(maxgy) < BOTTOM_RANGE || abs(maxgz) < BOTTOM_RANGE)
  //  {
  //    if(gyroRange > 0)
  //    {
  //      gyroRange--;
  //      mpu.setFullScaleGyroRange(accRange);
  //      reset();
  //    }
  //  }
}

int16_t int_pow(int16_t base, int16_t exp)
{
  int result = base;
  for (int i = 1; i < exp;  i++)
  {
    result *= base;
  }
  return result;
}
void displayReadings(int16_t _ax, int16_t _ay, int16_t _az, int16_t _gx, int16_t _gy, int16_t _gz)
{
  display.print("Acc", DSP_V_POS_ACC, ROW_HIGH);
  display.print("x=", DSP_V_POS_ACC, ROW_HIGH * 2);
  display.printNumF(convertAcc(_ax), 2, DSP_V_POS_ACC + DSP_V_DATA_OFFSET, FONT_HEIGHT * 2);
  display.print("y=", DSP_V_POS_ACC, ROW_HIGH * 3);
  display.printNumF(convertAcc(_ay), 2, DSP_V_POS_ACC + DSP_V_DATA_OFFSET, FONT_HEIGHT * 3);
  display.print("z=", DSP_V_POS_ACC, ROW_HIGH * 4);
  display.printNumF(convertAcc(_az), 2, DSP_V_POS_ACC + DSP_V_DATA_OFFSET, FONT_HEIGHT * 4);

  display.print("Angles", DSP_V_POS_GYRO, ROW_HIGH);
  display.print("pitch=", DSP_V_POS_GYRO, ROW_HIGH * 2);
  display.printNumF(convertGyro(_gx), 2, DSP_V_POS_GYRO + (FONT_WIDTH * 6) , FONT_HEIGHT * 2);
  display.print("roll=", DSP_V_POS_GYRO, ROW_HIGH * 3);
  display.printNumF(convertGyro(_gy), 2, DSP_V_POS_GYRO + (FONT_WIDTH * 6), FONT_HEIGHT * 3);
  display.print("yaw=", DSP_V_POS_GYRO, ROW_HIGH * 4);
  display.printNumF(convertGyro(_gz), 2, DSP_V_POS_GYRO + (FONT_WIDTH * 6), FONT_HEIGHT * 4);
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

void calibrateGyro() {
 
  display.clrScr();
  display.print("Calibrating gyro", 0, 0);
  display.update();
  
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Run this code 2000 times
    //if(cal_int % 125 == 0) display.print();                              //Print a dot on the LCD every 125 readings
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_cal += gx;                                                  //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_cal += gy;                                                  //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_cal += gz;                                                  //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  
  gyro_x_cal /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_cal /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_cal /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset  
}

void refresh() {

  if (!refreshNeeded) return;

  refreshNeeded = false;

  display.clrScr();

  displayButtons();

//  display.print("+/-", DSP_V_POS_ACC, ROW_HIGH * 5);
//  display.printNumI(int_pow(2, (accRange + 1)), DSP_V_POS_ACC + 18, ROW_HIGH * 5, 2);
//  display.print("+/-", DSP_V_POS_GYRO, ROW_HIGH * 5);
//  display.printNumI(250 * (gyroRange + 1), DSP_V_POS_GYRO + 18, ROW_HIGH * 5, 4);

  switch (menuPosition)
  {
    case MENU_HOME:
      displayReadings(ax, ay, az, gx, gy, gz);
      display.print("MPU6050", CENTER, 0);
      display.print("Mode", 33, 63 - 8);
      break;
    case MENU_HOME_MAX:
      displayReadings(maxax, maxay, maxaz, maxgx, maxgy, maxgz);
      display.print("ADXL345", CENTER, 0);
      display.print("Mode", 33, 63 - 8);
      break;
  }
  display.update();
}

double convertAcc(int16_t value) {
  //return roundf(((value / 16384) * pow(2,(accRange + 1)))*100.0)/100.0;
  return (value / 16384.00) * pow(2, accRange);
}

double convertGyro(int16_t value) {
  return (value / 32768.00) * 250 * (gyroRange + 1);
}

