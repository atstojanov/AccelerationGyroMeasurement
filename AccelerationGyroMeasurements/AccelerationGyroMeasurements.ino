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

#define FONT_WIDTH 6
#define FONT_HEIGHT 8
#define DISPLAY_WIDTH_PIXELS 128
#define DISPLAY_HEIGHT_PIXELS 64

#define BUTTON_1 '1'
#define BUTTON_2 '2'
#define BUTTON_3 '3'
#define BUTTON_4 '4'

#define MENU_HOME 1
#define MENU_HOME_MAX 2

#define ADXL345_ID 0x01
#define ADXL345_ADDRESS 0x53
#define ADXL345_RA_POWER_CTL 0x2D
#define ADXL345_RA_DATA_FORMAT 0x31
#define ADXL345_RA_DATAX0 0x32
#define ADXL345_RA_DATAX1 0x33
#define ADXL345_RA_DATAY0 0x34
#define ADXL345_RA_DATAY1 0x35
#define ADXL345_RA_DATAZ0 0x36
#define ADXL345_RA_DATAZ1 0x37
#define ADXL345_ACC_LSB 256

#define MPU6050_ID 0x00
#define MPU6050_ADDRESS 0x68
#define MPU6050_RA_GYRO_CONFIG 0x1B
#define MPU6050_RA_ACCEL_CONFIG 0x1C
#define MPU6050_ACC_LSB 16384
#define MPU6050_GYRO_LSB 32768

#define LED_PIN 13

char *labelsXYZ[] = {"x=", "y=", "z="};
char *labelsYPR[] = {"pitch=", "roll=", "yaw="};

// Брой редове на клавиатурата
const byte ROWS = 2;
// Брой колони на клавиатурата
const byte COLS = 2;

// Описание на стойностите, на които отговарят бутоните на клавиатурата
char keys[ROWS][COLS] = {
    {'4', '3'},
    {'2', '1'}};

//пинове към които са свързани редовете на клавиатурата
byte rowPins[ROWS] = {4, 5};
//пинове към които са свързани колоните на клавиатурата
byte colPins[COLS] = {6, 7};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

OLED display(SDA, SCL, 10);
extern uint8_t SmallFont[];

long rawAcc[3];    // данни от акселерометъра в чист вид.
long rawGyro[3];   // данни от жироскопа в чист вид.
float realAcc[3];  // показания на акселерометъра в g.
float realGyro[3]; // показания на жироскопа в deg/s.

unsigned long interval; // интервал от време изминал от последното изчисление
float RwAcc[3];         // проекция нормализирания вектор на гравитацията върху x/y/z осите, измерен от акселерометъра.
float RwGyro[3];        // Rw получен при последните изчисления
float Awz[2];           // ъглите между проекцията на R върху  XZ/YZ равнините и оста Z (deg).

float RwEst[3]; //Rw изчислен от комбинирането на RwAcc и RwGyro

unsigned long lastMicros;
int temperature; //  използва се само при вземане на данните от MPU 6050.

float wGyro; // коефициент показващ каква тежест имат резултатите от жироскопа

// отмествания от реалната стойност за акселерометъра
long accOffsetX, accOffsetY, accOffsetZ;
// отмествания от реалната стойности за жироскопа
long gyroOffsetX, gyroOffsetY, gyroOffsetZ;

volatile bool updateDisplayNeeded = false; // показва дали данните на дисплея трябва да се опреснят.

bool firstSample = true; // Първо изчисление или не?

int accRange = 0, gyroRange = 3; // текущ обхват на акселерометъра и жироскопа

// char ADXL345Name[] = "ADXL345";
// char MPU6050Name[] = "MPU6050";
//char *modes[] = {"Estimates", "Max estimates", "RAW", "Max RAW" };

int mode = 0;
// char keyPressed;
// bool holdButtonProcessing = false;

// struct
// {
//   byte id; // 0 MPU6050; 1 ADXL345
//   char *name;
//   int gRange;
//   int aRange;
// } sensor;

void interrupt()
{
  updateDisplayNeeded = true;
}

void setup()
{
  //Стартиране на серийната комуникация. Скорост 38400 bps
  Serial.begin(38400);
  // Издаване на съобщение за старт на инициализацията
  Serial.println(F("Initialization ..."));

  // Стартиране на връзката с дисплея.
  display.begin();
  display.setFont(SmallFont);

  //Стартиране на I2C в режим мастър.
  Wire.begin();

  // конфигуриране на LED пина, като изход.
  pinMode(LED_PIN, OUTPUT);

  //Конфигуриране на MPU6050.
  setupMPU6050();

  //Конфигуриране на ADXL345.
  setupADXL345();

  //Калибриране на сензорите.
  calibrate();

  //Конфигуриране на клавиатурата
  keypad.setHoldTime(1000);

  wGyro = 10;

  //Стартиране на брояча за опресняване на дисплея
  MsTimer2::set(100, interrupt);
  MsTimer2::start();

  // // избор на активен сензор
  // setActiveSensor(MPU6050_ID);

  //Съобщение в серийната конзола, че инициализацията е завършила
  Serial.println(F("Done!"));
}

void loop()
{
  // Прочита се състоянието на бутоните
  readButtons();
  // чете се сензора
  readSensor();
  // Конвертиране на данните от чист вид в удобен за ползавне
  // От акселерометъра в g.
  // От жироскопа в градуси за секунда (deg/s).
  rawToReal();
  // Изчислява се наклона на устройството.
  getEstimatedInclination();

  // Опреснява се информацията на дисплея
  updateDisplay();

  // Изпращт се данни през серийния интерфейс към PC.
  sendData();

  // Променя се състоянието на индикиращия LED. Той служи като индикация, че програмата не е спряла.
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void getEstimatedInclination()
{
  static int i, w;
  static float tmpf, tmpf2;
  static unsigned long newMicros;
  static char signRzGyro;

  // Запазва се времето когато е направено измерването
  newMicros = micros();

  // Изчисляване на интервала от време от предишното измерване
  interval = newMicros - lastMicros;
  // Запазване за следващото измерване. При първото измерване тази стойност ще е невалидна, но в този случай тя не влиза в изчисленията.
  lastMicros = newMicros;

  // Получаване на данните за акселерометъра в g.
  for (w = 0; w <= 2; w++)
  {
    // вектор RwAcc
    RwAcc[w] = realAcc[w];
  }

  // нормализиране на вектора (новият вектор има същата посока и дължина 1)
  normalize3DVector(RwAcc);

  if (firstSample)
  {
    // Първа стойност. Вземат се данните от акселерометъра
    for (w = 0; w <= 2; w++)
      RwEst[w] = RwAcc[w];
  }
  else
  {
    // Изчисляване на RwGyro вектора
    if (abs(RwEst[2]) < 0.1)
    {
      // Стойността на Rz е твърде малка, тъй като тази стойност се взема за референция за изчисляването на Axz, Ayz,  флуктоациите на нейната грешка, ще бъдат усилени,
      // което ще доведе до грешни резлутати. В този случай можем да я пренебрегнем и за тази цел ще използваме стойностите от предишното изчисление.
      for (w = 0; w <= 2; w++)
        RwGyro[w] = RwEst[w];
    }
    else
    {
      // изчисляване на ъглите между проекцията на R върху ZX/ZY и оста Z, базирани на последните изчисления RwEst
      for (w = 0; w <= 1; w++)
      {
        tmpf = realGyro[w] / 1000.0;                     // текуща стойност на жироскопа в deg/ms
        tmpf *= interval / 1000.0f;                      // промяна на ъгъла в градуси
        Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD_TO_DEG; // изчисляване на ъглите в градуси
        Awz[w] += tmpf;                                  // нова стойност за ъгъла, спрямо жироскопа
      }
      // Определяне знака на RzGyro, като проверяваме в кой квадрант лежи ъгъла Axz.
      // RzGyro е положителен ако Axz е в границите -90 до 90 градуса => cos(Awz) >= 0
      signRzGyro = (cos(Awz[0] * DEG_TO_RAD) >= 0) ? 1 : -1;

      //Обратно пресмятане на RwGyro от ъглите Awz.
      for (w = 0; w <= 1; w++)
      {
        RwGyro[w] = sin(Awz[w] * DEG_TO_RAD);
        RwGyro[w] /= sqrt(1 + squared(cos(Awz[w] * DEG_TO_RAD)) * squared(tan(Awz[1 - w] * DEG_TO_RAD)));
      }
      RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) - squared(RwGyro[1]));
    }

    // комбиниране на данните от Акселерометъра и жироскопа
    for (w = 0; w <= 2; w++)
      RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

    normalize3DVector(RwEst);
  }

  // Преизчисляване на ъглите след променените стойности, за визуализация.
  for (w = 0; w <= 1; w++)
  {
    Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD_TO_DEG;
  }

  firstSample = false;
}

// функция за нормализиране на вектор.
void normalize3DVector(float *vector)
{
  static float R;
  R = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  vector[0] /= R;
  vector[1] /= R;
  vector[2] /= R;
}

// функция за изчесление на квадрат на число с плаваща запетая.
float squared(float x)
{
  return x * x;
}

void setupMPU6050()
{
  // изкарване на от режим Sleep MPU-6050
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission();
  // задаване на най-нисък обхват на работа
  setFullScaleAccRange(accRange);
  setFullScaleGyroRange(gyroRange); //2000 deg/s
}

void setFullScaleAccRange(uint8_t range)
{
  // задаване на обхват на акселерометъра
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_ACCEL_CONFIG);
  // битове 3 и 4 определят обхвата
  Wire.write(range << 3);
  Wire.endTransmission();
}

void setFullScaleGyroRange(uint8_t range)
{
  // задаване на обхват на жироскопа
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(MPU6050_RA_GYRO_CONFIG);
  // битове 3 и 4 определят обхвата
  Wire.write(range << 3);
  Wire.endTransmission();
}

// четене на данните от MPU6050
void readMPU6050()
{
  // Начало на комуникацията с MPU-6050 по I2C.
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);                      // Изпращане на адреса на регистъра от който ще се чете
  Wire.endTransmission();                // Край на предааването
  Wire.requestFrom(MPU6050_ADDRESS, 14); // Заявка за 14 байта от MPU-6050
  while (Wire.available() < 14)          // Изчакване докато бойтовете са получени
  {
  };
  // Измерените стойности от сензора са 16 битови. Четенето по I2C е на части от по 8 бита.
  // за това се налага събирането на два последователни байта в една стойностт.
  rawAcc[0] = Wire.read() << 8 | Wire.read();
  rawAcc[1] = Wire.read() << 8 | Wire.read();
  rawAcc[2] = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() << 8 | Wire.read();
  rawGyro[0] = Wire.read() << 8 | Wire.read();
  rawGyro[1] = Wire.read() << 8 | Wire.read();
  rawGyro[2] = Wire.read() << 8 | Wire.read();
}

void setupADXL345()
{

  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_RA_POWER_CTL);
  Wire.write(0x00);
  Wire.endTransmission();
  // включване на акселерометъра в режим на измерване
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_RA_POWER_CTL);
  Wire.write(0x18);
  Wire.endTransmission();
  setADXL345Range(accRange);
}

void setADXL345Range(uint8_t range)
{
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_RA_DATA_FORMAT);
  Wire.write(range);
  Wire.endTransmission();
}

void readADXL345()
{
  // Начало на комуникацията с MPU-6050 по I2C.
  Wire.beginTransmission(ADXL345_ADDRESS);
  Wire.write(ADXL345_RA_DATAX0);        // Изпращане на адреса на регистъра от който ще се чете
  Wire.endTransmission();               // Край на предааването
  Wire.requestFrom(ADXL345_ADDRESS, 6); // Заявка за 6 байта от ADXL345
  while (Wire.available() < 6)          // Изчакване докато бойтовете са получени
  {
  };
  // Измерените стойности от сензора са 12 битови. Четенето по I2C е на части от по 8 бита.
  // за това се налага събирането на два последователни байта в една стойностт.
  rawAcc[0] = Wire.read() | Wire.read() << 8;
  rawAcc[1] = Wire.read() | Wire.read() << 8;
  rawAcc[2] = Wire.read() | Wire.read() << 8;
  temperature = Wire.read() << 8 | Wire.read();
  rawGyro[0] = 0;
  rawGyro[1] = 0;
  rawGyro[2] = 0;
}

void rawToReal()
{
  for (int i = 0; i < 3; i++)
  {
    realGyro[i] = rawToRealGyro(rawGyro[i]);
    realAcc[i] = rawToRealAcc(rawAcc[i]);
  }
}

// рестарт
void reset()
{
  firstSample = true;
  for (int i = 0; i < 3; i++)
  {
    RwAcc[i] = 0;
    RwGyro[i] = 0;
    Awz[i] = 0;
    RwEst[i] = 0;
    rawAcc[i] = 0;
    rawGyro[i] = 0;
  }
  accRange = 0;
  gyroRange = 3;
  setFullScaleAccRange(accRange);
  setFullScaleGyroRange(gyroRange);
  setADXL345Range(accRange);
}

// показване на данните на екрана
void displayReadings()
{
  static int i;

  display.print(F("Accel"), colPos(0), rowPos(1));
  display.print(F("[g]"), colPos(1), rowPos(2));
  
  for (i = 0; i < 3; i++)
  {
    display.print(labelsXYZ[i], colPos(0), rowPos(4 + i));
    if (realAcc[i] < 0.0)
      display.print(F("-"), colPos(2), rowPos(4 + i));
    display.printNumF(abs(realAcc[i]), 2, colPos(3), rowPos(4 + i));
  }

  if (isRawMode())
  {
    display.print(F("Velocity"), colPos(9), rowPos(1));
    display.print(F("[deg/s]"), colPos(19), rowPos(2)); 
    for (i = 0; i < 3; i++)
    {
      display.print(labelsXYZ[i], colPos(9), rowPos(4 + i));
      if (realGyro[i] < 0.0)
        display.print(F("-"), colPos(11), rowPos(4 + i));
      display.printNumF(abs(realGyro[i]), 2, colPos(12), rowPos(4 + i));
    }
  }
  else
  {
    display.print(F("Angles"), colPos(9), rowPos(1));
    display.print(F("[deg]"), colPos(9), rowPos(2));
    for (i = 0; i < 2; i++)
    {
      display.print(labelsYPR[i], colPos(9), rowPos(4 + i));
      if (Awz[i] < 0.0)
        display.print(F("-"), colPos(15), rowPos(4 + i));
      display.printNumF(abs(Awz[i]), 2, colPos(16), rowPos(4 + i));
    }
  }
}

// изчисляване на позицията на редовете на екрана
int rowPos(int row)
{
  return row * FONT_HEIGHT;
}

// изчисляване на позицията на колоните на екрана
int colPos(int col)
{
  return col * FONT_WIDTH;
}

// показване на най-долния ред с текст над бутоните.
void displayButtons()
{
  // display.drawRect(0, 63 - 10, 127, 63);
  // display.drawLine(31, 63 - 10, 31, 63);
  // display.drawLine(63, 63 - 10, 63, 63);
  // display.drawLine(95, 63 - 10, 95, 63);
  // display.print("Mode", 2, 63 - 8);
  // display.print("Rst", 97, 63 - 8);
}

void calibrateSensor()
{
  int percent = 0;
  // Взимаме 500 стойности
  for (int cal_int = 0; cal_int < 500; cal_int++)
  {
    if (cal_int % 5 == 0)
    {
      display.printNumI(++percent, CENTER, rowPos(1));
      display.update();
    }
    readSensorNoOffset();
    gyroOffsetX += rawGyro[0];
    gyroOffsetY += rawGyro[1];
    gyroOffsetZ += rawGyro[2];
    accOffsetX += rawAcc[0];
    accOffsetY += rawAcc[1];
    accOffsetZ += (rawAcc[2] - 1); // z винаги показва 1.
    delay(3);                      // Забавяне от 3 микро секунди.
  }

  // изчисляваме средната стойност от събраните данни
  gyroOffsetX /= 500;
  gyroOffsetY /= 500;
  gyroOffsetZ /= 500;
  accOffsetX /= 500;
  accOffsetY /= 500;
  accOffsetZ /= 500;
}

void updateDisplay()
{
  if (!updateDisplayNeeded)
    return;

  updateDisplayNeeded = false;

  display.clrScr();

  if (getSensorID() == MPU6050_ID)
  {
    display.print(F("MPU6050"), CENTER, rowPos(0));
  }
  else
  {
    display.print(F("ADXL345"), CENTER, rowPos(0));
  }
  displayReadings();

  display.update();
}

// изчисляване на показанията на акселерометъра в g. Обхвата се взема в предвид.
double rawToRealAcc(int16_t value)
{
  return (double)value / (getAccLSB() / pow(2, accRange));
}

// изчисляване на показанията на жироскопа в deg/s. Обхвата се взема в предвид.
double rawToRealGyro(int16_t value)
{
  return (double)value / (getGyroLSB() / (250 * pow(2, gyroRange)));
}

void readButtons()
{
  char key = keypad.getKey();
  switch (key)
  {
  case BUTTON_1:
    calibrate();
    break;
  case BUTTON_2:
    changeRange();
    break;
  case BUTTON_3:
    changeConnection();
    break;
  case BUTTON_4:
    changeMode();
    break;
  }
}

void calibrate()
{
  display.clrScr();
  if (getSensorID() == MPU6050_ID)
    display.print(F("Calibrating MPU6050"), CENTER, rowPos(0));
  else
    display.print(F("Calibrating ADXL345"), CENTER, rowPos(0));
  display.update();

  calibrateSensor();
}

void changeSensor()
{
  Serial.println(F("Changing sensors ..."));
}

void changeMode()
{
  if (mode == 3)
  {
    mode = 0;
  }
  else
  {
    mode++;
  }
  reset();

  if (mode >= 2)
  {
    calibrate();
  }
  reset();
}

void changeConnection()
{
  Serial.println(F("Changing connection ..."));
}

void changeRange()
{
  Serial.println(F("Changing range ..."));
}

void sendData()
{
}

void readSensor()
{
  readSensorNoOffset();
  rawAcc[0] += accOffsetX;
  rawAcc[1] += accOffsetY;
  rawAcc[2] += accOffsetZ;
  rawGyro[0] += gyroOffsetX;
  rawGyro[1] += gyroOffsetY;
  rawGyro[2] += gyroOffsetZ;
}

void readSensorNoOffset()
{
  if (getSensorID() == MPU6050_ID)
  {
    readMPU6050();
  }
  else
    readADXL345();
}

int16_t getAccLSB()
{
  if (getSensorID() == MPU6050_ID)
    return MPU6050_ACC_LSB;
  else
    return ADXL345_ACC_LSB;
}

int16_t getGyroLSB()
{
  if (getSensorID() == MPU6050_ID)
    return MPU6050_GYRO_LSB;
  else
    return 1; // ADXL 345 няма жироскоп, затова се връща 1.
}

byte getSensorID()
{
  if (mode <= 1)
    return MPU6050_ID;
  if (mode > 1)
    return ADXL345_ID;
}

bool isRawMode()
{
  return mode == 1 || mode == 3;
}