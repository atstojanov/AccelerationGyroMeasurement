#include "Wire.h"
#include "Keypad.h"
#include "OLED_I2C.h"
#include <MsTimer2.h>
#include "avr/pgmspace.h"

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
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48
#define MPU6050_RA_PWR_MGMT_1 0x6B
#define MPU6050_ACC_LSB 16384
#define MPU6050_GYRO_LSB 32768

#define LED_PIN 13

extern uint8_t plusminus[];

char *labelsXYZ[] = { "x=", "y=", "z=" };
char *labelsYPR[] = { "p=", "r=", "y=" };
char *accRanges[] = { "2g", "4g", "8g", "16g" };

// Брой редове на клавиатурата
const byte ROWS = 2;
// Брой колони на клавиатурата
const byte COLS = 2;

// Описание на стойностите, 
// на които отговарят бутоните на клавиатурата
char keys[ROWS][COLS] = {
	{'4', '3'},
	{'2', '1'} };

// Пинове към които са свързани редовете на клавиатурата.
byte rowPins[ROWS] = { 4, 5 };
// Пинове към които са свързани колоните на клавиатурата
byte colPins[COLS] = { 6, 7 };

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

OLED display(SDA, SCL, 10);
extern uint8_t SmallFont[];

// данни от акселерометъра в чист вид.
long rawAcc[3];
// данни от жироскопа в чист вид.
long rawGyro[3];
// показания на акселерометъра в g.
float realAcc[3];
// показания на жироскопа в deg/s.
float realGyro[3]; 

// интервал от време изминал от последното изчисление
unsigned long interval;
// проекция вектора на гравитацията върху x/y/z осите.
float RwAcc[3];
// Rw получен при последните изчисления
float RwGyro[3];
// ъглите между проекцията на R върху XZ/YZ равнините и оста Z (deg).
float Awz[2];
//Rw изчислен от комбинирането на RwAcc и RwGyro
float RwEst[3]; 

// Първо изчисление или не?
bool firstSample = true;

unsigned long lastMicros;

// Температурата измерена от температурния сензор на MPU6050.
int temperature; 

// Тежест на резултатите от жироскопа.
const float wGyro = 10; 

// Отмествания от реалната стойност за акселерометъра.
long accOffsetX, accOffsetY, accOffsetZ;
// Отмествания от реалната стойности за жироскопа.
long gyroOffsetX, gyroOffsetY, gyroOffsetZ;

// Показва дали данните на дисплея трябва да се опреснят.
volatile bool updateDisplayNeeded = false; 

// Текущ обхват на акселерометъра и жироскопа.
int accRange = 0, gyroRange = 3; 

// режим на работа
// 0 - MPU6050, визуализира се ускорение и наклон на устройството.
// 1 - MPU6050, визуализира се ускорение и скорост на завъртане.
// 2 - ADXL345, визуализира се ускорение и наклон на устройството.
byte mode = 0;

// Оказва дали е включена връзката към PC или не.
bool pcConnection;

// Принудително опресняване на дисплея.
bool forceUpdate = false;

// Извиква се, когато таймера достигне зададеното време.
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

	//Стартиране на брояча за опресняване на дисплея
	MsTimer2::set(100, interrupt);
	MsTimer2::start();

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

	if (pcConnection)
	{
		// Изпращт се данни през серийния интерфейс към PC.
		sendData();
	}
	else
	{
		// Опреснява се информацията на дисплея
		refresh();
	}

	// Променя се състоянието на LED индикацията.
	// Тя служи като индикация, че програмата не е спряла.
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
	// Запазване за следващото измерване. 
	// При първото измерване тази стойност ще е невалидна,
	// но в този случай тя не влиза в изчисленията.
	lastMicros = newMicros;

	// Получаване на данните за акселерометъра в g.
	for (w = 0; w <= 2; w++)
	{
		// вектор RwAcc
		RwAcc[w] = realAcc[w];
	}

	// нормализиране на вектора (новият вектор има същата посока и 
	// дължина 1)

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
			// Стойността на Rz е твърде малка, тъй като тази 
			// стойност се взема за референция за изчисляването 
			// на Axz, Ayz,  флуктоациите на нейната грешка, ще бъдат
			// усилени, което ще доведе до грешни резлутати. 
			// В този случай можем да я пренебрегнем и за тази цел 
			// ще използваме стойностите от предишното изчисление.
			for (w = 0; w <= 2; w++)
				RwGyro[w] = RwEst[w];
		}
		else
		{
			// изчисляване на ъглите между проекцията на R  
			// върху ZX/ZY и оста Z
			for (w = 0; w <= 1; w++)
			{
				// текуща стойност на жироскопа в deg/ms
				tmpf = realGyro[w] / 1000.0;                     
				// интервал от последното измерване в ms.
				tmpf *= interval / 1000.0f;                      
				// изчисляване на ъглите в градуси
				Awz[w] = atan2(RwEst[w], RwEst[2]) * RAD_TO_DEG; 
				// нова стойност за ъгъла, спрямо жироскопа
				Awz[w] += tmpf;                                  
			}
			// Определяне знака на RzGyro, като проверяваме в кой 
			// квадрант лежи ъгъла Axz.
			// RzGyro е положителен ако Axz е в границите 
			// -90 до 90 градуса => cos(Awz) >= 0
			signRzGyro = (cos(Awz[0] * DEG_TO_RAD) >= 0) ? 1 : -1;

			//Обратно пресмятане на RwGyro от ъглите Awz.
			for (w = 0; w <= 1; w++)
			{
				RwGyro[w] = sin(Awz[w] * DEG_TO_RAD);
				RwGyro[w] /= sqrt(1 + 
					squared(cos(Awz[w] * DEG_TO_RAD)) *
					squared(tan(Awz[1 - w] * DEG_TO_RAD)));
			}
			RwGyro[2] = signRzGyro * sqrt(1 - squared(RwGyro[0]) -
				squared(RwGyro[1]));
		}

		// комбиниране на данните от Акселерометъра и жироскопа
		for (w = 0; w <= 2; w++)
			RwEst[w] = (RwAcc[w] + wGyro * RwGyro[w]) / (1 + wGyro);

		normalize3DVector(RwEst);
	}

	// Преизчисляване на ъглите след променените стойности.
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
	// Привеждане на MPU6050 в работен режим.
	// Начало на комуникация по I2C. Изпраща се адреса
	// на устройството с което ще се комуникира.
	Wire.beginTransmission(MPU6050_ADDRESS); 
	// Изпраща се адреса на регистара в който ще се 
	// записват/четат данни
	Wire.write(MPU6050_RA_PWR_MGMT_1); 
	// Изпращат се данните
	Wire.write(0x01); 
	// Прекратява се комуникацията
	Wire.endTransmission();
	
	// задаване на обхват на работа
	setFullScaleAccRange(accRange);
	setFullScaleGyroRange(gyroRange);
}

// задаване на обхват на акселерометъра на MPU6050
void setFullScaleAccRange(uint8_t range)
{
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(MPU6050_RA_ACCEL_CONFIG);
	Wire.write(range << 3); // битове 3 и 4 определят обхвата
	Wire.endTransmission();
}

void setFullScaleGyroRange(uint8_t range)
{
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(MPU6050_RA_GYRO_CONFIG);
	Wire.write(range << 3); // битове 3 и 4 определят обхвата
	Wire.endTransmission();
}

// четене на данните от MPU6050
void readMPU6050()
{
	Wire.beginTransmission(MPU6050_ADDRESS);
	Wire.write(MPU6050_RA_ACCEL_XOUT_H);
	Wire.endTransmission();                
	// Заявка за прочитане на 14 байта
	Wire.requestFrom(MPU6050_ADDRESS, 14); 
	// Изчакване докато бойтовете са получени
	while (Wire.available() < 14)          
	{
	};
	// Измерените стойности от сензора са 16 битови.
	// Четенето по I2C е на части от по 8 бита.
	// за това се налага събирането на два последователни 
	// байта в една стойност.
	// байтовете са по двойки, като първия байт е MSB,
	// а втория LSB.
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
	// Привеждане на ADXL345 в неработен режим.
	Wire.beginTransmission(ADXL345_ADDRESS);
	Wire.write(ADXL345_RA_POWER_CTL);
	Wire.write(0x00);
	Wire.endTransmission();
	// Привеждане на ADXL345 в работен режим.
	Wire.beginTransmission(ADXL345_ADDRESS);
	Wire.write(ADXL345_RA_POWER_CTL);
	Wire.write(0x08);
	Wire.endTransmission();

	setADXL345Range(accRange);
}

// Установяване на работния обхват на ADXL345
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
	Wire.write(ADXL345_RA_DATAX0);        
	Wire.endTransmission();
	// Заявка за 6 байта от ADXL345
	Wire.requestFrom(ADXL345_ADDRESS, 6); 
	while (Wire.available() < 6)
	{
	};
	// Измерените стойности от сензора са 16 битови.
	// Четенето по I2C е на части от по 8 бита.
	// за това се налага събирането на два последователни 
	// байта в една стойност.
	// байтовете са по двойки, като първия байт е LSB,
	// а втория MSB.
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
	// конвертиране на данните:
	// Accelerometer: bytes -> g
	// Gyroscope: bytes -> deg/s
	for (int i = 0; i < 3; i++)
	{
		realGyro[i] = rawToRealGyro(rawGyro[i]);
		realAcc[i] = rawToRealAcc(rawAcc[i]);
	}
}

// ресет на състоянието на вътрешните променливи
// използва се при смяна режима на работа, обхвата или след калибрация.
void reset()
{
	firstSample = true;
	interval = 0;
	for (int i = 0; i < 3; i++)
	{
		RwAcc[i] = 0;
		RwGyro[i] = 0;
		Awz[i] = 0;
		RwEst[i] = 0;
		rawAcc[i] = 0;
		rawGyro[i] = 0;
	}
}

void setRanges()
{
	setFullScaleAccRange(accRange);
	setFullScaleGyroRange(gyroRange);
	setADXL345Range(accRange);
}

void updateCalibrationInfo(int percent)
{
	updateDisplayNeeded = updateDisplayNeeded || percent == 100;

	if (updateDisplayNeeded)
	{
		display.clrScr();
		displayText(F("Calibrating"), 0, 0);
		displayText(F("%"), 3, 2);
		displaySensorName(0, 1);
		displayAccRange(9, 1);
		display.printNumI(++percent, 0, rowPos(2), 3);
		updateDisplay();
	}
}

void calibrate()
{
	// Взимат се 500 стойности
	for (int cal_int = 0; cal_int < 500; cal_int++)
	{
		if (cal_int % 5 == 0)
		{
			updateCalibrationInfo((cal_int / 5));
		}
		readSensorNoOffset();
		gyroOffsetX += rawGyro[0];
		gyroOffsetY += rawGyro[1];
		gyroOffsetZ += rawGyro[2];
		accOffsetX += rawAcc[0];
		accOffsetY += rawAcc[1];
		accOffsetZ += (rawAcc[2] - getAccLSB());
		// Забавяне от 3 микро секунди.
		delay(3);
	}

	display.update();

	// изчислява се средната стойност от събраните данни
	gyroOffsetX /= 500;
	gyroOffsetY /= 500;
	gyroOffsetZ /= 500;
	accOffsetX /= 500;
	accOffsetY /= 500;
	accOffsetZ /= 500;

	// изпращат се данните от калибрацията към PC
	Serial.print(gyroOffsetX);
	Serial.print(F("\t"));
	Serial.print(gyroOffsetY);
	Serial.print(F("\t"));
	Serial.print(gyroOffsetZ);
	Serial.print(F("\t"));
	Serial.print(accOffsetX);
	Serial.print(F("\t"));
	Serial.print(accOffsetY);
	Serial.print(F("\t"));
	Serial.print(accOffsetZ);
	Serial.println(F("\t"));

	delay(500);
}

// показване на данните на екрана
void displayReadings()
{
	int i;

	// Данните от акселерометъра се показват винаги
	displayText(F("Accel"), 0, 2);
	displayAccRange(0, 3);

	for (i = 0; i < 3; i++)
	{
		displayText(labelsXYZ[i], 0, 4 + i);
		displayNumberF(realAcc[i], 2, 4 + i, 5);
	}

	// Данните от жироскопа могат да бъдат в два вариянта
	// 1. Наклон - в градуси
	// 2. Скорост на въртене - градус/секунда
	if (isRawMode())
	{
		// скорост на въртене
		displayText(F("Velocity"), 9, 2);
		displayText(F("[deg/s]"), 9, 3);

		for (i = 0; i < 3; i++)
		{
			displayText(labelsXYZ[i], 9, 4 + i);
			displayNumberF(realGyro[i], 11, 4 + i, 7);
		}
	}
	else
	{
		// Наклон
		displayText(F("Angles"), 9, 2);
		displayText(F("[deg]"), 9, 3);
		for (i = 0; i < 2; i++)
		{
			display.print(labelsYPR[i], colPos(9), rowPos(4 + i));
			displayNumberF(Awz[i], 11, 4 + i, 6);
		}
	}
}

// Изпращане на информацията от дисплейния буфер към дисплея
void updateDisplay()
{
	if (updateDisplayNeeded || forceUpdate)
	{
		updateDisplayNeeded = false;
		forceUpdate = false;
		display.update();
	}
}

// Опресняване инфромацията в буфера на дисплея
void refresh()
{
	if (!updateDisplayNeeded)
		return;

	display.clrScr();
	displaySensorName(0, 0);
	displayReadings();
	updateDisplay();
}

// Изчисляване на показанията на акселерометъра в g.
// Обхвата се взема в предвид.
double rawToRealAcc(int16_t value)
{
	return (double)value / getAccLSB();
}

// Изчисляване на показанията на жироскопа в deg/s.
// Обхвата се взема в предвид.
double rawToRealGyro(int16_t value)
{
	return (double)value / getGyroLSB();
}

// Четене на бутоните на клавиатурата
// Ако някой от тях е натиснат - изпълнява се съответната функция
// "1" - Калибриране на сензора
// "2" - Смяна на обхвата на сензора
// "3" - Преминаване на връзка към PC или обратно
// "4" - Смяна на режима на работа
void readButtons()
{
	char key = keypad.getKey();
	switch (key)
	{
	case BUTTON_1:
		calibrate();
		forceUpdate = true;
		break;
	case BUTTON_2:
		changeRange();
		forceUpdate = true;
		break;
	case BUTTON_3:
		changeConnection();
		forceUpdate = true;
		break;
	case BUTTON_4:
		changeMode();
		forceUpdate = true;
		break;
	}
}

// Смяна на режима на работа
void changeMode()
{
	if (mode == 2)
	{
		mode = 0;
	}
	else
	{
		mode++;
	}
	reset();

	if (mode == 0 || mode == 2)
	{
		calibrate();
	}
	reset();
}

// Смяна на връзката 
void changeConnection()
{
	pcConnection = !pcConnection;
	forceUpdate = pcConnection;
}

// Смяна на обхвата на акселерометъра

void changeRange()
{
	if (accRange == 3)
	{
		accRange = 0;
	}
	else
	{
		accRange++;
	}
	reset();
	setRanges();
	calibrate();
}

void sendData()
{
	if (forceUpdate)
	{
		display.clrScr();
		displaySensorName(0, 0);
		displayAccRange(0, 1);
		displayText(F("Sending data... "), 0, 4);
		updateDisplay();
	}

	Serial.print(interval); 
	Serial.print(F(","));
	Serial.print(RwAcc[0]); 
	Serial.print(F(","));
	Serial.print(RwEst[0]); 
	Serial.print(F(","));
	Serial.print(RwAcc[1]); 
	Serial.print(F(","));
	Serial.print(RwEst[1]); 
	Serial.print(F(","));
	Serial.print(RwAcc[2]); 
	Serial.print(F(","));
	Serial.print(RwEst[2]); 
	Serial.println(F(""));
}

void readSensor()
{
	readSensorNoOffset();
	rawAcc[0] -= accOffsetX;
	rawAcc[1] -= accOffsetY;
	rawAcc[2] -= accOffsetZ;
	rawGyro[0] -= gyroOffsetX;
	rawGyro[1] -= gyroOffsetY;
	rawGyro[2] -= gyroOffsetZ;
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
		return MPU6050_ACC_LSB / pow(2, accRange);
	else
		return ADXL345_ACC_LSB / pow(2, accRange);
}

int16_t getGyroLSB()
{
	if (getSensorID() == MPU6050_ID)
		return MPU6050_GYRO_LSB / (250 * pow(2, gyroRange));
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