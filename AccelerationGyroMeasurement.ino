#include "Wire.h"
#include "Keypad.h"
#include "OLED_I2C.h"

#define BUTTON_1 '1'
#define BUTTON_2 '2'
#define BUTTON_3 '3'
#define BUTTON_4 '4'


const byte ROWS = 2; //Два реда
const byte COLS = 2; //Две колони

char keys[ROWS][COLS] = {
  {'4', '3'},
  {'2', '1'}
};

byte rowPins[ROWS] = {4, 5}; //пинове към които са свързани редовете на клавиатурата
byte colPins[COLS] = {6, 7}; //пинове към които са свързани колоните на клавиатурата

Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

void setup()
{

}

void loop() {
  
}

