//#include "Wire.h"
#include "Keypad.h"
//#include "OLED_I2C.h"

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
  Serial.begin(38400);
  Serial.println("Setting some things up.");

  keypad.addEventListener(keypadEvent);

  Serial.println("Done");

}

void loop() {
    
  char key = keypad.getKey();
  if (key) {
//    Serial.println(key);
  }
}

void keypadEvent(KeypadEvent key) {
  switch (keypad.getState()) {
    
    case PRESSED:
      Serial.print(key);
      Serial.println(" - PRESSED");
      break;
    case RELEASED:
      Serial.print(key);
      Serial.println(" - RELEASED");
      break;

    case HOLD:
      Serial.print(key);
      Serial.println(" - HOLD");
      break;
  }
}
