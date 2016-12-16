#include <Adafruit_CircuitPlayground.h>
#include <Wire.h>
#include <SPI.h>

typedef enum { 
  input, servomotor, pwm, digital } 
pinType;

typedef struct pin {
  pinType type;       //Type of pin
  int state;         //State of an output
  //byte value;       //Value of an input. Not used by now. TODO
};

pin arduinoPins[14];  //Array of struct holding 0-13 pins information

unsigned long lastDataReceivedTime = millis();
uint8_t pixeln = 0;

void setup()
{
  Serial.begin(38400);
  CircuitPlayground.begin();
  Serial.flush();
  configurePins();
  resetPins();
}

void loop()
{
  static unsigned long timerCheckUpdate = millis();

  if (millis()-timerCheckUpdate>=10)
  {
    sendUpdateServomotors();
    sendSensorValues();
    timerCheckUpdate=millis();
  }
  /************* TEST 10 NEOPIXELS */
//  CircuitPlayground.setPixelColor(pixeln++, CircuitPlayground.colorWheel(25 * pixeln));
//  if (pixeln == 11) {
//    pixeln = 0;
//    CircuitPlayground.clearPixels();
  readSerialPort();
  

}

void configurePins()
{
  arduinoPins[0].type=input;
  arduinoPins[1].type=input;
  arduinoPins[2].type=input;
  arduinoPins[3].type=input;
  arduinoPins[4].type=input;
  arduinoPins[5].type=pwm;
  arduinoPins[6].type=pwm;
  arduinoPins[7].type=servomotor;
  arduinoPins[8].type=servomotor;
  arduinoPins[9].type=servomotor;
  arduinoPins[10].type=digital;
  arduinoPins[11].type=digital;
  arduinoPins[12].type=digital;
  arduinoPins[13].type=digital;
 
}

void resetPins() {
  for (byte index=0; index <=13; index++) 
  {
    if (arduinoPins[index].type!=input)
    {
      pinMode(index, OUTPUT);
      if (arduinoPins[index].type==servomotor)
      {
        arduinoPins[index].state = 255;
        servo (index, 255);
      }
      else
      {
        arduinoPins[index].state=0;
        digitalWrite(index,LOW);
      }
    }
  }
}

void sendSensorValues()
{
  
  ScratchBoardSensorReport(0, CircuitPlayground.temperature()*10);
  ScratchBoardSensorReport(1, analogRead(4));
  ScratchBoardSensorReport(2, analogRead(5));
  ScratchBoardSensorReport(3, CircuitPlayground.motionX()*10+100);
  ScratchBoardSensorReport(4, CircuitPlayground.motionY()*10+100);
  ScratchBoardSensorReport(5, analogRead(21));
  //send digital sensor values
  ScratchBoardSensorReport(6, digitalRead(19)?1023:0);
  ScratchBoardSensorReport(7, digitalRead(4)?1023:0);
}

void insertionSort(unsigned int* array, unsigned int n)
{
  for (int i = 1; i < n; i++)
    for (int j = i; (j > 0) && ( array[j] < array[j-1] ); j--)
      swap( array, j, j-1 );
}

void swap(unsigned int* array, unsigned int a, unsigned int b)
{
  unsigned int temp = array[a];
  array[a] = array[b];
  array[b] = temp;
}

void ScratchBoardSensorReport(byte sensor, int value) //PicoBoard protocol, 2 bytes per sensor
{
  Serial.write( B10000000
    | ((sensor & B1111)<<3)
    | ((value>>7) & B111));
  Serial.write( value & B1111111);
}

void readSerialPort()
{
  byte pin;
  int newVal;
  static byte actuatorHighByte, actuatorLowByte;
  static byte readingSM = 0;

  if (Serial.available())
  {
    if (readingSM == 0)
    {
      actuatorHighByte = Serial.read();
      if (actuatorHighByte >= 128) readingSM = 1;
    }
    else if (readingSM == 1)
    {
      actuatorLowByte = Serial.read();
      if (actuatorLowByte < 128) readingSM = 2;
      else readingSM = 0;
    }

    if (readingSM == 2)
    {
      lastDataReceivedTime = millis();    
      pin = ((actuatorHighByte >> 3) & 0x0F);
      newVal = ((actuatorHighByte & 0x07) << 7) | (actuatorLowByte & 0x7F); 

      if(arduinoPins[pin].state != newVal)
      {
        arduinoPins[pin].state = newVal;
        updateActuator(pin);
      }
      readingSM = 0;
    }
  }
  else checkScratchDisconnection();
}

void reset() //with xbee module, we need to simulate the setup execution that occurs when a usb connection is opened or closed without this module
{
  resetPins();        // reset pins
  sendSensorValues(); // protocol handshaking
  lastDataReceivedTime = millis();
}

void updateActuator(byte pinNumber)
{
//  if (arduinoPins[pinNumber].type==digital) digitalWrite(pinNumber, arduinoPins[pinNumber].state);
//  else if (arduinoPins[pinNumber].type==pwm) analogWrite(pinNumber, arduinoPins[pinNumber].state);
  analogWrite(6, arduinoPins[6].state);
  analogWrite(9, arduinoPins[9].state);
  digitalWrite(10, arduinoPins[10].state);
  digitalWrite(13, arduinoPins[13].state);
  CircuitPlayground.strip.setPixelColor(arduinoPins[5].state, arduinoPins[4].state, arduinoPins[7].state, arduinoPins[8].state);
  CircuitPlayground.strip.show();
  
}

void sendUpdateServomotors()
{
  for (byte p = 0; p < 10; p++)
    if (arduinoPins[p].type == servomotor) servo(p, arduinoPins[p].state);
}

void servo (byte pinNumber, byte angle)
{
  if (angle != 255)
    pulse(pinNumber, (angle * 10) + 600);
}

void pulse (byte pinNumber, unsigned int pulseWidth)
{
  digitalWrite(pinNumber, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pinNumber, LOW);
}

void checkScratchDisconnection() //the reset is necessary when using an wireless arduino board (because we need to ensure that arduino isn't waiting the actuators state from Scratch) or when scratch isn't sending information (because is how serial port close is detected)
{
  if (millis() - lastDataReceivedTime > 1000) reset(); //reset state if actuators reception timeout = one second
}



