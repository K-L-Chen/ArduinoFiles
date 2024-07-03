/*
 * Tutorial 8: Using the LCD
 * 
 * Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 * library works with all LCD displays that are compatible with the 
 * Hitachi HD44780 driver. There are many of them out there, and you
 * can usually tell them by the 16-pin interface.
 *
 * Adjust the LCDs contrast with the Potentiometer until you
 * can see the characters on the LCD.
 *
 * The circuit:
 * - LCD RS pin to digital pin 12
 * - LCD Enable pin to digital pin 11
 * - LCD D4 pin to digital pin 7
 * - LCD D5 pin to digital pin 8
 * - LCD D6 pin to digital pin 9
 * - LCD D7 pin to digital pin 10
 * - LCD R/W pin to ground
 * - 10K potentiometer divider for LCD pin VO:
 * - 330ohm resistor betweenm LCD pin A and 5v
 * - LCD pin K to ground
 *
 * Library originally added 18 Apr 2008
 * by David A. Mellis
 * library modified 5 Jul 2009
 * by Limor Fried (http://www.ladyada.net)
 * example added 9 Jul 2009
 * by Tom Igoe
 * modified 22 Nov 2010
 * by Tom Igoe
 * modified 14 August 2013
 * by Blaise Jarrett
 *
 * This example code is in the public domain.
 *
 * Derivative work from:
 * http://www.arduino.cc/en/Tutorial/LiquidCrystal
 *
 */

// include the library
#include <LiquidCrystal.h>
#include <Servo.h>

// all of our LCD pins
int lcdRSPin = 12;
int lcdEPin = 11;
int lcdD4Pin = 7;
int lcdD5Pin = 8;
int lcdD6Pin = 9;
int lcdD7Pin = 10;


int servoPin = 3;
int xAxis = A0;
int yAxis = A1;
int joystickButton = 5;

int xAxis2 = A5;
int yAxis2 = A4;
Servo servo1;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(lcdRSPin, lcdEPin,
                  lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

float xValue = 0;
float yValue = 0;

void setup()
{
    Serial.begin(9600);
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    
    // Print a message to the LCD.
    //lcd.print("hello, world!");
    //setup physical test light
    //pinMode(LED_BUILTIN, OUTPUT);
    pinMode(xAxis, INPUT);
    pinMode(yAxis, INPUT);
    pinMode(joystickButton, INPUT);

    servo1.attach(servoPin);
}

void calibrate(){
  //does nothing for now
}

void loop()
{
    //digitalWrite(LED_BUILTIN, LOW);
    
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0)
    lcd.setCursor(0, 0);

    // print the number of seconds since reset
    //lcd.print(millis() / 1000);
    
    xValue = analogRead(xAxis);// / (float)1023;
    yValue = analogRead(yAxis);// / (float)1023;
    lcd.print(xValue / (float)1023);
    lcd.setCursor(8, 0);
    lcd.print(yValue / (float)1023);
    //Serial.print(analogRead(xAxis) + "\n" + analogRead(yAxis));

    xValue = analogRead(xAxis2);// / (float)1023;
    yValue = analogRead(yAxis2);// / (float)1023;
    lcd.setCursor(0, 1);
    lcd.print(xValue / (float)1023);
    lcd.setCursor(8, 1);
    lcd.print(yValue/ (float)1023);
    
    int state = 0;
    lcd.setCursor(12, 1);
    /*if(digitalRead(joystickButton) == 1){
      state++;
    }*/

    int moveValue;

    xValue = xValue / (float)1023;
    
    if(xValue > .45 && xValue < .52){
        servo1.write(90);
        lcd.print("90");
      }
    else{
        moveValue = 90 + (xValue - .5) * 180;
        servo1.write(moveValue);
        lcd.print(moveValue);
    }
   
    /*if(state % 2 == 0){
      if(xValue > .45 && xValue < .55){
        servo1.write(90);
        lcd.print("90");
      }
      else{
        moveValue = 90 + (xValue - .5) * 180;
        servo1.write(moveValue);
        lcd.print(moveValue);
      }
    }
    else{
      if(yValue > .45 && yValue < .55){
        servo1.write(90);
        lcd.print("  90");
      }
      else{
        moveValue = 90 + (yValue - .5) * 180;
        servo1.write(moveValue);
        lcd.print(moveValue);
      }
    }*/

    //Go to next line
    //lcd.setCursor(0, 1);
    //lcd.print(moveValue);
}
