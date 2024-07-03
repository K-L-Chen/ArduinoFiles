// include the library
#include <LiquidCrystal.h>

// all of our LCD pins
int lcdRSPin = 12;
int lcdEPin = 11;
int lcdD4Pin = 5;
int lcdD5Pin = 4;
int lcdD6Pin = 3;
int lcdD7Pin = 2;

//other pins
int echoPin = 7;
int trigPin = 8;
int lightPin = 6;
int lm35Pin = A5;
int buttonPin = 9;

//array variable
//float arr [3] = {getTime(),rangefinder(),temperature()};
String arr [3] = {" ", " ", " "};
int index = 0;
//other
String clearLine = "                ";

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(lcdRSPin, lcdEPin,
                  lcdD4Pin, lcdD5Pin, lcdD6Pin, lcdD7Pin);

void setup()
{
    Serial.begin(9600);
    
    // set up the LCD's number of columns and rows: 
    lcd.begin(16, 2);
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(lightPin, OUTPUT);
    pinMode(buttonPin, INPUT_PULLUP);
    // Print a message to the LCD.
    //lcd.print("hello, world!");
}



double roundNearestPlace(float val, int place){
  int temp = (int)(val*(pow(10.0, place)));
  if(temp%10 >= 5){
    temp = temp/10 + 1;
  }
  else{
    temp = temp/10;
  }
  return (float)(temp)/(float)(pow(10,place-1));
}

double getTime(){
  return roundNearestPlace((double)(millis()/1000.0),3);
}


float rangefinder(){
    float distanceMeters;
    int pulseLenMicroseconds;

    digitalWrite(trigPin, LOW);
    delayMicroseconds(20);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(trigPin, LOW);
  
    pulseLenMicroseconds = pulseIn(echoPin, HIGH);
    distanceMeters = (float)(pulseLenMicroseconds) / 2938.66995798;
    distanceMeters /= 2; //sound must travel back, so divide previous num by 2

    if(distanceMeters <= 0.25){
      digitalWrite(lightPin, HIGH);
    }
    else{
      digitalWrite(lightPin, LOW);
    }
    return roundNearestPlace(distanceMeters,3);
    //lcd.setCursor(0, 2);
    //Serial.println((String)(distanceMeters) + " meters");
}

float temperature(){
    int analogValue;
    float temperature;

    // read our temperature sensor
    analogValue = analogRead(lm35Pin);

    // convert the 10bit analog value to celcius
    temperature = float(analogValue) / 1023;
    temperature = temperature * 500;

    return roundNearestPlace(temperature,2);
}

void updateArray(){
  arr[0] = "Time: " + (String)getTime() + "s";
  arr[1] = "Range: " + (String)rangefinder() + "m";
  arr[2] = "Temp: " + (String)temperature() + "*C";
}

void loop(){
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0)

    updateArray();
    
    int button = digitalRead(buttonPin);
    if(button == LOW){
      //Serial.print(index);
      index++;
      //Serial.print(index);
      //delay(1000);
      lcd.setCursor(0, 0);
      lcd.print(clearLine);
      lcd.setCursor(0,1);
      lcd.print(clearLine);
      //delay(1000);
    }
    
    lcd.setCursor(0, 0);
    lcd.print(arr[index % 3]);
    /*float range = rangefinder();
    if(range <= 0.25){
      digitalWrite(lightPin, HIGH);
    }
    else{
      digitalWrite(lightPin, LOW);
    }
    lcd.print("Range: " + (String)(range) + "m");*/
    
    lcd.setCursor(0,1);
    lcd.print(arr[(index + 1) % 3]);
    /*float temp = temperature();
    lcd.print("Temp: " + (String)(temp) + "*C");
    
    delay(1000);*/
}
