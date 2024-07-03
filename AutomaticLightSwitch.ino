const int sensorDark = 650; //350 = complete darkness, 500 = baseline, 880 = always on

int photocellPin = A0;
int LEDPin = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(LEDPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  int analogValue;
  
  //String s = getSTR();
  //Make a library for this stuff
  /*switch (s){
    case "dark":
      digitalWrite(LEDPin, HIGH);
      break;
    default:
      digitalWrite(LEDPin, LOW);
      break;
  }*/
  
  analogValue = analogRead(photocellPin);
  
  if(analogValue < sensorDark){
    digitalWrite(LEDPin, HIGH);
  }
  else{
    digitalWrite(LEDPin, LOW);
  }

  delay(1);
}
