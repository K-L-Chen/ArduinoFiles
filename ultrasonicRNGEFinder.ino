int echoPin = 7;
int trigPin = 8;
int lightPin = 6;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(lightPin, OUTPUT);
}

// max distance the rangefinder can calculate is 3 meters
void loop() {
  // put your main code here, to run repeatedly:
  float distanceMeters;
  int pulseLenMicroseconds;
  // 2938.66995798 Microseconds/Meter

  //digitalWrite(lightPin, LOW);
  
  //check range for rangefinder
  digitalWrite(trigPin, LOW);
  delayMicroseconds(20);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(100);
  digitalWrite(trigPin, LOW);

  pulseLenMicroseconds = pulseIn(echoPin, HIGH);
  distanceMeters = (float)(pulseLenMicroseconds) / 2938.66995798;
  distanceMeters /= 2; //sound must travel back, so divide previous num by 2

  Serial.println((String)(distanceMeters) + " meters");

  if(distanceMeters <= 0.25){
    digitalWrite(lightPin, HIGH);
    delay(1000);
  }
  else{
    digitalWrite(lightPin, LOW);
    delay(1000);
  }
}
