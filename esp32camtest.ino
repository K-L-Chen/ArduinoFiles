#define ONBOARD_LED  4


//update May 24, 2024
//Small, two piece board is AI Thinker ESP32-CAM
//  needs driver https://sparks.gogo.co.nz/ch340.html
//Large, one piece board is ESP32-Wrover
//the WROOM board is "FireBeetle-ESP32", somehow
void setup() {
  pinMode(ONBOARD_LED,OUTPUT);
  Serial.begin(115200);
  Serial.println("STARTING");
}

void loop() {
  delay(5000);
  digitalWrite(ONBOARD_LED,HIGH);
  delay(5000);
  digitalWrite(ONBOARD_LED,LOW);
}
