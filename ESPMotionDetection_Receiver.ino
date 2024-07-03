/**
   ESP32-CAM Directional Motion Detection (https://www.youtube.com/watch?v=NIbiG6at01g)
   Learn more on Makers Mashup - https://youtube.com/MakersMashup
   Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
   http://creativecommons.org/licenses/by-sa/4.0/

   PLEASE NOTE YOU WILL NEED TO ADD THE TaskSchedule and TinySTepper libarires to 
   run this code unaltered.  Use TOOLS->Manage Libraries in the arduino IDE.
   Also use the board manager and make sure you select ESP32 as an available
   board. The one I used specifically was https://amzn.to/3m2dJrl
   
*/
#include <TaskScheduler.h>
#include <TinyStepper_28BYJ_48.h>


#define DEBUG 0                            // Good for making changes
#define INFO 1                             // Good to see whats happening (turn off to save CPU)
#define INFO_DATA 1
#define STEPS_PER_DEGREE 6                 // How many degrees per block to turn 11.37
#define STEPS_PER_SECOND 4096              // How fast the stepper turns
#define ACCELLERATION_STEPS_PER_SECOND 256 // How quickly the movement accellerates
#define FLASH_PIN 4                        // Pin of ESP32 Flash (Led)
#define FLASH_MODE 0                       // 0 = 0ff , 1 = flash, 2 = steady

#include <esp_now.h>
#include <WiFi.h>

// uint16_t prev_frame[H][W] = {0};
// uint16_t current_frame[H][W] = {0};
// uint16_t empty_frame[H][W] = {0};
// long motionView[VIEWPORT_PIXELS];

// stepper
const uint8_t MOTOR_IN1_PIN = 12;
const uint8_t MOTOR_IN2_PIN = 13;
const uint8_t MOTOR_IN3_PIN = 14;
const uint8_t MOTOR_IN4_PIN = 15;
int moveTo = 0;
int currentPos = 0;
bool motion_detected = false;

typedef struct smsg {
  int moveTo;
  bool motion_detection;
} smsg;
smsg *received = (smsg *) malloc(sizeof(smsg));

TinyStepper_28BYJ_48 stepper;

// Scheduler
Scheduler tasks;

void receive_data();
Task trcv_data(250, TASK_FOREVER, &receive_data, &tasks, true);
void moveSteppers();
Task tmvStepper(250, TASK_FOREVER, &moveSteppers, &tasks, true);

// === 1 =======================================
void receive_data()
{
  moveTo = received->moveTo;
  motion_detected = received->motion_detection;
}

void moveSteppers()
{
  if (motion_detected){
    // Stepper Movement - Move by steps so we capture more motion frames.
    if (currentPos > moveTo)
    {
        currentPos -= ceil((currentPos - moveTo) / 2);
        if (ceil((currentPos - moveTo) / 2) == 0)
            currentPos = moveTo;
    }
    else if (currentPos < moveTo)
    {
        currentPos += floor((moveTo - currentPos) / 2);
        if (floor((moveTo - currentPos) / 2) == 0)
            currentPos = moveTo;
    }
    else
    {
        currentPos = moveTo;
    }
#if INFO
    if (currentPos != moveTo)
    {
        Serial.print("MOVETO: ");
        Serial.print(moveTo);
        Serial.print(" CURRENT: ");
        Serial.println(currentPos);
    }
#endif
    stepper.moveToPositionInSteps(currentPos);
  }
}

void OnDataRecv(const esp_now_recv_info *macs, const uint8_t *incomingData, int len) {
  memcpy(&received, incomingData, sizeof(received));
#if INFO_DATA
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Move To: ");
  Serial.println(received->moveTo);
  Serial.print("Motion Detected?: ");
  Serial.println(received->motion_detection);
#endif
}

/**
 *
 */
void setup()
{
    uint32_t Freq = 0;
    Serial.begin(115200);
    Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getXtalFrequencyMhz();
    Serial.print("XTAL Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getApbFrequency();
    Serial.print("APB Freq = ");
    Serial.print(Freq);
    Serial.println(" Hz");
    Serial.println("Begin Setup...");

    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    esp_now_register_recv_cb(OnDataRecv);

    pinMode(FLASH_PIN, OUTPUT);

    stepper.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
    stepper.setSpeedInStepsPerSecond(STEPS_PER_SECOND);
    stepper.setAccelerationInStepsPerSecondPerSecond(ACCELLERATION_STEPS_PER_SECOND);
    // on startup move the full field of view.
    stepper.moveToPositionInSteps(0);
    stepper.moveToPositionInSteps(STEPS_PER_DEGREE * 10);
    stepper.moveToPositionInSteps(0);
    Serial.println("End Setup...");

    tasks.startNow();
}

/**
 *
 */
void loop()
{
    tasks.execute();
}

int readBinaryString(char *s)
{
    int result = 0;
    while (*s)
    {
        result <<= 1;
        if (*s++ == '1')
            result |= 1;
    }
    return result;
}