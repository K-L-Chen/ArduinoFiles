#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"
#include <eloquent_esp32cam.h>
//#include <analogWrite.h>
#include <ESP32Servo.h>
#include <esp_now.h>

//#define CAMERA_MODEL_WROVER_KIT
#include "define_camera.h"

#if defined(CAMERA_MODEL_WROVER_KIT)
  #define MAIN_CAM 1
  #define BUTTON_A 1 //CHANGE
  #define BUTTON_B 2 //CHANGE
  #define CALIBRATE_BUTTON 3 //CHANGE
#elif defined(CAMERA_MODEL_AI_THINKER)
  #define MAIN_CAM 0
#else
  #error "Camera model not supported."
#endif

#define PART_BOUNDARY "123456789000000000000987654321"

bool calibrate_button_pressed = false;
int aLimit, bLimit;

void calibrate(){
  bool aPressed = false;
  bool bPressed = false;
  //0 - left, 1 - right
  int dir = 0;
  while(true){
    if(aPressed && bPressed){
      break;
    }

    if(digitalRead(BUTTON_A) == 1 and !aPressed){
      continue;
      aLimit = dir;
    }

    if(digitalRead(BUTTON_B) == 1 and !bPressed){
      continue;
      bLimit = dir;
    }
  }
  return ret;
}

void setup() {
  // put your setup code here, to run once:
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if(MAIN_CAM){
    if(digitalRead(CALIBRATE_BUTTON) == 1){
      if(!calibrate_button_pressed){
        
      }
    }
  }
}
