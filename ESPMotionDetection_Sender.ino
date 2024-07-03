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

#define CAMERA_MODEL_AI_THINKER // look in camera_pins.h to find the camera type you have
#include "camera_pins.h"        // Must include this after setting
#include "esp_camera.h"
#include "FS.h"     // SD Card ESP32
#include "SD_MMC.h" // SD Card ESP32

#define FRAME_SIZE FRAMESIZE_QVGA // Frambuffer size (matches resolution below)
#define WIDTH 320                 // Resolution Width
#define HEIGHT 240                // Resolution height
#define BLOCK_SIZE 4              // Size of each sensor block on the display (reduced granularity for speed)
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 1.5
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 0                            // Good for making changes
#define INFO 0                             // Good to see whats happening (turn off to save CPU)
#define INFO_SEND 1
#define VIEWPORT_PIXELS WIDTH / BLOCK_SIZE // 320/4 = 80 Positions to look at in left to right view plane [-------X------] X is where motion exists
#define STEPS_PER_DEGREE 6                 // How many degrees per block to turn 11.37
#define FLASH_PIN 4                        // Pin of ESP32 Flash (Led)
#define FLASH_MODE 3                       // 0 = 0ff , 1 = flash, 2 = steady, 3 = special
#include <esp_http_server.h>
#include <WiFi.h>
#include <esp_now.h>
#define PART_BOUNDARY "123456789000000000000987654321"

//Replace with your network credentials
const char* ssid = "ESP32CAM_Server";
const char* password = "test_password1";

uint16_t prev_frame[H][W] = {0};
uint16_t current_frame[H][W] = {0};
uint16_t empty_frame[H][W] = {0};
long motionView[VIEWPORT_PIXELS];

// stepper
const uint8_t MOTOR_IN1_PIN = 12;
const uint8_t MOTOR_IN2_PIN = 13;
const uint8_t MOTOR_IN3_PIN = 14;
const uint8_t MOTOR_IN4_PIN = 15;
int moveTo = 0;

uint8_t receiver_address[] = {0x40, 0x91, 0x51, 0xFB, 0xDB, 0xF0};

esp_now_peer_info_t peerInfo;

typedef struct smsg {
  int moveTo;
  bool motion_detection;
} smsg;

smsg *msg = (smsg *) malloc(sizeof(smsg));

// Scheduler
Scheduler tasks;

void sendInfo();
Task tsendInfo(250, TASK_FOREVER, &sendInfo, &tasks, true);


// === 1 =======================================
void sendInfo(){
  esp_err_t result = esp_now_send(receiver_address, (uint8_t *) msg, sizeof(*msg));

#if INFO
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  Serial.print("Move To: ");
  Serial.println(msg->moveTo);
  Serial.print("Motion Detection: ");
  Serial.println(msg->motion_detection);
#endif

  delay(10);
  return;
}

long viewPortToRegion(long mv[])
{
    int maxVal = 0;
    int region = 0;
    int tmpVal = 0;
    char str_tmp[9];
    // Fill each char arry with the 8 bits of the 10 regions
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            str_tmp[j] = (mv[((i * 8) + j)] == 1) ? '1' : '0';
            //  Serial.println(((i*8)+j));
        }
        tmpVal = readBinaryString(str_tmp);
#if INFO
        Serial.print("Block: ");
        Serial.print(i);
        Serial.print(" STR=");
        Serial.print(str_tmp);
        Serial.print(" Value: ");
        Serial.println(tmpVal);
#endif
        if (tmpVal > maxVal)
        {
            maxVal = tmpVal; // Set new uppper mark
            region = i;      // Which viewport has the most movement.
        }
    }
#if INFO
    Serial.print("Most activity in block region:");
    Serial.println(region);
#endif
    return region;
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
#if INFO_SEND
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif
}

static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
  }
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
    for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++)
    {
        motionView[i] = 0; // Setup Pixel Array
    }
    pinMode(FLASH_PIN, OUTPUT);
    //    Serial.println(setup_camera(FRAME_SIZE,    PIXFORMAT_JPEG) ? "OK" : "ERR INIT");
    Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_GRAYSCALE) ? "OK" : "ERR INIT");

    // stepper.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
    // stepper.setSpeedInStepsPerSecond(STEPS_PER_SECOND);
    // stepper.setAccelerationInStepsPerSecondPerSecond(ACCELLERATION_STEPS_PER_SECOND);
    // // on startup move the full field of view.
    // stepper.moveToPositionInSteps(0);
    // stepper.moveToPositionInSteps(STEPS_PER_DEGREE * 10);
    // stepper.moveToPositionInSteps(0);
    

    WiFi.mode(WIFI_STA);
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);
    
    // Register peer
    memcpy(peerInfo.peer_addr, receiver_address, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add peer        
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
      Serial.println("Failed to add peer");
      return;
    }
    

    // Wi-Fi connection
    WiFi.softAP(ssid, password);
    //  WiFi.begin(ssid, password);
    //  while (WiFi.status() != WL_CONNECTED) {
    //   delay(500);
    //   Serial.print(".");
    //  }
    //  Serial.println("");
    //  Serial.println("WiFi connected");
    
    Serial.print("Camera Stream Ready! Go to: http://");
    Serial.print(WiFi.softAPIP());
    Serial.print("\n");  
    // Start streaming web server
    startCameraServer();

    Serial.println("End Setup...");

    tasks.startNow();
}

/**
 *
 */
void loop()
{
    if (FLASH_MODE == 2)
        digitalWrite(FLASH_PIN, HIGH);
    else digitalWrite(FLASH_PIN, LOW);
    if (!capture_still())
    {
#if INFO
        Serial.println("Failed capture");
#endif
        return;
    }

    msg->motion_detection = motion_detect();
    if (msg->motion_detection)
    {
#if INFO
        Serial.println("Motion detected");
#endif
      if (FLASH_MODE == 3){
        digitalWrite(FLASH_PIN, HIGH);
      }
    }
    update_frame();

    tasks.execute();
}

/**
 * Capture image and do down-sampling
 */
bool capture_still()
{

    if (FLASH_MODE == 1)
        digitalWrite(FLASH_PIN, HIGH);
    camera_fb_t *frame_buffer = esp_camera_fb_get();
    if (FLASH_MODE == 1)
        digitalWrite(FLASH_PIN, LOW);
    if (!frame_buffer)
        return false;
    // set all 0s in current frame
    memcpy(empty_frame, current_frame, sizeof(empty_frame)); // FAST! Memcopy vs iterations so much faster.

    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++)
    {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];
        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
    Serial.println("Current frame:");
    print_frame(current_frame);
    Serial.println("---------------");
#endif

    esp_camera_fb_return(frame_buffer); // Needed to free up camera memory

    return true;
}

/**
 * Compute the number of different blocks
 * If there are enough, then motion happened
 */
bool motion_detect()
{
    uint16_t changes = 0;
    int lastBlock = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);
    for (uint16_t y = 0; y < H; y++)
    {
        for (uint16_t x = 0; x < W; x++)
        {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;

            // Fill only those areas that meet the threashold.
            if (delta >= BLOCK_DIFF_THRESHOLD)
            {
#if DEBUG
                Serial.println(delta);
                Serial.print("diff\t");
                Serial.print(y);
                Serial.print('\t');
                Serial.println(x);
#endif

                motionView[x] = 1;
                changes++;
            }
        }
    }
    if (changes == 0)
        return false; // don't need to go any further

    // Change screen data into linear (left to right) expression of data.

    moveTo = ((10 - viewPortToRegion(motionView)) * 10) * STEPS_PER_DEGREE;
    msg->moveTo = moveTo;

// Display updates for informational purposes.
#if INFO
    Serial.print(":::");
#endif
    // Clear viewport to zero for next detection phase
    for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++)
    {
#if INFO
        Serial.print(motionView[i]);
#endif
        motionView[i] = 0;
    }
#if INFO
    Serial.println(":::");
    Serial.print("Changed ");
    Serial.print(changes);
    Serial.print(" out of ");
    Serial.println(blocks);

    Serial.print("MoveTo:");
    Serial.println(moveTo);
#endif
    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}

/**
 * Copy current frame to previous
 */
void update_frame()
{
    memcpy(prev_frame, current_frame, sizeof(prev_frame)); // FAST! Memcopy vs iterations so much faster.
}

/**
 * For serial debugging
 * @param frame
 */
void print_frame(uint16_t frame[H][W])
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
}

/**
 *  Camera configuration values
 */
bool setup_camera(framesize_t frameSize, pixformat_t PIXEL_FORMAT)
{
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXEL_FORMAT;
    config.frame_size = frameSize;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_camera_deinit();
    bool ok = esp_camera_init(&config) == ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, frameSize);

    return ok;
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

void captureSnapshotToSD()
{
    Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_JPEG) ? "OK" : "ERR INIT");
    esp_camera_fb_get();
    Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_GRAYSCALE) ? "OK" : "ERR INIT");
}
