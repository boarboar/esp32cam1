/*
  Based on - 
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-post-image-photo-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include "esp_task_wdt.h"
#include "connect.inc"

// const char*  serverPath = "/upload";     
// const char*  serverName_Local = "192.168.1.138";   
// const int serverPort_Local = 51062;
// const char*  serverName_Remote = "193.70.73.242";   
// const int serverPort_Remote = 51062;

const int CamID = 1;

WiFiClient client;

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED_PIN           33
#define FLASH_PIN         4
#define MODE_PIN          16

const int timerInterval = 30000;    // time between each HTTP POST image
unsigned long previousMillis = 0;   // last time image was sent
String  serverName = serverName_Local;   
int serverPort = serverPort_Local;
String content_head = "--IMG\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"CAM-";
String content_head_2 = ".jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
String content_tail = "\r\n--IMG--\r\n";
String content_start = "Content-Type: multipart/form-data; boundary=IMG";


bool sendPhoto();

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  pinMode(LED_PIN, OUTPUT);
  pinMode(MODE_PIN, INPUT_PULLUP);

  Serial.begin(115200);

  if(digitalRead(MODE_PIN) == HIGH) {
    Serial.print("Remote mode");
    serverName = serverName_Remote;   
    serverPort = serverPort_Remote;
  } else {
    Serial.print("Local mode");
    serverName = serverName_Local;   
    serverPort = serverPort_Local;
  }

  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, LOW);
    Serial.print(".");
    delay(500);
    digitalWrite(LED_PIN, HIGH);
  }

  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());

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
  config.pixel_format = PIXFORMAT_JPEG;

  // init with high specs to pre-allocate larger buffers
  if(psramFound()){
    Serial.println("PSRAM");
    //config.frame_size = FRAMESIZE_SVGA;
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    Serial.println("NO PSRAM");
    //config.frame_size = FRAMESIZE_CIF;
    //config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }
  
  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

 sensor_t * s = esp_camera_sensor_get();

 //if (NULL != s && s->id.PID == OV3660_PID) {
 if (NULL != s) {
    Serial.println("Setting camera params");
    //s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -1);//lower the saturation
    s->set_contrast(s, 1);
    s->set_awb_gain(s, 0);  
    s->set_lenc(s, 0);
  }

  esp_task_wdt_init(timerInterval*3, true);
  esp_task_wdt_add(NULL); // add current thread to WDT
  
  content_head += String(CamID);
  content_head += content_head_2;

  sendPhoto(); 
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= timerInterval) {
    digitalWrite(LED_PIN, LOW);
    sendPhoto();
    digitalWrite(LED_PIN, HIGH);
    previousMillis = currentMillis;
    esp_task_wdt_reset();
  }
}

bool sendPhoto() {

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if(!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return false;
  }
  
  Serial.println("Connecting to server: " + serverName);

  if (client.connect(serverName.c_str(), serverPort)) {
    Serial.println("Connection successful!");    

    uint32_t imageLen = fb->len;
    uint32_t extraLen = content_head.length() + content_tail.length();
    uint32_t totalLen = imageLen + extraLen;
  
    client.println("POST " + String(serverPath) + " HTTP/1.1");
    client.println("Host: " + serverName);
    client.println("Content-Length: " + String(totalLen));
    client.println("Authorization : Bearer whatever");
    client.println(content_start); // IMG multipart start
    client.println();
    client.print(content_head);  // IMG multipart head
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n=0; n<fbLen; n=n+1024) {
      if (n+1024 < fbLen) {
        client.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen%1024>0) {
        size_t remainder = fbLen%1024;
        client.write(fbBuf, remainder);
      }
    }   
    client.print(content_tail); //IMG multipart tail
    client.flush();
    client.stop();
    Serial.println("Completed.");
  }
  else {
    Serial.println("Connection to " + serverName +  " failed.");
  }

  esp_camera_fb_return(fb);

  Serial.println("Free head: ");
  Serial.println(ESP.getFreeHeap());

  return true;
}
