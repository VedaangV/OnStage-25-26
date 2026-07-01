//file include
#include "movement.h"
#include "common.h"

//libraries
#include <Wire.h>
#include <WiFi.h>

#define ENABLE_WIFI

WiFiServer server(5000); 
const char* ssid = "jetson";
const char* password = "todbot1234";

float vx = 0.0;
float vy = 0.0;
float vx_act = 0.0;
float vy_act = 0.0;
bool haswater = false;

#define LED 21  //LED on Pico W to indicate Wifi connection
String inputBuffer = "";
void setup() {
  //Serial
  Serial.begin(115200);
  delay(2500);

  //Neopixel
  strip.begin();
  strip.show();

  //Wire
  Wire.setSDA(16);
  Wire.setSCL(17);
  Wire.begin();
  
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(direction_pin, INPUT);
  pinMode(encoder_pin, INPUT);
  attachInterrupt(encoder_pin, enc_update, RISING);

  delay(500);

  //IMU BNO055
  if (!bno.begin(OPERATION_MODE_IMUPLUS)) {
    Serial.println("IMU not working");
    for (;;){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  Serial.println("IMU working");
  
  //Wifi communication
#ifdef ENABLE_WIFI
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");
  server.begin();
  digitalWrite(LED_BUILTIN, HIGH);
#else
#endif

  //reset encoders
  encoders = 0;

  //START indicator
  for(int i = 0; i < 4; i++) {
    resetNPixel();
    delay(500);
    fillNPixel(WHITE);
    delay(500);
  } 

  //get IMU initial value
  sensors_event_t event;
  bno.getEvent(&event);
  rotation_offset = ((int) event.orientation.x);

  upd_rotation();
}

void loop() {
#ifdef ENABLE_WIFI
  WiFiClient client = server.available(); // Check for a new client connection
  Serial.println(WiFi.localIP());
  if (client) {
    Serial.println("\nNew Client Connected!");
    while (client.connected()) { // While the client is connected
      if (client.available()) { // If there is data available to read
        char c = client.read(); // Read a character
        if (c == '\n') {
          if(inputBuffer.indexOf("stop") != -1){
            inputBuffer = "";
            motor(0,0,0);
          }
          if(inputBuffer.indexOf("deplete") != -1){
            inputBuffer = "";
            motor(0,0,0);
            wipeNPixel(3000, ICEBLUE, WHITE);
            haswater = false;
            delay(500);
          }
          else if (inputBuffer.indexOf("collect") != -1) {
            inputBuffer = "";
            motor(0,0,0);
            wipeNPixel(3000, WHITE, ICEBLUE);
            haswater = true;
            delay(500);
          }
          else if (inputBuffer.indexOf("dust") != -1) {
            inputBuffer = "";
            motor(0,0,0);
            resetNPixel();
            for(int i = 0; i < 4; i++) {
              resetNPixel();
              delay(500);
              fillNPixel(RED);
              delay(500);
            } 
            delay(1000);
          }
          else if (inputBuffer.indexOf("enter") != -1) {
            inputBuffer = "";
            for (int i = 0; i < 600; i++) {
              vmotor(0, -0.4);
              delay(1);
            }
            motor(0,0,0);
            client.println("done");
          }
          else if (inputBuffer.indexOf("exit") != -1) {
            inputBuffer = "";
            for (int i = 0; i < 700; i++) {
              vmotor(0, 0.4);
              delay(1);
            }
            motor(0,0,0);
            if (haswater == true) {
              fillNPixel(ICEBLUE);
            }
            else {
              fillNPixel(WHITE);
            }
            client.println("done");
          }
          else if (inputBuffer.indexOf("end") != -1) {
            inputBuffer = "";
            motor(0,0,0);
            wipeNPixel(3000, WHITE, GREEN);
            delay(500);
          }
          else if (inputBuffer.indexOf("v") != -1) {
            parseVelocity(inputBuffer);
            Serial.println(vx);
            Serial.println(vy);
            inputBuffer = "";
            vmotor(vx, vy, &vx_act, &vy_act); 
            String str = String(vx_act) + ", " + String(vy_act);
            client.println(str);
            vx_act = 0;
            vy_act = 0;
          }
          else {
            inputBuffer = "";
            motor(0, 0, 0);
          }
        } 
        else {
          inputBuffer += c;
        } 
      }
    }
    Serial.println("Client Disconnected.");
    client.stop();
  }
#else

  //*** TESTING ***//
  //test individual motor config
  // delay(750);
  // motor(0, 0, 30);
  // delay(750);

  // motor(0, 0, -30);
  // delay(750);
  // motor(0, -30, 0);
  // delay(750);
  // motor(-30, 0, 0);
  // delay(750);

  //test overall motor config
  // for (int t = 0; t < 1000; t++) {
  //   vmotor(0.4*cos(2*PI*t/2000), 0.4*sin(2*PI*t/2000));
  //   delay(1);
  // }
  // for (int t = 0; t < 2000; t++) {
  //   vmotor(-0.4*cos(2*PI*t/2000), 0.4*sin(2*PI*t/2000));
  //   delay(1);
  // }
  // for (int t = 0; t < 1000; t++) {
  //   vmotor(-0.4*cos(2*PI*t/2000), -0.4*sin(2*PI*t/2000));
  //   delay(1);
  // }

  // for (int i = 0; i < 1000; i++) {
  //   vmotor(-0.4, 0);
  //   delay(1);
  // }
  // for (int i = 0; i < 1000; i++) {
  //   vmotor(0.4, 0);
  //   delay(1);
  // }
  // for (int i = 0; i < 1000; i++) {
  //   vmotor(0, 0.4);
  //   delay(1);
  // }
  // for (int i = 0; i < 1000; i++) {
  //   vmotor(0, -0.4);
  //   delay(1);
  // }

#endif
}

void parseVelocity(String data) {
  int xpos = data.indexOf("x");
  int ypos = data.indexOf("y");
  if (xpos == -1 || ypos == -1) return;

  String vxStr = data.substring(data.indexOf("x:") + 3, ypos-3);
  String vyStr = data.substring(data.indexOf("y:") + 3, data.length());

  vxStr.trim();
  vyStr.trim();

  vx = vxStr.toFloat();
  vy = vyStr.toFloat();
}
