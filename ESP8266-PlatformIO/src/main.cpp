/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/* This driver reads quaternion data from the MPU6060 and sends
   Open Sound Control messages.

  GY-521  NodeMCU
  MPU6050 devkit 1.0
  board   Lolin         Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D5 (GPIO14)   Interrupt pin

*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1    // Reset pin # (or -1 if sharing reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5/3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ESP8266 GPIO15
   pin.
 * ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z]         quaternion container

float euler[3]; // [psi, theta, phi]    Euler angle container

#define ssid "Xiaomi_6D79"
#define pass "Lhp198231"

const char *mqtt_server = "192.168.0.18";
const char *client_id = "test-001";   // 标识当前设备的客户端编号
const char *mqtt_user = "admin";      //MQTT用户名
const char *mqtt_password = "public"; //MQTT密码
const char *topic = "test";           //TOPIC主题
WiFiClient espClient;
PubSubClient client(espClient);

#define INTERRUPT_PIN 14 // use pin 15 on ESP8266

const char DEVICE_NAME[] = "mpu6050";

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void setup(void)
{
  WiFi.mode(WIFI_STA);
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.display();
  delay(2);
  display.clearDisplay();

  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1.5);
  display.setCursor(0, 0);
  display.println("Initializing...");
  display.display();
  WiFi.begin(ssid, pass);
  Serial.println("Connecting...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected,IP Address:");
  Serial.println(WiFi.localIP());
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1.5);
  display.setCursor(0, 0);
  display.println("Wifi Connected.");
  client.setServer(mqtt_server, 1883);
  while (!client.connected())
  {
    Serial.println("MQTT connecting...");
    if (client.connect(client_id, mqtt_user, mqtt_password))
    {
      Serial.println("MQTT connect success.");
      display.setTextColor(WHITE);
      display.setTextSize(1.5);
      display.setCursor(0, 18);
      display.println("MQTT Connected.");
    }
    else
    {
      delay(5000);
    }
  }
  display.display();
  mpu_setup();
}

void mpu_loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady)
    return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize)
    return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("euler: ");
    float x, y, z;
    x = euler[0] * 180 / M_PI;
    y = euler[1] * 180 / M_PI;
    z = euler[2] * 180 / M_PI;
    Serial.print(euler[0] * 180 / M_PI);
    Serial.print(" , ");
    Serial.print(euler[1] * 180 / M_PI);
    Serial.print(" , ");
    Serial.println(euler[2] * 180 / M_PI);
    String xx, yy, zz, sum;
    xx = String(x);
    yy = String(y);
    zz = String(z);
    sum = xx + "," + yy + "," + zz;
    char *reschar = (char *)sum.c_str(); //转换为char*
    client.publish(topic, reschar);      //发送MQTT消息
    display.clearDisplay();
    display.setTextSize(2.0);
    display.setCursor(0, 0);
    display.println("WifiCube");

    display.setTextSize(1.5);
    display.setCursor(0, 20);
    display.print("X: ");
    display.println(euler[0] * 180 / M_PI);

    display.setTextSize(1.5);
    display.setCursor(0, 35);
    display.print("Y: ");
    display.println(euler[1] * 180 / M_PI);

    display.setTextSize(1.5);
    display.setCursor(0, 50);
    display.print("Z: ");
    display.println(euler[2] * 180 / M_PI);

    display.display();
  }
}

void loop(void)
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println();
    Serial.println("*** Disconnected from AP so rebooting ***");
    Serial.println();
    ESP.reset();
  }
  client.loop();
  mpu_loop();
  delay(20);
}