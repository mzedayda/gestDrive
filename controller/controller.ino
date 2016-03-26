#include <VirtualWire.h>

#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

#define GYRO_LED_PIN 13
#define RF_LED_PIN 11
#define TRANSMIT_PIN 12
#define RECEIVE_PIN 2
#define PTT_PIN 3

#define FB_THRESH 1000
#define RL_THRESH 1000
#define FB_MAX 13000
#define RL_MAX 13000

int count = 0;

bool blinkState = false;

void setup() {
  // Initialise RF transmitter
  vw_set_tx_pin(TRANSMIT_PIN);
  vw_set_rx_pin(RECEIVE_PIN);
  vw_set_ptt_pin(PTT_PIN);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);   // Bits per sec

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values
  /*
  Serial.println("Updating internal sensor offsets...");
  // -76  -2359 1688  0 0 0
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(220);
  accelgyro.setYGyroOffset(76);
  accelgyro.setZGyroOffset(-85);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  */

  // configure Arduino LED for
  pinMode(GYRO_LED_PIN, OUTPUT);
  pinMode(RF_LED_PIN, OUTPUT);
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);

  #ifdef OUTPUT_READABLE_ACCELGYRO
    // display tab-separated accel/gyro x/y/z values
    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
  #endif

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(GYRO_LED_PIN, blinkState);

  byte msg[4] = {128, 127, 127, 126};
  msg[1] = power(ax, FB_THRESH, FB_MAX);
  msg[2] = power(ay, RL_THRESH, RL_MAX);

  digitalWrite(RF_LED_PIN, HIGH); // Flash a light to show transmitting
  vw_send(msg, 4);
  vw_wait_tx();
  digitalWrite(RF_LED_PIN, LOW);
}

int power(int accel, int thresh, int ceiling) {
  bool pos = accel > 0 ? true : false;
  accel = constrain(abs(accel), 0, ceiling);
  if (accel < thresh) return 127;                      // Neutral
  int power = map(accel, thresh, ceiling, 0, 127);
  return pos ? 127 + power : 127 - power;
}

