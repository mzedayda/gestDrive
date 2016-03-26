#include <VirtualWire.h>

#define RF_LED_PIN 2
#define TRANSMIT_PIN 12
#define RECEIVE_PIN 11
#define PTT_PIN 4

#define FORWARD 7
#define BACKWARD 8
#define RIGHT 9
#define LEFT 10
#define PWM_FB 3
#define PWM_RL 6
#define STBY 5

int count = 0;
bool standby;

void setup() {
  delay(1000);
  Serial.begin(9600);  // Debugging only
  Serial.println("setup");
  
  // Initialise RF transmitter
  vw_set_tx_pin(TRANSMIT_PIN);
  vw_set_rx_pin(RECEIVE_PIN);
  vw_set_ptt_pin(PTT_PIN);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);   // Bits per sec

  vw_rx_start();       // Start the receiver PLL running

  // configure Motor driver pins
  pinMode(RF_LED_PIN, OUTPUT);
  pinMode(FORWARD, OUTPUT);
  pinMode(BACKWARD, OUTPUT);
  pinMode(RIGHT, OUTPUT);
  pinMode(LEFT, OUTPUT);
  pinMode(PWM_FB, OUTPUT);
  pinMode(PWM_RL, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);
  standby = false;
}

void loop() {
  uint8_t msg[VW_MAX_MESSAGE_LEN];
  uint8_t msgLength = VW_MAX_MESSAGE_LEN;
  if (vw_get_message(msg, &msgLength)) { // Non-blocking
    Serial.print("Got: ");          // Message with a good checksum received, print it.
    
    for (int i = 0; i < msgLength; i++) {
      Serial.print(msg[i], HEX);
      Serial.print(' ');
    }
    Serial.println();

    if (validatePacket(msg, msgLength)) {
      digitalWrite(RF_LED_PIN, LOW);
      count = 0;
      if (standby) {
        digitalWrite(STBY, LOW);
        standby = false;
      }
      driveMotors(msg[1], msg[2]);
    }
    else {
      Serial.println("Invalid packet");
      digitalWrite(RF_LED_PIN, HIGH);
    }
  }

  delay(1);
  if(count++ > 60) stopMotors();
}

void driveMotors(uint8_t fb, uint8_t rl) {
  drivefb(fb);
  driverl(rl);
}

void drivefb(uint8_t fb) {
  bool forward = fb >= 127 ? true : false;
  digitalWrite(FORWARD, forward);
  digitalWrite(BACKWARD, !forward);
  uint8_t speed = abs(fb - 127);
  speed = map(speed, 0, 127, 0, 255);
  analogWrite(PWM_FB, speed);
}

void driverl(uint8_t rl) {
  bool right = rl >= 127 ? true : false;
  digitalWrite(RIGHT, right);
  digitalWrite(LEFT, !right);
  uint8_t speed = abs(rl - 127);
  speed = map(speed, 0, 127, 0, 255);
  analogWrite(PWM_RL, speed);
}

void stopMotors() {
  digitalWrite(STBY, HIGH);
  standby = true;
}

bool validatePacket(uint8_t msg[VW_MAX_MESSAGE_LEN], uint8_t msgLength) {
  if (msgLength != 4) return false;
  if (msg[0] == 128 && msg[3] == 126) return true;
  return false;
}

