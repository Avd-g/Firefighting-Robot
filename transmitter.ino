// RECEIVER CODE (Robot ← nRF24L01) 
#include <SPI.h> 
#include <nRF24L01.h> 
#include <IRremote.h> 
#include <RF24.h> 
#include <Servo.h> 
#define LEFT_MOTOR_FORWARD    
2 
#define LEFT_MOTOR_BACKWARD   3 
#define RIGHT_MOTOR_FORWARD   4 
#define RIGHT_MOTOR_BACKWARD  5 
#define IR_LED 8 
#define SERVO_PIN 6 
#define IR_PIN 7 
const byte address[6] = "00001"; 
Servo ladderServo; 
RF24 radio(9, 10); 
struct JoyData { 
int x; 
} __attribute__((packed)); 
int y; 
bool buttonPressed; 
bool servoAt30 = true; 
bool lastButtonState = false; 
bool irBlinking = false; 
unsigned long irBlinkStart = 0; 
unsigned long lastReceiveTime = 0; 
void setup() { 
Serial.begin(9600); 
pinMode(LEFT_MOTOR_FORWARD, OUTPUT); 
pinMode(LEFT_MOTOR_BACKWARD, OUTPUT); 
pinMode(RIGHT_MOTOR_FORWARD, OUTPUT); 
pinMode(RIGHT_MOTOR_BACKWARD, OUTPUT); 
pinMode(IR_LED, OUTPUT); 
ladderServo.attach(SERVO_PIN); 
ladderServo.write(30); 
IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK); 
radio.begin(); 
radio.setPALevel(RF24_PA_MIN); 
radio.setDataRate(RF24_1MBPS); 
radio.setChannel(108); 
radio.setAutoAck(false); 
radio.openReadingPipe(0, address); 
} 
radio.startListening(); 
lastReceiveTime = millis(); 
void loop() { 
if (radio.available()) { 
lastReceiveTime = millis(); 
JoyData joystick; 
radio.read(&joystick, sizeof(joystick)); 
if (joystick.y > 600)       
moveForward(); 
else if (joystick.y < 400)  moveBackward(); 
else                        
else if (joystick.x < 400)  turnLeft(); 
else if (joystick.x > 600)  turnRight(); 
stopMotors(); 
if (joystick.buttonPressed && !lastButtonState) { 
servoAt30 = !servoAt30; 
} 
ladderServo.write(servoAt30 ? 30 : 135); 
delay(50); 

lastButtonState = joystick.buttonPressed; 
} 
if (millis() - lastReceiveTime > 2000) { 
radio.begin(); 
radio.setPALevel(RF24_PA_MIN); 
radio.setDataRate(RF24_1MBPS); 
radio.setChannel(108); 
radio.setAutoAck(false); 
lastReceiveTime = millis(); 
radio.openReadingPipe(0, address); 
radio.startListening(); 
} 
if (IrReceiver.decode()) { 
digitalWrite(IR_LED, HIGH); 
irBlinkStart = millis(); 
} 
irBlinking = true; 
IrReceiver.resume(); 
irBlinking = false; 
if (irBlinking && millis() - irBlinkStart >= 50) { 
digitalWrite(IR_LED, LOW); 
} 
} 
void moveForward() { 
digitalWrite(LEFT_MOTOR_FORWARD, HIGH);  digitalWrite(LEFT_MOTOR_BACKWARD, 
LOW); 
digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); digitalWrite(RIGHT_MOTOR_BACKWARD, 
LOW); 
} 
void moveBackward() { 
digitalWrite(LEFT_MOTOR_FORWARD, LOW);   digitalWrite(LEFT_MOTOR_BACKWARD, 
HIGH); 
digitalWrite(RIGHT_MOTOR_FORWARD, LOW);  digitalWrite(RIGHT_MOTOR_BACKWARD, 
HIGH); 
} 

void turnLeft() { 
digitalWrite(LEFT_MOTOR_FORWARD, LOW);   digitalWrite(LEFT_MOTOR_BACKWARD, 
HIGH); 
digitalWrite(RIGHT_MOTOR_FORWARD, HIGH); digitalWrite(RIGHT_MOTOR_BACKWARD, 
LOW); 
} 
void turnRight() { 
digitalWrite(LEFT_MOTOR_FORWARD, HIGH);  digitalWrite(LEFT_MOTOR_BACKWARD, 
LOW); 
digitalWrite(RIGHT_MOTOR_FORWARD, LOW);  digitalWrite(RIGHT_MOTOR_BACKWARD, 
HIGH); 
} 
void stopMotors() { 
digitalWrite(LEFT_MOTOR_FORWARD, LOW); 
digitalWrite(LEFT_MOTOR_BACKWARD, LOW); 
digitalWrite(RIGHT_MOTOR_FORWARD, LOW); 
digitalWrite(RIGHT_MOTOR_BACKWARD, LOW); 
}