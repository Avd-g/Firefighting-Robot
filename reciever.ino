/* ========================================================= 
 *  FIREFIGHTER-GRID BOT – with NRF24L01 debug logging and fire detection 
 * =======================================================*/ 
#include <NewPing.h> 
#include <Servo.h> 
#include <SPI.h> 
#include <RF24.h> 
/* ---------- USER-TUNE CONSTANTS ------------------------------- */ 
#define GRID_ROWS 8 
#define GRID_COLS 8 
const uint16_t CELL_TIME_GRID       = 1400; 
const uint16_t CELL_TIME_WALL_FIRST = 900; 
const uint16_t CELL_TIME_WALL       = 1400; 
const uint16_t TURN_LEFT_MS  = 930; 
const uint16_t TURN_RIGHT_MS = 1700; 
const uint16_t ROW_TURN_LEFT_MS  = 1300; 
const uint16_t ROW_TURN_RIGHT_MS = 1500; 
const uint16_t HALF_CELL_MS      = CELL_TIME_GRID / 2; 
const uint16_t POST_ADVANCE_MS   = CELL_TIME_WALL; 
const uint16_t MOVE_SLICE        = 80; 
const uint16_t APPROACH_BURST_MS = 60; 
const uint16_t DETECT_FRONT_CM = 25; 
const uint16_t STOP_FRONT_CM   = 6; 
const uint16_t CRAWL_THRESHOLD_CM = 15; 
const uint8_t FWD_SPEED   = 160; 
const uint8_t TURN_SPEED  = 170; 
const uint8_t CRAWL_SPEED = 110; 
/* ---------- PIN MAP ---------------------------------------- */ 

#define IN1 2 
#define IN2 3 
#define IN3 4 
#define IN4 5 
#define ENA 11 
#define ENB 12 
#define IR_LEFT_RX 7 
#define TRIG_PIN 9 
#define ECHO_PIN 10 
#define LADDER_SERVO_PIN 33 
#define MAX_DIST_CM 120 
#define RF_CE_PIN  44 
#define RF_CSN_PIN 45 
Servo ladderServo; 
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DIST_CM); 
RF24 radio(RF_CE_PIN, RF_CSN_PIN); 
const byte ADDRESS[6] = "BOT01"; 
enum BotState { SERP_STEP, APPROACH_WALL, PERIM_SCAN, DEPLOY_LADDER }; 
BotState state = SERP_STEP; 
enum Heading { NORTH, EAST, SOUTH, WEST }; 
Heading heading = NORTH; 
int row = 0, col = 0; 
bool sweepRight = true; 
uint8_t lastFireWall = 255; 
inline void stopMotors(){ digitalWrite(IN1,0); digitalWrite(IN2,0); 
digitalWrite(IN3,0); digitalWrite(IN4,0); } 
inline void setLeft (int p){ analogWrite(ENA,abs(p)); digitalWrite(IN1,p>0); 
digitalWrite(IN2,p<=0); } 
inline void setRight(int p){ analogWrite(ENB,abs(p)); digitalWrite(IN3,p>0); 
digitalWrite(IN4,p<=0); } 
inline void fwdFast (){ setLeft( FWD_SPEED ); setRight( FWD_SPEED  ); } 
inline void fwdCrawl(){ setLeft( CRAWL_SPEED); setRight( CRAWL_SPEED); } 
inline void rev     (){ setLeft(-FWD_SPEED ); setRight(-FWD_SPEED ); } 
inline void leftPivot (uint16_t ms = TURN_LEFT_MS ){ setLeft( TURN_SPEED); 
setRight(-TURN_SPEED); delay(ms); stopMotors(); } 

inline void rightPivot(uint16_t ms = TURN_RIGHT_MS){ setLeft(-TURN_SPEED); 
setRight( TURN_SPEED); delay(ms); stopMotors(); } 
void sendLog(const char* message) { 
radio.write(&message, strlen(message) + 1); 
} 
inline uint16_t pingFront(){ return sonar.ping_cm(); } 
inline bool firePulse(uint16_t ms = 25){ 
unsigned long t0 = millis(); 
while(millis() - t0 < ms){ 
if(!digitalRead(IR_LEFT_RX)) return true; 
} 
} 
return false; 
void advanceOneCell(){ 
fwdFast(); delay(CELL_TIME_GRID); stopMotors(); 
if(heading==NORTH) row++; 
else if(heading==SOUTH) row--; 
} 
else if(heading==EAST) col++; 
else col--; 
void rowChange(bool atRight){ 
if(atRight){ 
leftPivot(ROW_TURN_LEFT_MS); 
fwdFast(); 
delay(HALF_CELL_MS); 
stopMotors(); 
leftPivot(ROW_TURN_LEFT_MS); heading = WEST; sweepRight = false; 
} else { 
rightPivot(ROW_TURN_RIGHT_MS); 
fwdFast(); 
stopMotors(); 
delay(HALF_CELL_MS); 
rightPivot(ROW_TURN_RIGHT_MS); heading = EAST; sweepRight = true; 
} 
row++; 
} 
bool driveAndScanRight(uint16_t ms){ 
uint16_t loops = ms / MOVE_SLICE; 
for(uint16_t i=0;i<loops;i++){ 
    fwdFast(); delay(MOVE_SLICE); 
    if(firePulse()){ stopMotors(); return true; } 
  } 
  stopMotors(); return false; 
} 
void postLadderMove(){ 
  switch(lastFireWall){ 
  case 0: driveAndScanRight(POST_ADVANCE_MS); rightPivot(); 
driveAndScanRight(CELL_TIME_WALL); rightPivot(); 
driveAndScanRight(CELL_TIME_WALL/2); rightPivot(); break; 
  case 1: driveAndScanRight(POST_ADVANCE_MS); rightPivot(); 
driveAndScanRight(CELL_TIME_WALL/2); rightPivot(); break; 
  case 2: driveAndScanRight(POST_ADVANCE_MS); rightPivot(); break; 
  case 3: rev(); delay(CELL_TIME_WALL/2); stopMotors(); leftPivot(); 
fwdFast(); delay(CELL_TIME_WALL/2); stopMotors(); leftPivot(); break; 
  default: rev(); delay(CELL_TIME_WALL); stopMotors(); rightPivot(); 
fwdFast(); delay(CELL_TIME_WALL/2); stopMotors(); rightPivot(); break; 
  } 
} 
void setup(){ 
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); 
  pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT); 
  pinMode(ENA,OUTPUT); pinMode(ENB,OUTPUT); 
  pinMode(IR_LEFT_RX, INPUT_PULLUP); 
  ladderServo.attach(LADDER_SERVO_PIN); ladderServo.write(0); 
  Serial.begin(115200); 
  radio.begin(); 
  radio.setPALevel(RF24_PA_LOW); 
  radio.setDataRate(RF24_250KBPS); 
  radio.openWritingPipe(ADDRESS); 
  radio.stopListening(); 
} 
void loop(){ 
  static uint8_t sideIdx = 0; 
  switch(state){ 

case SERP_STEP:{ 
Serial.println("Entered SERP_STEP"); sendLog("Entered SERP_STEP"); 
if(row >= GRID_ROWS){ sendLog("Grid limit reached"); while(true); } 
uint16_t dF = pingFront(); 
bool frontClose = dF && dF < DETECT_FRONT_CM; 
if(frontClose){ 
sendLog("Wall detected. Turning left."); 
leftPivot(); 
state = APPROACH_WALL; 
sendLog("Switch to APPROACH_WALL"); 
} else if(sweepRight){ 
if(col < GRID_COLS-1){ heading = EAST; advanceOneCell(); } 
else { rowChange(true); } 
} else { 
if(col > 0){ heading = WEST; advanceOneCell(); } 
else { rowChange(false); } 
} 
} break; 
case APPROACH_WALL:{ 
sendLog("Entered APPROACH_WALL"); 
uint16_t d = pingFront(); 
if(d > STOP_FRONT_CM){ 
sendLog(d > CRAWL_THRESHOLD_CM ? "Bursting forward" : "Crawling 
forward"); 
(d > CRAWL_THRESHOLD_CM ? fwdFast() : fwdCrawl()); 
delay(APPROACH_BURST_MS); stopMotors(); 
} else { 
stopMotors(); leftPivot(); sideIdx = 0; 
state = PERIM_SCAN; 
sendLog("Switch to PERIM_SCAN"); 
} 
} break; 
case PERIM_SCAN: 
sendLog("Entered PERIM_SCAN"); 
if(firePulse(150)){ 

lastFireWall = sideIdx; 
state = DEPLOY_LADDER; 
sendLog("🔥 Fire detected by IR sensor!"); 
break; 
} 
if(sideIdx >= 4){ 
lastFireWall = 255; 
postLadderMove(); 
break; 
state = SERP_STEP; 
sendLog("No fire. Resume sweep."); 
} 
if(driveAndScanRight(sideIdx==0 
? 
CELL_TIME_WALL_FIRST 
: 
CELL_TIME_WALL)){ 
lastFireWall = sideIdx; 
state = DEPLOY_LADDER; 
sendLog("🔥 Fire detected during wall scan!"); 
break; 
} 
rightPivot(); sideIdx++; 
break; 
case DEPLOY_LADDER: 
sendLog("Deploying ladder..."); 
stopMotors(); 
ladderServo.write(120); delay(800); ladderServo.write(0); 
sendLog("✅ Fire extinguished"); 
postLadderMove(); 
state = SERP_STEP; 
} 
sendLog("Returning to SERP_STEP"); 
break; 
}