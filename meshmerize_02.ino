#include <QTRSensors.h>
#include <SparkFun_TB6612.h>
#define Twhite 900
#define NUM_SENSORS 8  // Number of sensors used in the QTR array
int error_dir = 0;

// PID calculation variables
int last_error = 0;  // Last error value for PID calculation
int I = 0;           // PID terms
float Kp = 0.18, Ki = 0, Kd = 1.8;
float sp = 0.7, si = 0, sd =10;
// float Kp = 0.03, Ki = 0.0005, Kd = 0.3;         // PID constants
int center_position = 3500;                       // Default center position
uint8_t MAX_SPEED_R = 255, MAX_SPEED_L = 255;     // Maximum speed for motors
uint8_t BASE_SPEED_R = 250, BASE_SPEED_L = 250;  // Base speed for motors

// Create QTR object for IR
QTRSensors qtr;
uint16_t sensorValues[NUM_SENSORS];

// Motor driver pin definitions (TB6612FNG)
#define LIN1 17  
#define LIN2 16
#define LPWM 4  

#define RIN1 5
#define RIN2 18
#define RPWM 19
Motor motor_r = Motor(RIN1, RIN2, RPWM, -1, 99);  // Right motor
Motor motor_l = Motor(LIN1, LIN2, LPWM, -1, 99);  // Left motorl
#define LED_L 0
#define LED_R 21
// Motor speed control
// #define BASE_SPEED 150
// #define MAX_SPEED 200
#define TURN_SPEED 190
// setting up bittons for RSLB and LSRB
//#define ALGO 22
#define FINALB 23
#define STARTB 22
bool ABUT = false;
bool FBUT = false;
bool SBUT = false;
// for available path
bool LP = false;
bool RP = false;
bool FP = false;
char Turn;
char turn;
/**
 * @brief Sets the speed for both motors.
 * 
 * @param set_speed_r Speed to set for the right motor (0-255).
 * @param set_speed_l Speed to set for the left motor (0-255).
 * @references SparkFun_TB6612::drive
 */

/**
 * @brief Sets the speed for both motors.
 * 
 * @param set_speed_r Speed to set for the right motor (0-255).
 * @param set_speed_l Speed to set for the left motor (0-255).
 * @references SparkFun_TB6612::drive
 */
// Variables to store the path
char path[100];
int pathLength = 0;

void setup() {
  Serial.begin(9600);
  // Initialize QTR sensor array
  qtr.setTypeAnalog();                                                                  // Specify that we're using analog sensors
  qtr.setSensorPins((const uint8_t[]){ 13, 35, 14, 27, 26, 25, 33, 32 }, NUM_SENSORS);  // Set sensor pins

  pinMode(LED_L, OUTPUT);
  pinMode(LED_R, OUTPUT);
  // Setup Motor pins
  pinMode(RIN1, OUTPUT);  // Right motor direction pin 1
  pinMode(RIN2, OUTPUT);  // Right motor direction pin 2
  pinMode(RPWM, OUTPUT);  // Right motor PWM pin
  pinMode(LIN1, OUTPUT);  // Left motor direction pin 1
  pinMode(LIN2, OUTPUT);  // Left motor direction pin 2
  pinMode(LPWM, OUTPUT);  // Left motor PWM pin

  // Initialize motors but stop initially
  motor_r.brake();
  motor_l.brake();
  delay(2);
  // Setup Buttons
  pinMode(FINALB, INPUT);  // FINAL RUN
                           // pinMode(ALGO, INPUT);    // LSRB RSLB
  pinMode(STARTB, INPUT);  //Start/Stop
                           // Calibrate IR sensors
  while (true) {
    if (digitalRead(FINALB) == HIGH) {
      Serial.println("First butt..");
      delay(600);
      break;
    }
  }

  for (int i = 0; i < 100; i++) {
    if (i < 50) {
      setMotor(-100, 100);
      //set_motor_speed(-200, 200);  // Anticlockwise calibration
    } else {
      setMotor(100, -100);
      //set_motor_speed(200, -200);  // Clockwise calibration
    }
    Serial.print("calibrating sensors");
    qtr.calibrate();
    delay(20);
  }
  motor_r.brake();
  motor_l.brake();
  delay(2);

  while (true) {
    Serial.println("inside while");
    if (digitalRead(STARTB) == HIGH) {
      delay(600);
      ABUT = true;
      break;
    }
    if (digitalRead(FINALB) == HIGH) {
      delay(600);
      ABUT = false;
      break;
    }
  }
}



void loop() {
  
next:
  Serial.print("goto executed");
  while (1) {
    FP = false;
    RP = false;
    LP = false;
    pid_control();
    qtr.readLineBlack(sensorValues);
    if (sensorValues[0] == 1000) {
      LP = true;
      Turn = 'L';
      stopMotors();
      digitalWrite(LED_L, HIGH);
      for (int i = 0; i < 10; i++) {
        qtr.readLineBlack(sensorValues);
        if (sensorValues[7] == 1000) {
          RP = true;
          Turn = 'R';
          digitalWrite(LED_R, HIGH);
          break;
        }
      }
      break;
    } else {
      digitalWrite(LED_L, LOW);
    }


    qtr.readLineBlack(sensorValues);
    if (sensorValues[7] == 1000) {
      RP = true;
      Turn = 'R';
      stopMotors();
      digitalWrite(LED_R, HIGH);
      for (int i = 0; i < 10; i++) {
        qtr.readLineBlack(sensorValues);
        if (sensorValues[0] == 1000) {
          LP = true;
          Turn = 'L';
          digitalWrite(LED_L, HIGH);
          break;
        }
      }
      break;
    } else {
      digitalWrite(LED_R, LOW);
    }
    
    if (sensorValues[0] < Twhite && sensorValues[1] < Twhite && sensorValues[2] < Twhite && sensorValues[3] < Twhite && sensorValues[4] < Twhite && sensorValues[5] < Twhite && sensorValues[6] < Twhite && sensorValues[7] < Twhite) {
      uTurn();
      recordPath('U');
      optimizePath();
      stopMotors();
   for(int i=0;i<20;i++){
      Spid();
    }
    }
  }
  moveForward();
  stopMotors();
  //
  qtr.readLineBlack(sensorValues);
  if (sensorValues[0] == 1000) {
    LP = true;
    Turn = 'L';
    stopMotors();
    digitalWrite(LED_L, HIGH);
    for (int i = 0; i < 10; i++) {
      qtr.readLineBlack(sensorValues);
      if (sensorValues[7] == 1000) {
        RP = true;
        Turn = 'R';
        digitalWrite(LED_R, HIGH);
      }
    }
  } else {
    digitalWrite(LED_L, LOW);
  }

  
  qtr.readLineBlack(sensorValues);
  if (sensorValues[7] == 1000) {
    RP = true;
    Turn = 'R';
    stopMotors();
    digitalWrite(LED_R, HIGH);
    for (int i = 0; i < 10; i++) {
      qtr.readLineBlack(sensorValues);
      if (sensorValues[0] == 1000) {
        LP = true;
        Turn = 'L';
        digitalWrite(LED_L, HIGH);
      }
    }
  } else {
    digitalWrite(LED_R, LOW);
  }
  moveForward();
  stopMotors();


  if (sensorValues[0] == 1000 && sensorValues[1] == 1000 && sensorValues[2] == 1000 && sensorValues[3] == 1000 && sensorValues[4] == 1000 && sensorValues[5] == 1000 && sensorValues[6] == 1000 && sensorValues[7] == 1000) {
    while (true) {
      if (digitalRead(FINALB) == HIGH) {
        delay(600);
        break;
      }
    }
    finalRun();
  }
  if (sensorValues[3] == 1000 || sensorValues[4] == 1000 || sensorValues[2] == 1000 || sensorValues[5] == 1000) {
    FP = true;
  } else {
    FP = false;
  }
  if (ABUT) {
    if (LP) {
      turnLeft();
      recordPath('L');
      optimizePath();

    } else if (FP) {
      // moveForward();
      recordPath('S');
      optimizePath();
      goto next;
    } else if (RP) {
      turnRight();
      recordPath('R');
      optimizePath();
    }
    //delay(1000);
  } else {
    if (RP) {
      turnRight();
      recordPath('R');
      optimizePath();

    } else if (FP) {
      // moveForward();
      recordPath('S');
      optimizePath();
      goto next;
    } else if (LP) {
      turnLeft();
      recordPath('L');
      optimizePath();
    }
  }
  // delay(1000);
  digitalWrite(LED_L, LOW);
  digitalWrite(LED_R, LOW);
}

// Function to read the position of the line using QTR sensor array
int readLine() {
  int position = qtr.readLineBlack(sensorValues);  // Get the line position
  return position;
}

// Move forward
void moveForward() {
  setMotor(100, 100);
  delay(60);
}

// Turn left
void turnLeft() {
  while (sensorValues[3] == 1000 || sensorValues[4] == 1000) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    setMotor(100, -100);
  }
  while ( sensorValues[3] != 1000) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    setMotor(100, -100);
  }
  // while(sensorValues[7]==1000){
  //   setMotor(-50,50);
  // }
  
  stopMotors();
   for(int i=0;i<25;i++){
      Spid();
    }
  
}

// Turn right
void turnRight() {
  while (sensorValues[3] == 1000 || sensorValues[4] == 1000) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    setMotor(-100, 100);
  }
  while (sensorValues[4] != 1000) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    setMotor(-100, 100);
  }
  // while(sensorValues[0]==1000){
  //   setMotor(50,-50);
  // }
  stopMotors();
   for(int i=0;i<25;i++){
      Spid();
    }
}
void uTurn() {
  while (sensorValues[3] != 1000 || sensorValues[4] != 1000) {
    uint16_t position = qtr.readLineBlack(sensorValues);
    setMotor(-80, 80);
  }
  stopMotors();
  for(int i=0;i<20;i++){
      Spid();
    }
}
// Stop motors
void stopMotors() {
  motor_r.brake();
  motor_l.brake();
  delay(10);
}



// Record the movement in the path
void recordPath(char move) {
  if (pathLength < 100) {
    path[pathLength++] = move;
  }
}


void optimizePath() {

  if (pathLength < 3 || path[pathLength - 2] != 'U')  // simplify the path only if the second-to-last turn was a 'B'
    return;
  int total_angle = 0;
  int m;
  // Get the angle as a number between 0 and 360 degrees.
  for (m = 1; m <= 3; m++) {
    switch (path[pathLength - m]) {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'U':
        total_angle += 180;
        break;
    }
  }
  // Replace all of those turns with a single one.
  total_angle = total_angle % 360;
  switch (total_angle) {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = 'R';
      break;
    case 180:
      path[pathLength - 3] = 'U';
      break;
    case 270:
      path[pathLength - 3] = 'L';
      break;
  }
  pathLength -= 2;
}

void pid_control() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculate the error (difference from the center position)
  int error = center_position - position;
  int P = error;
  I += error;
  I = constrain(I, -255, 255);
  int D = error - last_error;
  last_error = error;

  // Calculate the motor speed adjustment based on PID values
  int motorSpeed = P * Kp + I * Ki + D * Kd;
  int rightMotorSpeed = constrain(BASE_SPEED_R + motorSpeed, 0, MAX_SPEED_R);
  int leftMotorSpeed = constrain(BASE_SPEED_L - motorSpeed, 0, MAX_SPEED_L);

  // Set motor speeds
  mspeed(rightMotorSpeed, leftMotorSpeed);
}
void Spid() {
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Calculate the error (difference from the center position)
  int error = center_position - position;
  int P = error;
  I += error;
  I = constrain(I, -255, 255);
  int D = error - last_error;
  last_error = error;

  // Calculate the motor speed adjustment based on PID values
  int motorSpeed = P * sp + I * si + D * sd;
  int rightMotorSpeed = constrain(BASE_SPEED_R + motorSpeed, 0, MAX_SPEED_R);
  int leftMotorSpeed = constrain(BASE_SPEED_L - motorSpeed, 0, MAX_SPEED_L);

  // Set motor speeds
  mspeed(rightMotorSpeed, leftMotorSpeed);
}
/**
 * @brief Sets the speed for both motors using the motor objects.
 * 
 * @param posa Speed to set for the right motor (0-255).
 * @param posb Speed to set for the left motor (0-255).
* @references SparkFun_TB6612::drive
                                                */
void mspeed(int posa, int posb) {
  motor_r.drive(posa);
  motor_l.drive(posb);
}
void setMotor(int set_speed_r, int set_speed_l) {
  motor_r.drive(set_speed_r);
  motor_l.drive(set_speed_l);
}
void finalRun() {
  int i = 0;
  uint16_t position = qtr.readLineBlack(sensorValues);
  while (pathLength) {
    while (1) {

      // for (int i = 0; i < 10; i++) {
      //   if (digitalRead(FINALB) == HIGH) {
      //     stopMotors();
      //     digitalWrite(LED_R, HIGH);
      //     digitalWrite(LED_L, HIGH);
      //     delay(6000);
      //     finalRun();
      //     return;
      //   }
      // }
      pid_control();
      qtr.readLineBlack(sensorValues);
      if (sensorValues[0] == 1000 || sensorValues[7] == 1000) {
        break;
      }
    }
    stopMotors();
    //delay(100);
    setMotor(100, 100);
    delay(100);
    stopMotors();
    //delay(100);
    delay(75);
    if (path[i] == 'L') {
      turnLeft();
    } else if (path[i] == 'S') {
      setMotor(80, 80);
      delay(50);
      //moveForward();
      stopMotors();
    } else if (path[i] == 'R') {
      turnRight();
    }

    i++;
    if (i > pathLength) {
      break;
    }
  }
  stopMotors();
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_L, HIGH);
  while (1) {
    if (digitalRead(FINALB) == HIGH) {
      delay(3000);
      finalRun();
      return;
    }
  }
}
