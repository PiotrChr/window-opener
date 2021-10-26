#include <CircularBuffer.h>
#include <Wire.h>
#include <AccelStepper.h>

const int stepperEnPin = 2;
const int stepPin = 6;
const int dirPin = 5;
int stepperState = LOW;
int pos = 0;

const int stepDelayMs = 500;
const int fullCyclePulses = 200;

float motorMaxSpeed = 1000.0;
float motorSpeed = 100.0;   //pulse per second
float motorAccel = 5000.0; //87500; //steps/second/second to accelerate
long stepperMaxPos = 0;
long stepperCurrentPos = 0;
AccelStepper stepper(1, stepPin, dirPin); 

const int bottomLimitPin = 8;
const int topLimitPin = 7;
const int openButton = 3;
const int closeButton = 4;
int openButtonState;
int closeButtonState;
int topLimitState;
int bottomLimitState;
int lastTopLimitState = LOW;
int lastBottomLimitState = LOW;

const float potentiometerMax = 1018;
const float potentiometerOffset = 2;
const float potentiometerStep = 100/potentiometerMax;
int potentiometerAnalogPin = 1;
int potentiometerValue = 0;
bool potentiometerLock = false;

bool runHoming = false;
bool topHoming = false;
bool bottomHoming = false;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 150;
unsigned long lastButtonLockTime = millis();
const unsigned long buttonLockTime = 3000;

bool locked = false;

typedef void (*Job)();
typedef bool (*RunCondition)();

struct Task {
  String name;
  unsigned long dueDate;
  bool finished;
  Job job;
  RunCondition runCondition;
};

const int taskLimit = 20;
CircularBuffer <Task, taskLimit> tasks;

void setup() {
  Serial.begin(9600);
  setupButtons();
  setupStepper();
//  setupI2c();
  potentiometerValue = analogRead(potentiometerAnalogPin);
  delay(1000);

  unsigned long now = millis();
  addTask("Start Homing", (now), []() { startHoming(); }, []() { return true; });
  addTask("Open To 50", (now+1), []() { openTo(50); }, []() { return isIdle(); });
}

void setupButtons() {
  pinMode(openButton, INPUT);
  pinMode(closeButton, INPUT);
  pinMode(topLimitPin, INPUT);
  pinMode(bottomLimitPin, INPUT);
  }

void setupStepper() {
   pinMode(stepperEnPin, OUTPUT);
    
   stepper.setMaxSpeed(motorMaxSpeed);
   stepper.setSpeed(motorSpeed);
   stepper.setAcceleration(motorAccel);
  }

void stepperOn() {
  stepperState = HIGH;
  }

void stepperOff() {
  stepperState = LOW;
  }

void addTask(String name, unsigned long dueDate, void job (), bool completionCondition ()) {
  struct Task task = {name, dueDate, false, job, completionCondition};
  tasks.push(task);
}

void stepperDir(String dir = "right") {
  if (dir == "right") {
    digitalWrite(dirPin,HIGH);
    } else {
      digitalWrite(dirPin,LOW);
      }
  }

void startHoming() {
  bottomHoming = true;
  runHoming = true;
  }

void setHoming(String side) {
  long newPosition = 3000L;
  int pin = topLimitPin;
  
  if (side == "bottom") {
    newPosition = -3000;
    pin = bottomLimitPin;
  }

  stepper.moveTo(newPosition);
  }

void openTo(int openPercentage) {
  stepper.moveTo(stepperLocationFromOpening(openPercentage));
  }

long stepperLocationFromOpening(int openPercentage) {
  return (long) openPercentage * stepperMaxPos / 100;
  }

void fullOpen() {
  openTo(100);
  }

void fullClose() {
  openTo(0);
  }

void performTasks() {
  for (int i = 0; i < tasks.size(); i++) {
    if (!tasks[i].finished && tasks[i].dueDate < millis() && tasks[i].runCondition() == true) {
      struct Task task = tasks.shift();
      task.job();
      task.finished = true;
    } else {
      return;
    }
  }
}

void checkButtons() {
  int openReading = digitalRead(openButton);
  int closeReading = digitalRead(closeButton);

  unsigned long now = millis();

  if ((now - lastButtonLockTime) < buttonLockTime) {
    return;
  }

  //  if (reading != lastButtonState) {
  //    lastDebounceTime = now;
  //    }

  if ((now - lastDebounceTime) > debounceDelay) {
    if (openReading != openButtonState) {
      openButtonState = openReading;
      if (openButtonState == HIGH) {
        addTask("Open", (now), []() { fullOpen(); }, []() { return isIdle(); });
      }
    }

    if (closeReading != closeButtonState) {
      closeButtonState = closeReading;
      if (closeButtonState == HIGH) {
        addTask("Open", (now), []() { fullClose(); }, []() { return isIdle(); });
      }

    }
    lastButtonLockTime = now;
  }
}

bool isIdle() {
  return stepper.distanceToGo() == 0 && !topHoming && !bottomHoming;
  }

void checkLimits() {
  if ((stepper.currentPosition() == stepperMaxPos && digitalRead(topLimitPin) && stepper.targetPosition() > stepper.currentPosition()) || (stepper.currentPosition() == 0 && digitalRead(bottomLimitPin) && stepper.targetPosition() < stepper.currentPosition())) {
    stepper.setCurrentPosition(stepper.currentPosition());
    stepper.stop();
    delay(200);
    stepperOff();
    }
}

void checkStepper() {
  digitalWrite(stepperEnPin, stepperState == LOW ? HIGH : LOW);
  }

void runStepper() {
  if (stepper.distanceToGo() != 0) {
  if (stepperState == LOW) {
      stepperOn();
      delay(1000);
    }
  
  if (topHoming) {
    if (digitalRead(topLimitPin) == HIGH) {
      topHoming = false;
      stepperMaxPos = stepper.currentPosition();
      stepper.setCurrentPosition(stepperMaxPos);
      stepper.stop();
      stepper.runToPosition();
      
      delay(500);
      return;
      }
    }

  if (bottomHoming) {
    if (digitalRead(bottomLimitPin) == HIGH) {
      stepper.setCurrentPosition(0);
      stepper.stop();
      stepper.runToPosition();
      bottomHoming = false;
      runHoming = true;
      topHoming = true;

      delay(500);
      return;
      }    
    }
    stepper.run();
    stepperCurrentPos = stepper.currentPosition();
  } else {
    stepperOff();
    Serial.println("Turning Off");
    }
}

int potToPercentage(int potValue) {
  Serial.println(potentiometerStep);
  return (int) (potentiometerStep * potValue);
  }

void checkHoming() {
  if (topHoming && runHoming) {
    setHoming("top");
    runHoming = false;
    }

  if (bottomHoming && runHoming) {
    setHoming("bottom");
    runHoming = false;
    }
  }

void checkPotentiometer() {
  int newRead = analogRead(potentiometerAnalogPin);

  if (newRead < (potentiometerValue-potentiometerOffset) || newRead > (potentiometerValue+potentiometerOffset)) {
    potentiometerValue = newRead;
    addTask("Open [potentiometer]", (millis()), []() { openTo(potToPercentage(potentiometerValue)); }, []() { return isIdle(); });
    }
  }
    
void loop() {
  checkButtons();
  checkLimits();
////  checkLeds();
  checkStepper();
  checkHoming();
  checkPotentiometer();
  performTasks();
  runStepper();
}
