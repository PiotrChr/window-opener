#include "ESPAsyncWebServer.h"
#include "CircularBuffer.h"
#include "ESP_FlexyStepper.h"

#include "EEPROM.h"
#include "Regexp.h"
#include "Wire.h"
#include "WiFi.h"
#include "SPI.h"
#include "FS.h"
#include "SD.h"
#include "Wire.h"
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C /// < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define LOGO_HEIGHT   64
#define LOGO_WIDTH    128
// 'logov1', 128x64px
const unsigned char logov1 [] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x03, 0x06, 0x0c, 0xc0, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x03, 0x06, 0x0c, 0xc0, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x01, 0x87, 0x18, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x01, 0x8f, 0x18, 0xcd, 0xe0, 0x7f, 0x1f, 0x0c, 0x63, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x01, 0x8b, 0x18, 0xcf, 0xf0, 0xff, 0x3f, 0x8c, 0x63, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x01, 0x89, 0x18, 0xce, 0x38, 0xc3, 0x71, 0xc4, 0x62, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0xc9, 0xb0, 0xcc, 0x19, 0x83, 0x60, 0xc6, 0xf6, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0xd9, 0xb0, 0xcc, 0x19, 0x83, 0x60, 0xc6, 0x96, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0xd9, 0xb0, 0xcc, 0x19, 0x83, 0x60, 0xc6, 0x96, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0xd0, 0xb0, 0xcc, 0x19, 0x83, 0x60, 0xc2, 0x9c, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x70, 0xe0, 0xcc, 0x19, 0xc3, 0x71, 0xc3, 0x0c, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x70, 0xe0, 0xcc, 0x18, 0xff, 0x3f, 0x83, 0x0c, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x70, 0xe0, 0xcc, 0x18, 0x7b, 0x1f, 0x03, 0x0c, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x03, 0xc0, 0x04, 
  0x20, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x07, 0xe0, 0x04, 
  0x20, 0x07, 0x1c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x06, 0x60, 0x04, 
  0x20, 0x0e, 0x0e, 0xde, 0x07, 0x86, 0xf0, 0x3c, 0x36, 0x00, 0xc3, 0x0f, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x0c, 0x06, 0xff, 0x0f, 0xc7, 0xf8, 0x7e, 0x3e, 0x00, 0xe7, 0x03, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x0c, 0x06, 0xe3, 0x98, 0x67, 0x1c, 0xc3, 0x38, 0x00, 0x66, 0x03, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x0c, 0x06, 0xc1, 0x9f, 0xe6, 0x0c, 0xff, 0x30, 0x00, 0x66, 0x03, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x0c, 0x06, 0xc1, 0x9f, 0xe6, 0x0c, 0xff, 0x30, 0x00, 0x66, 0x03, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x0c, 0x06, 0xc1, 0x98, 0x06, 0x0c, 0xc0, 0x30, 0x00, 0x3c, 0x03, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x0e, 0x0e, 0xc1, 0x98, 0x06, 0x0c, 0xc0, 0x30, 0x00, 0x3c, 0x03, 0x00, 0x0c, 0x30, 0x04, 
  0x20, 0x07, 0x1c, 0xc3, 0x1c, 0x26, 0x0c, 0xe1, 0x30, 0x00, 0x3c, 0x03, 0x01, 0x86, 0x60, 0x04, 
  0x20, 0x03, 0xf8, 0xff, 0x0f, 0xe6, 0x0c, 0x7f, 0x30, 0x00, 0x18, 0x0f, 0xc1, 0x87, 0xe0, 0x04, 
  0x20, 0x01, 0xf0, 0xfe, 0x07, 0xc6, 0x0c, 0x3e, 0x30, 0x00, 0x18, 0x0f, 0xc1, 0x83, 0xc0, 0x04, 
  0x20, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0xe4, 0x02, 0x00, 0x0f, 0x40, 0x00, 0x00, 0x04, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x10, 0x02, 0x00, 0x10, 0x40, 0x00, 0x00, 0x00, 0x04, 
  0x20, 0x00, 0x00, 0x00, 0x01, 0xe4, 0x41, 0x14, 0x73, 0x28, 0x20, 0x78, 0xa8, 0x9c, 0xe4, 0xc4, 
  0x20, 0x00, 0x00, 0x00, 0x01, 0x12, 0x81, 0x14, 0x8a, 0x30, 0x20, 0x44, 0xc8, 0x91, 0x05, 0x24, 
  0x3f, 0xff, 0xff, 0xff, 0xf1, 0x12, 0x81, 0xe4, 0x8a, 0x20, 0x20, 0x44, 0x88, 0x89, 0x05, 0xe4, 
  0x00, 0x00, 0x00, 0x00, 0x01, 0x12, 0x81, 0x04, 0x8a, 0x20, 0x10, 0x44, 0x88, 0x85, 0x05, 0x04, 
  0x00, 0x00, 0x00, 0x00, 0x01, 0xe1, 0x01, 0x04, 0x71, 0x20, 0x0f, 0x44, 0x87, 0x9c, 0xe4, 0xe4, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

AsyncWebServer server(80);

// Wifi config
const int SDCardReaderPin = 5;
const char* wifiConfigFile = "/wifi.txt";
const char* wifiConfigFileReplacement = "/wifi_old.txt";

String fileContents;
IPAddress localIp;

char wifiSsid [100];
char wifiPassword [100];
const int ssidEEPROMIndex = 0;
const int passwordEEPROMIndex = 100;

unsigned const long wifiConnectionTimeout = 10000;
unsigned const long wifiCheckInterval = 100000;
unsigned long wifiCheckLock = 0;

// Stepper config
const int stepperEnPin = 13;
const int stepPin = 12;
const int dirPin = 14;

int stepperState = LOW;
int pos = 0;

const int stepDelayMs = 500;
const int fullCyclePulses = 200;
float motorMaxSpeed = 1000.0;
float motorSpeed = 1000.0;
float motorAccel = 5000.0;

long stepperMaxPos = 0;
long stepperCurrentPos = 0;

ESP_FlexyStepper stepper;

// Limit Buttons config
const int bottomLimitPin = 26;
const int topLimitPin = 27;

int topLimitState;
int bottomLimitState;

// Control Buttons config
const int openButton = 25;
const int closeButton = 33;
int openButtonState;
int closeButtonState;

const int stepDownButton = 16;
const int stepUpButton = 17;
int stepUpButtonState;
int stepDownButtonState;
const int stepValue = 10;

const int actionButton = 4;

int lastTopLimitState = LOW;
int lastBottomLimitState = LOW;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 150;
unsigned long lastButtonLockTime = millis();
const unsigned long buttonLockTime = 500;

// Display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
unsigned long lastDisplayUpdate = millis();
int displayUpdateTimeout = 15000;

// Homing config
bool runHoming = false;
bool topHoming = false;
bool bottomHoming = false;

// Open states config
int openValue = 0;
int currentOpenValue = 0;
bool openReceived = false;

bool locked = false;

// Task runner config
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

// Setup
void setup() {
  Serial.begin(9600);
  stepperOff();
  setupDisplay();
  
  displayStatus("Restoring from EEPROM...");
  restoreSettingsFromEEPROM();
  delay(1000);
  
  setupButtons();
  
  displayStatus("Setting up Motor...");
  setupStepper();
  delay(1000);
  
  displayStatus("Reading SD...");
  if (getWifiConfigFromSD()) {
    saveSettingsToEEPROM();
  }
  delay(1000);

  displayStatus("Setting up Wifi...");
  setupWifi();
  delay(1000);

   
  if (wifiSsid[0] != '\0') {
    char wifiText[200] = "Connecting to: ";
    strcat (wifiText, wifiSsid);
    displayStatus(wifiText);
    wifiConnect();
    delay(1000);
  }

  displayStatus("Setting up HTTP server...");
  setupServer();
  delay(1000);

  displayStatus("Starting");
  delay(1000);
  displayStatus("OK");

  unsigned long now = millis();
  addTask("Start Homing", (now), []() { startHoming(); }, []() { return true; });
  addTask("Open To 50", (now+1), []() { openTo(50); }, []() { return isIdle(); });
}

void setupDisplay() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { 
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.drawBitmap(0, 0,  logov1, LOGO_WIDTH, LOGO_HEIGHT, WHITE);

  display.display();

  delay(5000); // Pause for 2 seconds
  
  }

void displayStatus(char* action) {
  display.clearDisplay();

  display.setTextSize(1);             
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);            
  display.println(localIp);
  display.setCursor(0,15);         
  display.print("Opened to: ");
  display.print(currentOpenValue);
  display.println("%");
  if (action != "") {
    display.setCursor(0,30);         
    display.println(action);
    }
  
  display.display();

  lastDisplayUpdate = millis();
  }  

void setupButtons() {
  pinMode(openButton, INPUT);
  pinMode(closeButton, INPUT);
  pinMode(topLimitPin, INPUT);
  pinMode(bottomLimitPin, INPUT);
  }

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  }

void setupServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", "Hello");
  });
  server.on("/full_open", HTTP_GET, [](AsyncWebServerRequest *request){
    addTask("API Open", millis(), []() { fullOpen(); }, []() { return isIdle(); });
    request->send_P(200, "text/plain", "Opening");
  });
  server.on("/full_close", HTTP_GET, [](AsyncWebServerRequest *request){
    addTask("API Close", millis(), []() { fullClose(); }, []() { return isIdle(); });
    request->send_P(200, "text/plain", "Closing");
  });
  server.on("open", HTTP_GET, [] (AsyncWebServerRequest *request) {
      AsyncWebParameter* param = request->getParam(0);
      openValue = param->value().toInt();

      if (openValue >= 0 && openValue <= 100) {
        addTask("API Open To selected value", millis(), []() { openTo(openValue); }, []() { return isIdle(); });
        request->send_P(200, "text/plain", "Opening to selected value");
        }
      
      request->send_P(400, "text/plain", "Wrong value");
  });

  server.begin();
  }

void setupStepper() {
  pinMode(stepperEnPin, OUTPUT);
  stepper.connectToPins(stepPin, dirPin);
  delay(500);
  stepper.setSpeedInStepsPerSecond(motorSpeed);
  stepper.setAccelerationInStepsPerSecondPerSecond(motorAccel);
  }

// Program
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

void wifiConnect() {
  Serial.println("Connecting to: ");
  Serial.println(wifiSsid);
  Serial.println(wifiPassword);
  WiFi.begin(wifiSsid, wifiPassword);
  unsigned long currentMillis = millis();
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
    if (currentMillis > wifiConnectionTimeout) {
      return;
      }
  }


  localIp = WiFi.localIP();
  Serial.println(localIp);
  }

void checkWifi() {
  unsigned long currentMillis = millis();
  
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - wifiCheckLock >= wifiCheckInterval)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    wifiCheckLock = currentMillis;
    localIp = WiFi.localIP();
    Serial.println(localIp);
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

  stepper.setTargetPositionInSteps(newPosition);
  }

void openTo(int openPercentage) {
  stepper.setTargetPositionInSteps(stepperLocationFromOpening(openPercentage));
  
  currentOpenValue = openPercentage;
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

void stepUp() {
  int newOpenValue;
  const int rest = currentOpenValue % stepValue;

  if (rest != 0) {
      newOpenValue = currentOpenValue + (stepValue - rest);
    } else {
      newOpenValue = currentOpenValue + 10;
    }
  
  if (newOpenValue > 100) {
    newOpenValue = 100;
  }

  openTo(newOpenValue);
}

void stepDown() {
  int newOpenValue;
  const int rest = currentOpenValue % stepValue;

  if (rest != 0) {
      newOpenValue = currentOpenValue - rest;
    } else {
      newOpenValue = currentOpenValue - 10;
    }
  
  if (newOpenValue < 0) {
    newOpenValue = 0;
  }

  openTo(newOpenValue);
}

void performTasks() {
  for (int i = 0; i < tasks.size(); i++) {
    if (!tasks[i].finished && tasks[i].dueDate < millis() && tasks[i].runCondition() == true) {
      struct Task task = tasks.shift();
      char taskName[100];
      task.name.toCharArray(taskName, sizeof(taskName));
      displayStatus(taskName);
      Serial.println(task.name);
      task.job();
      task.finished = true;
    } else {
      return;
    }
  }
}

void checkButtons() {
  checkButton(openButton, [](unsigned long initTime) {
    addTask("Open", initTime, []() { fullOpen(); }, []() { return isIdle(); });
  }, &lastButtonLockTime, &openButtonState);

  checkButton(closeButton, [](unsigned long initTime) {
    addTask("Close", initTime, []() { fullClose(); }, []() { return isIdle(); });
  }, &lastButtonLockTime, &closeButtonState);

  checkButton(closeButton, [](unsigned long initTime) {
    addTask("StepUp", initTime, []() { stepUp(); }, []() { return isIdle(); });
  }, &lastButtonLockTime, &stepUpButtonState);

  checkButton(closeButton, [](unsigned long initTime) {
    addTask("StepDown", initTime, []() { stepDown(); }, []() { return isIdle(); });
  }, &lastButtonLockTime, &stepDownButtonState);
}

void checkButton(int buttonPin, void handler (unsigned long initTime), unsigned long* lastLock, int* buttonState) {
  int reading = digitalRead(buttonPin);
  
  unsigned long now = millis();

  if ((now - *lastLock) < buttonLockTime) {
    return;
  }

  //  if (reading != lastButtonState) {
  //    lastDebounceTime = now;
  //    }

  if ((now - lastDebounceTime) > debounceDelay) {
    if (reading != *buttonState) {
      *buttonState = reading;
      if (*buttonState == HIGH) {
        handler(now);
      }
    }

    *lastLock = now;
  }
}

bool isIdle() {
  return stepper.getDistanceToTargetSigned() == 0 && !topHoming && !bottomHoming;
  }

void checkLimits() {
  long currentPos = stepper.getCurrentPositionInSteps();
  long targetPos = stepper.getTargetPositionInSteps();
  
  if ((currentPos == stepperMaxPos && digitalRead(topLimitPin) && targetPos > currentPos) || (currentPos == 0 && digitalRead(bottomLimitPin) && targetPos < currentPos)) {
    Serial.println("LimitHit");
    stepper.setCurrentPositionInSteps(currentPos);
    stepper.setTargetPositionInSteps(currentPos);
    delay(200);
    stepperOff();
    }
}

void checkStepper() {
  digitalWrite(stepperEnPin, stepperState == LOW ? HIGH : LOW);
  }

void runStepper() {
  if (stepper.getDistanceToTargetSigned() != 0) {
  if (stepperState == LOW) {
      stepperOn();
      delay(500);
    }
  
  if (topHoming) {
    if (digitalRead(topLimitPin) == HIGH) {
      topHoming = false;
      stepperMaxPos = stepper.getCurrentPositionInSteps();
      stepper.hardStop();
     
      return;
      }
    }

  if (bottomHoming) {
    if (digitalRead(bottomLimitPin) == HIGH) {
      Serial.println("Stop bottom!");
      stepper.setCurrentPositionAsHomeAndStop();
      bottomHoming = false;
      runHoming = true;
      topHoming = true;
      
      return;
      }    
    }
    stepper.processMovement();
    stepperCurrentPos = stepper.getCurrentPositionInSteps();
  } else {
    stepperOff();
    Serial.println("Turning Off");
    }
}

//int potToPercentage(int potValue) {
//  int value = (potValue - potentiometerMin);
//  
//  return value > 0 ? value * potentiometerStep : 0;
//  }

// File System and SD card reader

void restoreSettingsFromEEPROM() {
  readStringFromEEPROM(ssidEEPROMIndex, &wifiSsid);
  readStringFromEEPROM(passwordEEPROMIndex, &wifiPassword);
  }

void saveSettingsToEEPROM() {
  writeStringToEEPROM(ssidEEPROMIndex, wifiSsid);
  writeStringToEEPROM(passwordEEPROMIndex, wifiPassword);
}

void writeStringToEEPROM(int addrOffset, const String &strToWrite) {
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);
  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}

void readStringFromEEPROM(int addrOffset, char (*buf)[100]) {
  int newStrLen = EEPROM.read(addrOffset);
  
  for (int i = 0; i < newStrLen; i++)
  {
    *buf[i] = EEPROM.read(addrOffset + 1 + i);
  }
  *buf[newStrLen] = '\0';
}

bool getWifiConfigFromSD() {
  if(!SD.begin(SDCardReaderPin)){
    Serial.println("Card Mount Failed");
    return false;
  }

  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return false;
  }

  readFile(SD, wifiConfigFile, &fileContents);
  
  if (fileContents[0] == '\0') {
    return false;
    }

  return scanStringAndSetConfig(fileContents);
  }

bool scanStringAndSetConfig(String content) {
  MatchState ms;
  char ssidRegex [100] = "ssid=([A-Za-z0-9_@.\\/#&+-]*)";
  char passwordRegex [100] = "password=([A-Za-z0-9_@.\\/#&+-]*)";
  char buf[1024];

  content.toCharArray(buf, sizeof(buf));
  ms.Target(buf);

  if (ms.Match(ssidRegex) == REGEXP_NOMATCH) {
    Serial.println("No match for ssid");
    Serial.println(content);
    return false;
    }
  
  ms.GetCapture (wifiSsid, 0);

  if (ms.Match(passwordRegex) == REGEXP_NOMATCH) {
    Serial.println("No match for password");
    return false;
    }

  ms.GetCapture (wifiPassword, 0);    

  return true;
  }

void readFile(fs::FS &fs, const char * path, String* contents){
  Serial.printf("Reading file: %s\n", path);
  
  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  memset(contents, '\0', sizeof(contents));
  
  Serial.print("Read from file: ");
  while(file.available()) {
    char currentChar = file.read();
    *contents += currentChar;
  }
  file.close();
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
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

//void checkPotentiometer() {
//  int newRead = analogRead(potentiometerAnalogPin);
//  
//  if (newRead < (potentiometerValue+potentiometerOffset) || newRead > (potentiometerValue+potentiometerOffset)) {
//    potentiometerValue = newRead;
//    addTask("Open [potentiometer]", (millis()), []() { openTo(potToPercentage(potentiometerValue)); }, []() { return isIdle(); });
//    }
//  }
    
void loop() {
  checkButtons();
  checkLimits();
//  checkLeds();
  checkStepper();
  checkHoming();
//  checkWifi();
  performTasks();
  runStepper();
}
