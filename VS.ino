#include <Wire.h>
#include <MS5837.h>
#include <WiFi.h>
#include <WebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <ArduinoOTA.h>
#include <Update.h>

// --- I2C PINS ---
#define CUSTOM_SDA_PIN 21
#define CUSTOM_SCL_PIN 22

MS5837 sensor;

// --- WIFI CREDENTIALS ---
const char* ssid     = "SSCFloat";
const char* password = "DT1234dt";
WebServer server(80);

// --- SENSOR VARIABLES ---
float pressures[120];
int sensorIdx = 0;
int iterationCount = 0;
float currentRelativeDepth = 0.0; 
float surfaceDepthOffset = 0.0;   
bool sensorOnline = false; // TRACKS SENSOR HEALTH

// --- MOTOR PINS & SETTINGS ---
const int dirPin  = 4;
const int stepPin = 16;
int motorSpeed = 2000;

// --- LIMIT SWITCHES ---
#define BUTTON_PIN_1 19    // Upper Limit (Sinking stop)
#define BUTTON_PIN_2 18    // Lower Limit (Floating stop)

// --- STATE MACHINE ---
enum SystemMode {
  MODE_IDLE,
  MODE_SEQUENCE,
  MODE_HOLD_DEPTH
};
volatile SystemMode currentMode = MODE_IDLE; 

// --- CONFIGURATION VARIABLES ---
uint32_t floatWaitTimeMs = 45000;      
uint32_t sensorReadIntervalMs = 100;   
float targetDepthCm = 40.0;            

// --- MUTEXES & TASK HANDLES ---
SemaphoreHandle_t dataLock = NULL;
SemaphoreHandle_t modeLock = NULL;
SemaphoreHandle_t configLock = NULL;

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t webTaskHandle = NULL;
TaskHandle_t motorTaskHandle = NULL;

// --- SENSOR TASK ---
void sensorTask(void *parameter) {
  while(1) {
    if (sensorOnline) {
      sensor.read();
      
      if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
        pressures[sensorIdx] = sensor.pressure();
        currentRelativeDepth = sensor.depth() - surfaceDepthOffset; 
        sensorIdx = (sensorIdx + 1) % 120;
        iterationCount++;
        xSemaphoreGive(dataLock);
      }
    }
    
    uint32_t delayMs = 100; 
    if (xSemaphoreTake(configLock, portMAX_DELAY)) {
      delayMs = sensorReadIntervalMs;
      xSemaphoreGive(configLock);
    }
    
    vTaskDelay(pdMS_TO_TICKS(delayMs)); 
  }
}

// --- WEB SERVER TASK ---
void webTask(void *parameter) {
  while(1) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// --- WEB ENDPOINTS ---

void handleData() {
  if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
    int count = iterationCount < 120 ? iterationCount : 120;
    String data = "Current Depth: " + String(currentRelativeDepth * 100.0) + " cm\n";
    data += "Sensor Status: " + String(sensorOnline ? "ONLINE" : "OFFLINE") + "\n\n";
    
    for (int i = 0; i < count; i++) {
      int idx = (sensorIdx - 1 - i + 120) % 120;
      data += String(pressures[idx]) + "\n";
    }
    xSemaphoreGive(dataLock);
    server.send(200, "text/plain", data);
  }
}

void handleControl() {
  // Handle actions first
  if(server.hasArg("action")) {
    String action = server.arg("action");
    if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
      if (action == "start") currentMode = MODE_SEQUENCE;
      else if (action == "hold") currentMode = MODE_HOLD_DEPTH;
      else if (action == "stop") currentMode = MODE_IDLE;
      xSemaphoreGive(modeLock);
    }
    server.sendHeader("Location", "/control");
    server.send(303);
    return;
  } 

  // Read hardware state for UI display
  bool topLimitHit = (digitalRead(BUTTON_PIN_1) == LOW);
  bool botLimitHit = (digitalRead(BUTTON_PIN_2) == LOW);

  SystemMode modeCopy;
  if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
    modeCopy = currentMode;
    xSemaphoreGive(modeLock);
  }

  float currentTarget = 40.0;
  if (xSemaphoreTake(configLock, portMAX_DELAY)) {
    currentTarget = targetDepthCm;
    xSemaphoreGive(configLock);
  }
  
  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:sans-serif; text-align:center;} button { padding: 15px; margin: 10px; font-size: 18px; width: 90%; max-width: 400px; display: inline-block; border-radius:8px; border:none; cursor:pointer;} ";
  html += ".stop { background-color: #ff4c4c; color: white; font-weight: bold; } ";
  html += ".config { background-color: #4CAF50; color: white; }";
  html += ".status-box { padding: 12px; margin: 10px auto; border-radius: 8px; width: 85%; max-width: 380px; font-weight: bold; border: 1px solid #ccc;}";
  html += ".ok { background-color: #d4edda; color: #155724; }";
  html += ".err { background-color: #f8d7da; color: #721c24; }";
  html += ".active { background-color: #fff3cd; color: #856404; }";
  html += "</style></head><body>";

  html += "<h2>ROV Float Controller</h2>";

  // --- HARDWARE STATUS SECTION ---
  if (sensorOnline) html += "<div class='status-box ok'>PRESSURE SENSOR: CONNECTED</div>";
  else html += "<div class='status-box err'>PRESSURE SENSOR: DISCONNECTED</div>";

  html += "<div style='display: flex; justify-content: center; gap: 10px;'>";
  html += "<div class='status-box " + String(topLimitHit ? "active" : "ok") + "' style='width: 40%;'>TOP: " + String(topLimitHit ? "HIT" : "OPEN") + "</div>";
  html += "<div class='status-box " + String(botLimitHit ? "active" : "ok") + "' style='width: 40%;'>BOT: " + String(botLimitHit ? "HIT" : "OPEN") + "</div>";
  html += "</div><hr style='width:90%'>";

  // --- MODE STATUS ---
  if (modeCopy == MODE_IDLE) html += "<p>Status: <strong>IDLE</strong></p>";
  else if (modeCopy == MODE_SEQUENCE) html += "<p>Status: <strong>RUNNING SEQUENCE</strong></p>";
  else if (modeCopy == MODE_HOLD_DEPTH) html += "<p>Status: <strong>HOLDING " + String(currentTarget) + " CM</strong></p>";

  html += "<button onclick=\"location.href='/control?action=start'\" style='background:#007bff; color:white;'>Run Full Sequence</button>";
  html += "<button onclick=\"location.href='/control?action=hold'\" style='background:#17a2b8; color:white;'>Hold Depth (" + String(currentTarget) + " cm)</button>";
  html += "<button class='stop' onclick=\"location.href='/control?action=stop'\">EMERGENCY STOP</button>";
  html += "<hr style='width:90%'><button class='config' onclick=\"location.href='/config'\">Settings & Configuration</button>";
  
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleConfig() {
  if (server.method() == HTTP_POST || server.args() > 0) {
    if (xSemaphoreTake(configLock, portMAX_DELAY)) {
      if (server.hasArg("waitTime") && server.arg("waitTime") != "") floatWaitTimeMs = server.arg("waitTime").toInt();
      if (server.hasArg("sensorInterval") && server.arg("sensorInterval") != "") sensorReadIntervalMs = server.arg("sensorInterval").toInt();
      if (server.hasArg("targetDepth") && server.arg("targetDepth") != "") targetDepthCm = server.arg("targetDepth").toFloat();
      xSemaphoreGive(configLock);
    }
    server.sendHeader("Location", "/config");
    server.send(303);
    return;
  }

  uint32_t currWait = 0, currInterval = 0;
  float currDepth = 40.0;
  if (xSemaphoreTake(configLock, portMAX_DELAY)) {
    currWait = floatWaitTimeMs; currInterval = sensorReadIntervalMs; currDepth = targetDepthCm;
    xSemaphoreGive(configLock);
  }

  String html = "<html><head><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>body{font-family:Arial; text-align:center;} input, button{padding:12px; margin:8px 0; font-size:16px; width:90%; max-width:400px; border-radius:5px; border:1px solid #ccc;}</style></head><body>";
  html += "<h2>Configuration</h2><form action='/config' method='POST'>";
  html += "<label>Wait at Bottom (ms): [" + String(currWait) + "]</label><br><input type='number' name='waitTime'><br>";
  html += "<label>Sensor Rate (ms): [" + String(currInterval) + "]</label><br><input type='number' name='sensorInterval'><br>";
  html += "<label>Target Depth (cm): [" + String(currDepth) + "]</label><br><input type='number' step='0.1' name='targetDepth'><br>";
  html += "<button type='submit' style='background:#28a745; color:white;'>Save Changes</button></form>";
  html += "<button onclick=\"location.href='/control'\">Back</button></body></html>";
  server.send(200, "text/html", html);
}

// --- HELPER: STEP MOTOR CHUNK ---
void stepMotorChunk(int steps, SystemMode expectedMode) {
  for (int i = 0; i < steps; i++) {
    if (currentMode != expectedMode) return; 
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(motorSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(motorSpeed);
    if (i % 20 == 0) taskYIELD(); 
  }
}

// --- MOTOR SEQUENCE ---
void runStepperSequence() {
  digitalWrite(dirPin, LOW); // SINK
  while(digitalRead(BUTTON_PIN_1) == HIGH && currentMode == MODE_SEQUENCE) {  
      stepMotorChunk(20, MODE_SEQUENCE);
  }
  if (currentMode != MODE_SEQUENCE) return; 
  
  uint32_t waitTime = 45000; 
  if (xSemaphoreTake(configLock, portMAX_DELAY)) {
    waitTime = floatWaitTimeMs;
    xSemaphoreGive(configLock);
  }
  
  uint32_t startTime = millis();
  while ((millis() - startTime) < waitTime) {
    if (currentMode != MODE_SEQUENCE) return; 
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
  
  if (currentMode != MODE_SEQUENCE) return; 
  
  digitalWrite(dirPin, HIGH); // FLOAT
  while(digitalRead(BUTTON_PIN_2) == HIGH && currentMode == MODE_SEQUENCE) {  
      stepMotorChunk(20, MODE_SEQUENCE);
  }
  
  if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
    if (currentMode == MODE_SEQUENCE) currentMode = MODE_HOLD_DEPTH;
    xSemaphoreGive(modeLock);
  }
}

// --- BANG-BANG CONTROLLER ---
void runBangBangController() {
  float depthCopy = 0.0;
  if (xSemaphoreTake(dataLock, portMAX_DELAY)) {
    depthCopy = currentRelativeDepth;
    xSemaphoreGive(dataLock);
  }

  float targetMeters = 0.40;
  if (xSemaphoreTake(configLock, portMAX_DELAY)) {
    targetMeters = targetDepthCm / 100.0;
    xSemaphoreGive(configLock);
  }

  float upperBound = targetMeters + 0.02;
  float lowerBound = targetMeters - 0.02;

  if (depthCopy > upperBound && digitalRead(BUTTON_PIN_2) == HIGH) {
    digitalWrite(dirPin, HIGH); // Too deep, float up
    stepMotorChunk(50, MODE_HOLD_DEPTH);
  } 
  else if (depthCopy < lowerBound && digitalRead(BUTTON_PIN_1) == HIGH) {
    digitalWrite(dirPin, LOW); // Too shallow, sink down
    stepMotorChunk(50, MODE_HOLD_DEPTH);
  } 
  else {
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// --- MOTOR TASK ---
void motorTask(void *parameter) {
  while(1) {
    SystemMode currentModeCopy;
    if (xSemaphoreTake(modeLock, portMAX_DELAY)) {
      currentModeCopy = currentMode;
      xSemaphoreGive(modeLock);
    }
    
    if (currentModeCopy == MODE_SEQUENCE) runStepperSequence();
    else if (currentModeCopy == MODE_HOLD_DEPTH) runBangBangController();
    else vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void setup() {
  Serial.begin(115200); 
  Wire.begin(CUSTOM_SDA_PIN, CUSTOM_SCL_PIN);
  
  // SENSOR INIT (Non-blocking status update)
  if (!sensor.init()) {
    Serial.println("Sensor Init Failed.");
    sensorOnline = false;
  } else {
    sensorOnline = true;
    sensor.setFluidDensity(997);
    sensor.read();
    surfaceDepthOffset = sensor.depth(); 
  }
  
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(BUTTON_PIN_1, INPUT_PULLUP);
  pinMode(BUTTON_PIN_2, INPUT_PULLUP); 
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);
  
  // OTA Setup
  ArduinoOTA.setPassword("ota_password"); // Set a password for OTA updates
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  
  server.on("/data", handleData);
  server.on("/control", handleControl);
  server.on("/config", handleConfig);
  server.begin();
  
  dataLock = xSemaphoreCreateMutex();
  modeLock = xSemaphoreCreateMutex();
  configLock = xSemaphoreCreateMutex();
  
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, &sensorTaskHandle, 0);
  xTaskCreatePinnedToCore(webTask, "WebTask", 4096, NULL, 1, &webTaskHandle, 0);
  xTaskCreatePinnedToCore(motorTask, "MotorTask", 4096, NULL, 3, &motorTaskHandle, 1);
}

void loop() {
  ArduinoOTA.handle(); 
  vTaskDelay(pdMS_TO_TICKS(1000));
}
