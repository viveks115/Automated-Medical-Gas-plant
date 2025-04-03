#include <WiFi.h>
#include <WebServer.h>
#include "DHT.h"
// #include <Stepper.h>
#include <AccelStepper.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>
String generateHTML();
#define RX_PIN 25 // Define RX pin for communication with the sensor
#define TX_PIN 33 // Define TX pin for communication with the sensor

SoftwareSerial oxygenSensor(RX_PIN, TX_PIN); // Create software serial object

// WiFi credentials
const char *ssid = "STUDENT";
const char *password = "1234567890";

// Web server on port 80
WebServer server(80);
// #define MIN_PRESSURE 10
// #define MAX_PRESSURE 80

#define STEP_PIN 21
#define DIR_PIN 19
#define ENABLE_PIN 5
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define pins
#define PRESSURE_SENSOR_PIN 34
#define FLOW_SENSOR_PIN 25
#define BUZZER_PIN 32
#define BUTTON_PIN 35
#define VALVE1_PIN 14
#define VALVE2_PIN 27
#define VALVE3_PIN 26
#define VALVE4_PIN 12 // 12 cause boot issue
#define DHT_PIN 13
#define DHTTYPE DHT22

// Stepper motor setup
#define STEPPER_PIN1 21 //
#define STEPPER_PIN2 19
#define STEPPER_PIN3 18
#define STEPPER_PIN4 5
// Stepper stepperMotor(2048, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);

// DHT sensor setup
DHT dht(DHT_PIN, DHTTYPE);

// Monitoring variables
int humanPass = 0;
float currentPressure = 0.0;
float currentFlowRate = 0.0;
float desiredPressure = 30.0;
float desiredFlowRate = 5.0;
float currentOconcentration = 0.0;
bool valve1Open = false;
bool valve2Open = false;
bool valve3Open = false;
float temperature = 0.0;
float gasTemp = 0.0;
float humidity = 0.0;
uint8_t emergencyStop = 0;
uint8_t highTemperature = 0;
uint8_t manualMode = 0;
float PRESSURE_MIN = 0;  // Minimum acceptable pressure
float PRESSURE_MAX = 0;  // Maximum acceptable pressure
#define BUZZER_CHANNEL 0 // ESP32 has 16 PWM channels (0-15)

bool alarmActive = false;
unsigned long alarmEndTime = 0;
// Function to start a buzzer tone (Non-Blocking)
void startAlarm(int frequency, int duration)
{
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);
  ledcSetup(BUZZER_CHANNEL, frequency, 8);  // 8-bit resolution
  ledcWriteTone(BUZZER_CHANNEL, frequency); // Start tone
  alarmActive = true;
  alarmEndTime = millis() + duration;
}

// Function to stop the buzzer
void stopAlarm()
{
  ledcWriteTone(BUZZER_CHANNEL, 0); // Stop tone
  alarmActive = false;
}

// Function to handle non-blocking alarm duration
void updateAlarm()
{
  if (alarmActive && millis() >= alarmEndTime)
  {
    stopAlarm();
  }
}
// Helper functions
void setValve(int pin, bool state, bool &valveState)
{
  digitalWrite(pin, state ? LOW : HIGH);
  valveState = state;
}
char getCheckSum(char *getbuff)
{
  char i = 0, checksum = 0;
  for (i = 0; i < 11; i++)
  {
    checksum += getbuff[i];
  }
  checksum = 0x00 - checksum;
  return checksum;
}
void flowSensor()
{
  uint8_t buffer[12];
  uint8_t index = 0;
  while (oxygenSensor.available())
  { // Read incoming data
    buffer[index] = oxygenSensor.read();

    if (index == 0 && buffer[0] != 0x16)
    {
      // Ignore if first byte is not the expected start byte (0x16)
      continue;
    }

    index++;

    if (index >= 12)
    {            // Full packet received
      index = 0; // Reset index for next message
      if (getCheckSum((char *)buffer) == buffer[11])
      {
        // Extract oxygen concentration
        uint16_t concentration = (buffer[3] << 8) | buffer[4];
        currentOconcentration = (float)concentration / 10.0; // Convert to percentage

        // Extract flow rate
        uint16_t flow = (buffer[5] << 8) | buffer[6];
        currentFlowRate = (float)flow / 10.0; // Convert to L/min

        // Extract temperature
        uint16_t temperature = (buffer[7] << 8) | buffer[8];
        gasTemp = (float)temperature / 10.0; // Convert to °C
        /*
        Serial.print("Oxygen Concentration: ");
        Serial.print(currentOconcentration);
        Serial.println(" %");

        Serial.print("Flow Rate: ");
        Serial.print(currentFlowRate);
        Serial.println(" L/min");

        Serial.print("Temperature in flowmeter: ");
        Serial.print(gasTemp);
        Serial.println(" °C");
        */
      }
      else
      {
        Serial.println("Checksum error, data might be corrupted!");
      }
    }
    else
    {
      //
    }
  }
}
float safeReadSensor(int pin, const char *name)
{                          // Constants
#define ADC_MIN 455        // ADC value at 0 MPa
#define ADC_MAX 5588       // ADC value at 1.2 MPa
#define PRESSURE_RANGE 1.2 // Pressure range in MPa
#define PRESSURE_ERROR 3
  float value = 0;
  if (pin == PRESSURE_SENSOR_PIN)
  {
    int adc_value = analogRead(PRESSURE_SENSOR_PIN); // Read the analog value
    if (adc_value < ADC_MIN)
      adc_value = ADC_MIN;
    if (adc_value > ADC_MAX)
      adc_value = ADC_MAX;
    value = ((float)(adc_value - ADC_MIN) / (ADC_MAX - ADC_MIN)) * PRESSURE_RANGE;
    value *= 145.0377; // for psi conversion
    value += PRESSURE_ERROR;
  }
  else
  {
  }
  if (isnan(value) || value < 0.0) //|| value > 100.0)
  {
    Serial.printf("Error: %s sensor returned invalid value.\n", name);
    return -1; // Indicate an error
  }
  return value;
}

void updateStepperMotor(float error)
{
  Serial.println(error);
  if (error < -10)
  {
    // stepper.step(-10); // Decrease
  }
  else if (error > 20)
  {
    // stepper.step(10); // Increase
  }
}
void handleLiveData()
{
  // StaticJsonDocument<256> jsonDoc; // Use ArduinoJson for safety
  JsonDocument jsonDoc;
  jsonDoc["pressure"] = isnan(currentPressure) ? 0.0 : currentPressure;
  jsonDoc["flow"] = isnan(currentFlowRate) ? 0.0 : currentFlowRate;
  jsonDoc["roomtemp"] = isnan(temperature) ? 0.0 : temperature;
  jsonDoc["gastemp"] = isnan(gasTemp) ? 0.0 : gasTemp;
  jsonDoc["humidity"] = isnan(humidity) ? 0.0 : humidity;
  jsonDoc["concentration"] = isnan(currentOconcentration) ? 0.0 : currentOconcentration;

  jsonDoc["valve1"] = valve1Open;
  jsonDoc["valve2"] = valve2Open;
  jsonDoc["valve3"] = valve3Open;

  if (emergencyStop)
  {
    jsonDoc["alert"] = highTemperature ? "Emergency Stop DUE TO TEMPERATURE." : "Emergency Stop Activated. Set new pressure to resume.";
  }
  else if (currentPressure < PRESSURE_MIN)
  {
    jsonDoc["alert"] = "Pressure critically low! Check system immediately.";
  }
  else if (currentPressure > PRESSURE_MAX)
  {
    jsonDoc["alert"] = "Pressure dangerously high! Take action.";
  }
  else if (manualMode)
  {
    jsonDoc["alert"] = "Manual Mode Activated";
  }
  else
  {
    jsonDoc["alert"] = "";
  }

  String jsonResponse;
  serializeJson(jsonDoc, jsonResponse); // Convert JSON object to string
  server.send(200, "application/json", jsonResponse);
}

void handleValve1Close()
{
  setValve(VALVE1_PIN, false, valve1Open);
  // digitalWrite(VALVE1_PIN, LOW);
  // valve1Open = false;
}
void handleValve1Open()
{
  setValve(VALVE1_PIN, true, valve1Open);
  // digitalWrite(VALVE1_PIN, HIGH);
  // valve1Open = true;
}
void handleValve2Close()
{
  setValve(VALVE2_PIN, false, valve2Open);
  // digitalWrite(VALVE2_PIN, LOW);
  // valve2Open = false;
}
void handleValve2Open()
{
  setValve(VALVE2_PIN, true, valve2Open);
  // digitalWrite(VALVE2_PIN, HIGH);
  // valve2Open = true;
}
void handleValve3Close()
{
  setValve(VALVE3_PIN, false, valve3Open);
  // digitalWrite(VALVE3_PIN, LOW);
  // valve3Open = false;
}
void handleValve3Open()
{
  setValve(VALVE3_PIN, true, valve3Open);
  // digitalWrite(VALVE3_PIN, HIGH);
  // valve1Open = true;
}
void handleRoot()
{
  server.send(200, "text/html", generateHTML());
}

void handleSetPressure()
{
  emergencyStop = 0; // Reset emergency stop
  if (server.hasArg("pressure"))
    desiredPressure = server.arg("pressure").toFloat();
  if (server.hasArg("flow"))
    desiredFlowRate = server.arg("flow").toFloat();
  server.send(200, "text/html", "<p>Updated successfully!</p><a href='/'>Go Back</a>");
}

void handleStop()
{
  setValve(VALVE1_PIN, false, valve1Open);
  setValve(VALVE2_PIN, false, valve2Open);
  setValve(VALVE3_PIN, false, valve3Open);
  emergencyStop = 1;
  server.send(200, "text/html", "<p>Emergency Stop Activated!</p><a href='/'>Go Back</a>");
}

void handleCheckLeak()
{
  Serial.print("checking lekage ");
  setValve(VALVE1_PIN, false, valve1Open);
  setValve(VALVE2_PIN, false, valve2Open);
  setValve(VALVE3_PIN, false, valve3Open);
  float initialPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
  delay(5000); // Wait for pressure stabilization
  float finalPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");

  if (initialPressure - finalPressure > 20)
  {
    // tone(BUZZER_PIN, 1000);
    server.send(200, "text/html", "<p>Leak Detected! Alert triggered.</p><a href='/'>Go Back</a>");
    // showAlert("Leak Detected! Alert triggered.");
  }
  else
  {
    server.send(200, "text/html", "<p>No Leak Detected.</p><a href='/'>Go Back</a>");
  }

  // tone(BUZZER_PIN, 0);
}
void handleManualMode()
{
  manualMode = 1;
  Serial.println("MANUAL MODE");
}

void controlValves()
{
  constexpr float ERROR_THRESHOLD = 5.0;
  static bool usingBackup = false; // Track the last active valve
  static bool mainInRange = 0;
  static bool backupInRange = 0;
  if (humanPass)
  {
    if (!digitalRead(BUTTON_PIN))
    {
      humanPass = 0; // Reset human intervention mode
      stopAlarm();
      Serial.println("System reset by user. Rechecking pressure sources...");
    }
    else
    {
      return; // Stay in humanPass mode until button press
    }
  }

  currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
  if (!usingBackup)
  {
    mainInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
    backupInRange = false;
  }
  if (!mainInRange)
  {
    setValve(VALVE1_PIN, false, valve1Open); // Close Main Valve
    setValve(VALVE2_PIN, true, valve2Open);  // Open Backup Valve
    delay(1000);                             // Allow pressure stabilization
    currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
    backupInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
  }

  if (mainInRange)
  {
    setValve(VALVE1_PIN, true, valve1Open);  // Open Main Valve
    setValve(VALVE2_PIN, false, valve2Open); // Close Backup Valve
    usingBackup = false;
    stopAlarm();
  }
  else if (backupInRange)
  {
    usingBackup = true;
  }
  else
  {
    if (usingBackup)
    {
      setValve(VALVE1_PIN, true, valve1Open); // Retry Main Valve
      setValve(VALVE2_PIN, false, valve2Open);
      delay(1000);
      currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");

      float error = desiredPressure - currentPressure;
      if (abs(error) > ERROR_THRESHOLD)
      {
        stepper.move(error > 0 ? 10 : -10); // Adjust Pressure
        stepper.run();
      }
      mainInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
    }

    if (!mainInRange)
    {
      setValve(VALVE1_PIN, false, valve1Open);
      setValve(VALVE2_PIN, false, valve2Open);
      startAlarm(1000, 1000);
      humanPass = 1; // Enter human intervention mode
      Serial.println("CRITICAL ALERT: No available pressure source! System shutting down.");
      return;
    }
  }
}

// void controlValves()
// {

//   constexpr float ERROR_THRESHOLD = 5.0; // Acceptable pressure error before stepper correction
//   currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
//   bool mainInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
//   bool backupInRange = false;

//   if (mainInRange)
//   {
//     stopAlarm();
//     setValve(VALVE1_PIN, true, valve1Open);  // Open Main Valve
//     setValve(VALVE2_PIN, false, valve2Open); // Close Backup Valve
//   }
//   else
//   {
//     // startAlarm(2000, 5000);
//     setValve(VALVE1_PIN, false, valve1Open); // Close Main Valve
//     setValve(VALVE2_PIN, true, valve2Open);  // Open Backup Valve
//     delay(1000);
//     currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
//     backupInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
//     // Adjust pressure using stepper motor
//     float error = desiredPressure - currentPressure;
//     if (error > ERROR_THRESHOLD)
//     {
//       stepper.move(-10); // Reduce Pressure
//     }
//     else if (error < -ERROR_THRESHOLD)
//     {
//       stepper.move(10); // Increase Pressure
//     }
//     stepper.run(); // Non-blocking step execution

//     if (!backupInRange)
//     {
//       setValve(VALVE1_PIN, true, valve1Open);
//       setValve(VALVE2_PIN, false, valve2Open);
//       currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
//       mainInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);

//       if (!mainInRange)
//       {
//         setValve(VALVE1_PIN, false, valve1Open);
//         setValve(VALVE2_PIN, false, valve2Open);

//         startAlarm(1000, 1000);
//         humanPass = 1;
//         Serial.println("CRITICAL ALERT: No available pressure source! System shutting down.");
//       }
//       else
//       {
//         humanPass = 0;
//         mainInRange = true;
//         backupInRange = false;
//         stopAlarm();
//       }
//     }
//   }
// }

void setup()
{
  Serial.begin(9600);

  oxygenSensor.begin(9600);

  stepper.setMaxSpeed(600);       // Set default max speed
  stepper.setAcceleration(10000); // Set default acceleration

  // Pin configuration
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  pinMode(VALVE1_PIN, OUTPUT);
  pinMode(VALVE2_PIN, OUTPUT);
  pinMode(VALVE3_PIN, OUTPUT);
  // pinMode(VALVE4_PIN, OUTPUT);
  setValve(VALVE1_PIN, true, valve1Open);  // Open Main Valve
  setValve(VALVE2_PIN, false, valve2Open); // Close Backup Valve

  dht.begin(); // Initialize DHT sensor

  // WiFi initialization
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Web server routes
  server.on("/live_data", handleLiveData);
  server.on("/", handleRoot);
  server.on("/set_pressure", handleSetPressure);
  server.on("/STOP", handleStop);
  server.on("/check_leak", handleCheckLeak);
  server.on("/manual_mode", handleManualMode);
  server.on("/close_valve1", handleValve1Close);
  server.on("/open_valve1", handleValve1Open);
  server.on("/close_valve2", handleValve2Close);
  server.on("/open_valve2", handleValve2Open);
  server.on("/close_valve3", handleValve3Close);
  server.on("/open_valve3", handleValve3Open);

  server.begin();
  Serial.println("Web server started!");
}

void loop()
{

  // Sensor updates
  currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
  PRESSURE_MAX = desiredPressure + 10;
  PRESSURE_MIN = desiredPressure - 10;
  flowSensor();
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  /*
    Serial.print("Pressure: ");
    Serial.print(currentPressure);
    Serial.println(" psi");
    Serial.print("Temperature in DHT22: ");
    Serial.print(temperature);
    Serial.println(" 'C");
    Serial.print("Humidity in DHT22: ");
    Serial.print(humidity);
    Serial.println(" %");
    */

  if (!emergencyStop)
  {
    if (!manualMode)
    {
      controlValves();
    }
    else
    {

      // THE THINGS TO DONE IN MANUAL OPERATION CONTROL THE VALE OPENING AND CLOSING VALVE ACCORDING TO USER
      if (!digitalRead(BUTTON_PIN))
      {
        // exit from manual mode
        manualMode = 0;
        Serial.println("EXITED MANUAL MODE");
      }
    }
  }
  if (temperature <= -10 || temperature > 60)
  {
    handleStop();
    highTemperature = 1;
  }
  else
  {
    highTemperature = 0;
  }

  server.handleClient(); // Handle web server requests
  delay(500);
  updateAlarm();
}
