#include <WiFi.h>
#include <WebServer.h>
#include "DHT.h"
#include <Stepper.h>
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
Stepper stepperMotor(2048, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);

// DHT sensor setup
DHT dht(DHT_PIN, DHTTYPE);

// Monitoring variables
float currentPressure = 0.0;
float currentFlowRate = 0.0;
float desiredPressure = 20.0;
float desiredFlowRate = 5.0;
float currentOconcentration = 0.0;
bool valve1Open = false;
bool valve2Open = false;
bool valve3Open = false;
float temperature = 0.0;
float humidity = 0.0;
uint8_t emergencyStop = 0;
uint8_t highTemperature = 0;
uint8_t manualMode = 0;
float PRESSURE_MIN = 70; // Minimum acceptable pressure
float PRESSURE_MAX = 80; // Maximum acceptable pressure
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
  // if (oxygenSensor.available() >0)
  // { // Check if we received the full 12-byte response
  uint8_t buffer[12];
  uint8_t index = 0;
  //   for (int i = 0; i < 12; i++)
  //   {
  //     buffer[i] = oxygenSensor.read();
  //     Serial.print("buffer=");
  //     Serial.print(i);
  //     Serial.println(buffer[i]);
  //   }
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
        float temp = temperature / 10.0; // Convert to °C

        /*
          Serial.print("Oxygen Concentration: ");
          Serial.print(currentOconcentration);
          Serial.println(" %");

          Serial.print("Flow Rate: ");
          Serial.print(currentFlowRate);
          Serial.println(" L/min");

          Serial.print("Temperature in flowmeter: ");
          Serial.print(temp);
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
    stepperMotor.step(-10); // Decrease
  else if (error > 20)
    stepperMotor.step(10); // Increase
}
// Web server handlers
void handleLiveData()
{
  String json = "{";
  json += "\"pressure\":" + String(currentPressure) + ",";
  json += "\"flow\":" + String(currentFlowRate) + ",";
  json += "\"temperature\":" + String(temperature) + ",";
  json += "\"humidity\":" + String(humidity) + ",";
  json += "\"concentration\":" + String(currentOconcentration) + ",";
  json += "\"valve1\":" + String(valve1Open ? "true" : "false") + ",";
  json += "\"valve2\":" + String(valve2Open ? "true" : "false") + ",";
  json += "\"valve3\":" + String(valve3Open ? "true" : "false") + ",";
  if (emergencyStop)
  {
    if (!highTemperature)
      json += "\"alert\":\"Emergency Stop Activated. Set new pressure to resume.\"";
    else
      json += "\"alert\":\"Emergency Stop DUE TO TEMPERATURE .\"";
  }
  else if (currentPressure < PRESSURE_MIN)
  {
    json += "\"alert\":\"Pressure critically low! Check system immediately.\"";
  }
  else if (currentPressure > PRESSURE_MAX)
  {
    json += "\"alert\":\"Pressure dangerously high! Take action.\"";
  }
  else if (manualMode)
  {
    json += "\"alert\":\"ManualMode Activated\"";
  }
  else
  {
    json += "\"alert\":null";
  }

  json += "}";
  server.send(200, "application/json", json);
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
    tone(BUZZER_PIN, 1000);
    server.send(200, "text/html", "<p>Leak Detected! Alert triggered.</p><a href='/'>Go Back</a>");
    // showAlert("Leak Detected! Alert triggered.");
  }
  else
  {
    server.send(200, "text/html", "<p>No Leak Detected.</p><a href='/'>Go Back</a>");
  }

  tone(BUZZER_PIN, 0);
}
void handleManualMode()
{
  manualMode = 1;
  Serial.println("MANUAL MODE");
}
void controlValves(float currentPressure, float desiredPressure)
{

  constexpr float ERROR_THRESHOLD = 5.0; // Acceptable pressure error before stepper correction

  bool mainInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
  bool backupInRange = false;

  if (mainInRange)
  {
    tone(BUZZER_PIN, 0);
    // Main source is available → Use Main Source
    setValve(VALVE1_PIN, true, valve1Open);  // Open Main Valve
    setValve(VALVE2_PIN, false, valve2Open); // Close Backup Valve

    // Adjust pressure using stepper motor
    float error = desiredPressure - currentPressure;
    if (error > ERROR_THRESHOLD)
    {
      stepperMotor.step(-10); // Reduce Pressure
    }
    else if (error < -ERROR_THRESHOLD)
    {
      stepperMotor.step(10); // Increase Pressure
    }
  }
  else
  {
    startAlarm(2000, 5000);
    //  Main Source Out of Range → Switch to Backup
    setValve(VALVE1_PIN, false, valve1Open); // Close Main Valve
    setValve(VALVE2_PIN, true, valve2Open);  // Open Backup Valve

    // Recheck pressure with backup source
    backupInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);

    if (!backupInRange)
    {
      // Serial.println("CRITICAL ALERT:BACKUP ACTIVE");
      //  Backup also failed → Recheck Main Source
      mainInRange = (currentPressure >= PRESSURE_MIN && currentPressure <= PRESSURE_MAX);
      static int humanPass=0;
      if (!mainInRange)
      {
        // No source is available → Shut Down & Alert
        setValve(VALVE1_PIN, false, valve1Open);
        setValve(VALVE2_PIN, false, valve2Open);
        
        if (humanPass == 0)
        {
          startAlarm(1000, 1000);
          if(!digitalRead(BUTTON_PIN))
          {
            humanPass=1;
            stopAlarm(); 
          }
        }
        Serial.println("CRITICAL ALERT: No available pressure source! System shutting down.");
      }
      else
      {
        humanPass=0;
        mainInRange = true;
        backupInRange = false;
        tone(BUZZER_PIN, 0);
      }
    }
  }
}

// Setup and loop
void setup()
{
  Serial.begin(9600);
  oxygenSensor.begin(9600);
  stepperMotor.setSpeed(10);
  // Pin configuration
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  // pinMode(FLOW_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VALVE1_PIN, OUTPUT);
  pinMode(VALVE2_PIN, OUTPUT);
  pinMode(VALVE3_PIN, OUTPUT);
  // pinMode(VALVE4_PIN, OUTPUT);

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
      controlValves(currentPressure, desiredPressure);
    }
    else
    {

      // THE THINGS TO DONE IN MANUAL OPERATION CONTROL THE VALE OPENING AND CLOSING VALVE ACCORDING TO USER
      if (digitalRead(BUTTON_PIN))
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
