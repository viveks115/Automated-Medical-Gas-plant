#include <WiFi.h>
#include <WebServer.h>
#include "DHT.h"
#include <Stepper.h>

// WiFi credentials
const char *ssid = "Wokwi-GUEST";
const char *password = "";

// Web server on port 80
WebServer server(80);
#define MIN_PRESSURE 10
#define MAX_PRESSURE 80

// Define pins
#define PRESSURE_SENSOR_PIN 34
#define FLOW_SENSOR_PIN 35
#define BUZZER_PIN 32
#define BUTTON_PIN 26
#define VALVE1_PIN 25
#define VALVE2_PIN 33
#define DHT_PIN 23
#define DHTTYPE DHT22

// Stepper motor setup
#define STEPPER_PIN1 13
#define STEPPER_PIN2 12
#define STEPPER_PIN3 14
#define STEPPER_PIN4 27
Stepper stepperMotor(200, STEPPER_PIN1, STEPPER_PIN3, STEPPER_PIN2, STEPPER_PIN4);

// DHT sensor setup
DHT dht(DHT_PIN, DHTTYPE);

// Monitoring variables
float currentPressure = 0.0;
float currentFlowRate = 0.0;
float desiredPressure = 50.0;
float desiredFlowRate = 5.0;
bool valve1Open = false;
bool valve2Open = false;
float temperature = 0.0;
float humidity = 0.0;
uint8_t emergencyStop = 0;
uint8_t highTemperature=0;
String generateHTML()
{
  String html = R"rawliteral(
    <!DOCTYPE html>
    <html>
    <head>
      <title>Medical Gas System</title>
      <style>
        /* General styles */
        body {
          font-family: Arial, sans-serif;
          margin: 0;
          padding: 0;
          background-color: #f4f4f9;
        }
        header {
          background-color: #007bff;
          color: white;
          text-align: center;
          padding: 20px 10px;
        }
        h1 {
          margin: 0;
        }
        .container {
          max-width: 800px;
          margin: 20px auto;
          padding: 20px;
          background: white;
          border-radius: 10px;
          box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
        }
        .alert-box {
          border: 1px solid #f00;
          padding: 10px;
          background-color: #ffcccc;
          color: #900;
          margin-bottom: 20px;
          border-radius: 5px;
          display: none;
        }
        .data-block {
          display: flex;
          justify-content: space-between;
          margin: 10px 0;
          padding: 10px;
          background: #e9ecef;
          border-radius: 5px;
        }
        .data-block span {
          font-weight: bold;
        }
        .chart {
          margin: 20px 0;
        }
        .chart div {
          margin-bottom: 10px;
        }
        .bar-container {
          background: #e0e0e0;
          width: 100%;
          height: 30px;
          border-radius: 5px;
          overflow: hidden;
          position: relative;
        }
        .bar {
          background-color: #007bff;
          color: white;
          text-align: right;
          padding: 5px;
          height: 100%;
          border-radius: 5px 0 0 5px;
          position: absolute;
          top: 0;
          left: 0;
        }
        .form-group {
          margin: 15px 0;
        }
        input[type="number"], button {
          width: 100%;
          padding: 10px;
          margin: 5px 0;
          border: 1px solid #ccc;
          border-radius: 5px;
        }
        button {
          background-color: #007bff;
          color: white;
          border: none;
          cursor: pointer;
        }
        button:hover {
          background-color: #0056b3;
        }
      </style>
      <script>
        function updateData() {
          fetch('/live_data')
            .then(response => response.json())
            .then(data => {
              // Update text values
              document.getElementById('pressure').innerText = data.pressure + " units";
              document.getElementById('flow').innerText = data.flow + " units";
              document.getElementById('temperature').innerText = data.temperature + "°C";
              document.getElementById('humidity').innerText = data.humidity + " %";
              document.getElementById('valve1').innerText = data.valve1 ? "Open" : "Closed";
              document.getElementById('valve2').innerText = data.valve2 ? "Open" : "Closed";

              // Scale bar lengths to a fixed range (0 to 100)
              const pressureBar = document.getElementById('pressure-bar');
              const flowBar = document.getElementById('flow-bar');
              pressureBar.style.width = Math.min(data.pressure, 100) + "%";
              pressureBar.innerText = Math.min(data.pressure, 100) + "%";
              flowBar.style.width = Math.min(data.flow, 100) + "%";
              flowBar.innerText = Math.min(data.flow, 100) + "%";

              // Display alerts
              const alertBox = document.getElementById('alert-box');
              if (data.alert) {
                alertBox.style.display = "block";
                alertBox.innerText = data.alert;
              } else {
                alertBox.style.display = "none";
              }
            })
            .catch(error => console.error("Error fetching data:", error));
        }

        setInterval(updateData, 1000); // Update data every second
      </script>
    </head>
    <body onload="updateData()">
      <header>
        <h1>Medical Gas System</h1>
      </header>
      <div class="container">
        <!-- Alert Box -->
        <div id="alert-box" class="alert-box">Loading alerts...</div>

        <!-- Data Display -->
        <div class="data-block">
          <span>Pressure:</span>
          <span id="pressure">Loading...</span>
        </div>
        <div class="data-block">
          <span>Flow Rate:</span>
          <span id="flow">Loading...</span>
        </div>
        <div class="data-block">
          <span>Temperature:</span>
          <span id="temperature">Loading...</span>
        </div>
        <div class="data-block">
          <span>Humidity:</span>
          <span id="humidity">Loading...</span>
        </div>
        <div class="data-block">
          <span>Valve 1:</span>
          <span id="valve1">Loading...</span>
        </div>
        <div class="data-block">
          <span>Valve 2:</span>
          <span id="valve2">Loading...</span>
        </div>

        <!-- Bar Charts -->
        <div class="chart">
          <div>Pressure</div>
          <div class="bar-container">
            <div id="pressure-bar" class="bar" style="width: 0%;">0%</div>
          </div>
          <div>Flow Rate</div>
          <div class="bar-container">
            <div id="flow-bar" class="bar" style="width: 0%;">0%</div>
          </div>
        </div>

        <!-- Controls -->
        <form action="/set_pressure" method="get">
          <div class="form-group">
            <label for="pressure">Set Desired Pressure:</label>
            <input type="number" id="pressure-input" name="pressure" step="0.1" value="5.0">
          </div>
          <div class="form-group">
            <label for="flow">Set Desired Flow Rate:</label>
            <input type="number" id="flow-input" name="flow" step="0.1" value="5.0">
          </div>
          <button type="submit">Update</button>
        </form>
        <button onclick="fetch('/open_valve1', { method: 'POST' })">Open Valve 1</button>
        <button onclick="fetch('/close_valve1', { method: 'POST' })">Close Valve 1</button>
        <button onclick="fetch('/open_valve2', { method: 'POST' })">Open Valve 2</button>
        <button onclick="fetch('/close_valve2', { method: 'POST' })">Close Valve 2</button>
        <button onclick="fetch('/check_leak', { method: 'POST' })">Check for Leaks</button>
        <button onclick="fetch('/STOP', { method: 'POST' })">Emergency Stop</button>
      </div>
    </body>
    </html>
  )rawliteral";
  return html;
}

// Helper functions
void setValve(int pin, bool state, bool &valveState)
{
  digitalWrite(pin, state ? HIGH : LOW);
  valveState = state;
}

float safeReadSensor(int pin, const char *name)
{
  float value = analogRead(pin) / 4096.0 * 100;
  if (isnan(value) || value < 0.0 || value > 100.0)
  {
    Serial.printf("Error: %s sensor returned invalid value.\n", name);
    return -1; // Indicate an error
  }
  return value;
}

void updateStepperMotor(float error)
{
  Serial.println(error);
  if (error < -0.5)
    stepperMotor.step(-10); // Decrease
  else if (error > 0.5)
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
  json += "\"valve1\":" + String(valve1Open ? "true" : "false") + ",";
  json += "\"valve2\":" + String(valve2Open ? "true" : "false") + ",";

  // Alert messages
  if (emergencyStop)
  {
    if(!highTemperature)
    json += "\"alert\":\"Emergency Stop Activated. Set new pressure to resume.\"";
    else
    json += "\"alert\":\"Emergency Stop DUE TO TEMPERATURE .\"";
  }
  else if (currentPressure < MIN_PRESSURE)
  {
    json += "\"alert\":\"Pressure critically low! Check system immediately.\"";
  }
  else if (currentPressure > MAX_PRESSURE)
  {
    json += "\"alert\":\"Pressure dangerously high! Take action.\"";
  }
  else
  {
    json += "\"alert\":null";
  }

  json += "}";
  server.send(200, "application/json", json);
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
  emergencyStop = 1;
  server.send(200, "text/html", "<p>Emergency Stop Activated!</p><a href='/'>Go Back</a>");
}

void handleCheckLeak()
{
  setValve(VALVE1_PIN, false, valve1Open);
  setValve(VALVE2_PIN, false, valve2Open);

  float initialPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
  delay(5000); // Wait for pressure stabilization
  float finalPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");

  if (initialPressure - finalPressure > 0.5)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    server.send(200, "text/html", "<p>Leak Detected! Alert triggered.</p><a href='/'>Go Back</a>");
  }
  else
  {
    server.send(200, "text/html", "<p>No Leak Detected.</p><a href='/'>Go Back</a>");
  }

  digitalWrite(BUZZER_PIN, LOW);
}

// Setup and loop
void setup()
{
  Serial.begin(115200);

  // Pin configuration
  pinMode(PRESSURE_SENSOR_PIN, INPUT);
  pinMode(FLOW_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(VALVE1_PIN, OUTPUT);
  pinMode(VALVE2_PIN, OUTPUT);

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
  server.begin();
  Serial.println("Web server started!");
}

void loop()
{
  // Sensor updates
  currentPressure = safeReadSensor(PRESSURE_SENSOR_PIN, "Pressure");
  currentFlowRate = safeReadSensor(FLOW_SENSOR_PIN, "Flow");
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();

  if (!emergencyStop)
  {
    float error = desiredPressure - currentPressure;
    updateStepperMotor(error);

    // Valve control logic
    if (currentPressure <= 10 || currentPressure >= 80)//backup
    {
      setValve(VALVE1_PIN, false, valve1Open);
      setValve(VALVE2_PIN, true, valve2Open);
    }
    else//main valve
    {
      setValve(VALVE1_PIN, true, valve1Open);
      setValve(VALVE2_PIN, false, valve2Open);
    }
  }
    if (temperature <=-10||temperature >60)
    {
        emergencyStop=1;
        highTemperature=1;
        setValve(VALVE1_PIN, false, valve1Open);
        setValve(VALVE2_PIN, false, valve2Open);
    }
    else{
    highTemperature=0;

    }
  

  server.handleClient(); // Handle web server requests
  delay(500);
}
