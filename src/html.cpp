#include <WString.h>

String generateHTML()
{
  String html = R"rawliteral(
   <!DOCTYPE html>
<html>
  <head>
    <title>Medical Gas System</title>
    <style>
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
      .form-group {
        margin: 15px 0;
      }
      input[type="number"],
      button {
        width: 100%;
        padding: 10px;
        margin: 5px 0;
        border: 1px solid #ccc;
        border-radius: 5px;
      }
      table {
        width: 100%;
        border-collapse: collapse;
        margin: 5px 0;
      }

      td {
        padding: 5px;
        text-align: center;
      }
      button {
        width: 100%;
        padding: 10px;
        background-color: #007bff;
        color: white;
        border: none;
        border-radius: 5px;
        cursor: pointer;
        margin-top: 10px;
      }

      /* button {
        background-color: #007bff;
        padding: 10px;
        color: white;
        border: none;
        cursor: pointer;
        width: 100%;
        margin-top: 10px;
        border-radius: 5px;
      } */
      button:hover {
        background-color: #0056b3;
      }
    </style>
    <script>
      function sendCommand(endpoint) {
        fetch(endpoint, { method: "POST" })
          .then((response) => response.json())
          .then((data) => {
            alert(data.message || "Action completed successfully");
          })
          .catch((error) => console.error("Error:", error));
      }

      function updateData() {
        fetch("/live_data")
          .then((response) => response.json())
          .then((data) => {
            document.getElementById("pressure").innerText =
              data.pressure + " PSI";
            document.getElementById("flow").innerText = data.flow + " L/min";
            document.getElementById("temperature").innerText =
              data.temperature + " 'C";
            document.getElementById("humidity").innerText =
              data.humidity + " %";
              document.getElementById("concentration").innerText = data.concentration + " %";
            document.getElementById("valve1").innerText = data.valve1
              ? "Open"
              : "Closed";
            document.getElementById("valve2").innerText = data.valve2
              ? "Open"
              : "Closed";
            document.getElementById("valve3").innerText = data.valve3
              ? "Open"
              : "Closed";

            const alertBox = document.getElementById("alert-box");
            alertBox.style.display = data.alert ? "block" : "none";
            alertBox.innerText = data.alert || "";
          })
          .catch((error) => console.error("Error fetching data:", error));
      }
      setInterval(updateData, 1000);
    </script>
  </head>
  <body onload="updateData()">
    <header>
      <h1>Medical Gas System</h1>
    </header>
    <div class="container">
      <div id="alert-box" class="alert-box"></div>
      <div class="data-block">
        <span>Pressure:</span><span id="pressure">Loading...</span>
      </div>
      <div class="data-block">
        <span>Flow Rate:</span><span id="flow">Loading...</span>
      </div>
      <div class="data-block">
        <span>Temperature:</span><span id="temperature">Loading...</span>
      </div>
      <div class="data-block">
        <span>Humidity:</span><span id="humidity">Loading...</span>
      </div>
      <div class="data-block">
        <span>O2 concentration:</span><span id="concentration">Loading...</span>
      </div>
      <div class="data-block">
        <span>Valve 1:</span><span id="valve1">Loading...</span>
      </div>
      <div class="data-block">
        <span>Valve 2:</span><span id="valve2">Loading...</span>
      </div>
      <div class="data-block">
        <span>Valve 3:</span><span id="valve3">Loading...</span>
      </div>
      <!-- Controls -->
      <form action="/set_pressure" method="get">
        <div class="form-group">
          <label for="pressure">Set Desired Pressure(psi):</label>
          <input
            type="number"
            id="pressure-input"
            name="pressure"
            step="1"
            value="50"
          />
        </div>
        <!--<div class="form-group">-->
        <!--  <label for="flow">Set Desired Flow Rate:</label>-->
        <!--  <input type="number" id="flow-input" name="flow" step="0.1" value="5.0">-->
        <!--</div>-->
        <button type="submit">Update</button>
      </form>

      <button onclick="sendCommand('/manual_mode')">Manual Mode</button>
      <table>
        <tr>
          <td>
            <button onclick="sendCommand('/open_valve1')">Open Valve 1</button>
          </td>
          <td>
            <button onclick="sendCommand('/close_valve1')">Close Valve 1</button>
          </td>
        </tr>
        <tr>
          <td>
            <button onclick="sendCommand('/open_valve2')">Open Valve 2</button>
          </td>
          <td>
            <button onclick="sendCommand('/close_valve2')">
              Close Valve 2
            </button>
          </td>
        </tr>
        <tr>
          <td>
            <button onclick="sendCommand('/open_valve3')">Open Valve 3</button>
          </td>
          <td>
            <button onclick="sendCommand('/close_valve3')">
              Close Valve 3
            </button>
          </td>
        </tr>
      </table>
      <button onclick="sendCommand('/check_leak')">Check for Leaks</button>
      <button onclick="sendCommand('/STOP')">Emergency Stop</button>
    </div>
  </body>
</html>

  )rawliteral";
  return html;
}
