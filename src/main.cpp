#include <ArduinoJson.h>
#include <AsyncTCP.h>
#include <ESP32Servo.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <cmath>

// GPIO mapping for each servo
constexpr int servoPins[] = {4, 5, 12, 13, 14, 18, 19};
// Index order: base, armA, armB, wristA, wristB, gripper

// Wi-Fi credentials and websocket setup
constexpr char WIFI_SSID[] = "Apt 210";
constexpr char WIFI_PASSWORD[] = "mistycanoe3";
constexpr uint16_t HTTP_PORT = 80;
constexpr char WS_PATH[] = "/arm";

// Per-joint home and limit configuration (adjust as needed for your rig)
constexpr float HOME_ANGLES[6] = {90.0f, 90.0f, 90.0f, 90.0f, 90.0f, 90.0f};
constexpr float MIN_ANGLES[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
constexpr float MAX_ANGLES[6] = {180.0f, 180.0f, 180.0f, 180.0f, 180.0f, 180.0f};

// Slew limiting
constexpr float MAX_DEG_PER_STEP = 10.0f;      // limit per update tick
constexpr unsigned long SLEW_INTERVAL_MS = 20; // tick period

// Program execution structures
constexpr int MAX_WAYPOINTS = 20;
struct Waypoint
{
  unsigned long t; // timestamp in ms
  float joints[6]; // target angles for all joints
};

struct ProgramState
{
  bool isRunning;
  int waypointCount;
  int currentWaypointIndex;
  Waypoint waypoints[MAX_WAYPOINTS];
  unsigned long programStartTime;
  float startAngles[6]; // angles when current segment started
} programState = {false, 0, 0, {}, 0, {}};

Servo servo1; // Base (Root)
Servo servo2; // Arm A1
Servo servo3; // Arm A2 (mirrored)
Servo servo4; // Arm B
Servo servo5; // Wrist A
Servo servo6; // Wrist B
Servo servo7; // Gripper

AsyncWebServer server(HTTP_PORT);
AsyncWebSocket ws(WS_PATH);

float currentAngles[6];
float targetAngles[6];
unsigned long lastSlewMs = 0;

float clampAngle(float value, int jointIndex)
{
  return constrain(value, MIN_ANGLES[jointIndex], MAX_ANGLES[jointIndex]);
}

void writeServos()
{
  servo1.write(currentAngles[0]);          // Base
  servo2.write(currentAngles[1]);          // Arm A1
  servo3.write(180.0f - currentAngles[1]); // Arm A2 mirrored
  servo4.write(currentAngles[2]);          // Arm B
  servo5.write(currentAngles[3]);          // Wrist A
  servo6.write(currentAngles[4]);          // Wrist B
  servo7.write(currentAngles[5]);          // Gripper
}

void stopProgram()
{
  if (programState.isRunning)
  {
    Serial.println("Program stopped");
  }
  programState.isRunning = false;
  programState.waypointCount = 0;
  programState.currentWaypointIndex = 0;
  programState.programStartTime = 0;
}

void updateProgramExecution()
{
  if (!programState.isRunning)
  {
    return;
  }

  unsigned long elapsed = millis() - programState.programStartTime;

  // Check if we've completed all waypoints
  if (programState.currentWaypointIndex >= programState.waypointCount - 1)
  {
    // Check if we've reached the final waypoint's time
    unsigned long finalTime = programState.waypoints[programState.waypointCount - 1].t;
    if (elapsed >= finalTime)
    {
      // Set final targets and stop
      for (int i = 0; i < 6; i++)
      {
        targetAngles[i] = programState.waypoints[programState.waypointCount - 1].joints[i];
      }
      stopProgram();
      Serial.println("Program completed");
      return;
    }
  }

  // Find current segment (between currentWaypointIndex and currentWaypointIndex+1)
  int nextIndex = programState.currentWaypointIndex + 1;
  if (nextIndex >= programState.waypointCount)
  {
    return;
  }

  Waypoint &currentWP = programState.waypoints[programState.currentWaypointIndex];
  Waypoint &nextWP = programState.waypoints[nextIndex];

  // Check if we should advance to next segment
  if (elapsed >= nextWP.t)
  {
    programState.currentWaypointIndex++;
    // Store current angles as start of new segment
    for (int i = 0; i < 6; i++)
    {
      programState.startAngles[i] = currentAngles[i];
    }
    Serial.printf("Advanced to waypoint %d\n", programState.currentWaypointIndex + 1);
    return; // Process on next iteration
  }

  // Interpolate between current and next waypoint
  unsigned long segmentDuration = nextWP.t - currentWP.t;
  unsigned long segmentElapsed = elapsed - currentWP.t;

  if (segmentDuration == 0)
  {
    return; // Avoid division by zero
  }

  float progress = (float)segmentElapsed / (float)segmentDuration;
  progress = constrain(progress, 0.0f, 1.0f);

  // Linear interpolation for each joint
  for (int i = 0; i < 6; i++)
  {
    float start = currentWP.joints[i];
    float end = nextWP.joints[i];
    targetAngles[i] = start + (end - start) * progress;
  }
}

void updateServosSlew()
{
  unsigned long now = millis();
  if (now - lastSlewMs < SLEW_INTERVAL_MS)
  {
    return;
  }
  lastSlewMs = now;

  // Update program execution (calculates new targets if program running)
  updateProgramExecution();

  for (int i = 0; i < 6; i++)
  {
    float diff = targetAngles[i] - currentAngles[i];
    if (fabs(diff) < 0.01f)
    {
      continue;
    }

    float step = min(fabs(diff), MAX_DEG_PER_STEP);
    currentAngles[i] += (diff > 0 ? step : -step);
  }

  writeServos();
}

void handleJsonPayload(const char *message, size_t length)
{
  StaticJsonDocument<1024> doc; // Increased size for program payloads
  DeserializationError err = deserializeJson(doc, message, length);
  if (err)
  {
    Serial.print("JSON parse failed: ");
    Serial.println(err.c_str());
    return;
  }

  // Handle emergency stop command
  if (doc.containsKey("emergencyStop") && doc["emergencyStop"].as<bool>())
  {
    Serial.println("EMERGENCY STOP received");
    stopProgram();
    // Stop all motion by setting targets to current positions
    for (int i = 0; i < 6; i++)
    {
      targetAngles[i] = currentAngles[i];
    }
    return;
  }

  // Handle program execution
  if (doc.containsKey("type") && doc["type"].as<String>() == "program")
  {
    stopProgram(); // Stop any existing program

    if (!doc.containsKey("program"))
    {
      Serial.println("Program payload missing 'program' field");
      return;
    }

    JsonObject program = doc["program"].as<JsonObject>();
    if (!program.containsKey("waypoints"))
    {
      Serial.println("Program missing waypoints");
      return;
    }

    JsonArray waypoints = program["waypoints"].as<JsonArray>();
    int receivedCount = min((int)waypoints.size(), MAX_WAYPOINTS - 1); // Leave room for lead-in waypoint

    if (receivedCount < 1)
    {
      Serial.println("Program needs at least 1 waypoint");
      return;
    }

    // Lead-in time to move from current position to first waypoint (ms)
    constexpr unsigned long LEAD_IN_MS = 1000;

    // Insert current position as waypoint 0 (lead-in from where robot actually is)
    programState.waypoints[0].t = 0;
    for (int i = 0; i < 6; i++)
    {
      programState.waypoints[0].joints[i] = currentAngles[i];
    }

    // Load received waypoints, shifting times by LEAD_IN_MS
    for (int i = 0; i < receivedCount; i++)
    {
      JsonObject wp = waypoints[i].as<JsonObject>();
      // Shift all waypoint times forward by lead-in duration
      programState.waypoints[i + 1].t = wp["t"].as<unsigned long>() + LEAD_IN_MS;

      JsonObject joints = wp["joints"].as<JsonObject>();
      programState.waypoints[i + 1].joints[0] = clampAngle(joints["base"].as<float>(), 0);
      programState.waypoints[i + 1].joints[1] = clampAngle(joints["armA"].as<float>(), 1);
      programState.waypoints[i + 1].joints[2] = clampAngle(joints["armB"].as<float>(), 2);
      programState.waypoints[i + 1].joints[3] = clampAngle(joints["wristA"].as<float>(), 3);
      programState.waypoints[i + 1].joints[4] = clampAngle(joints["wristB"].as<float>(), 4);
      programState.waypoints[i + 1].joints[5] = clampAngle(joints["gripper"].as<float>(), 5);
    }

    // Total waypoints = lead-in + received
    programState.waypointCount = receivedCount + 1;
    programState.currentWaypointIndex = 0;

    // Store current angles as starting point
    for (int i = 0; i < 6; i++)
    {
      programState.startAngles[i] = currentAngles[i];
    }

    programState.programStartTime = millis();
    programState.isRunning = true;

    unsigned long totalDuration = programState.waypoints[programState.waypointCount - 1].t;
    Serial.printf("Program started: %d waypoints (+1 lead-in), duration: %lu ms (incl. %lu ms lead-in)\n",
                  receivedCount, totalDuration, LEAD_IN_MS);
    return;
  }

  // Any manual joint command stops the program
  if (doc.containsKey("base") || doc.containsKey("armA") || doc.containsKey("armB") ||
      doc.containsKey("wristA") || doc.containsKey("wristB") || doc.containsKey("gripper"))
  {
    stopProgram(); // Interrupt any running program
  }

  // Update any fields present in the payload
  if (doc.containsKey("base"))
  {
    targetAngles[0] = clampAngle(doc["base"].as<float>(), 0);
  }
  if (doc.containsKey("armA"))
  {
    targetAngles[1] = clampAngle(doc["armA"].as<float>(), 1);
  }
  if (doc.containsKey("armB"))
  {
    targetAngles[2] = clampAngle(doc["armB"].as<float>(), 2);
  }
  if (doc.containsKey("wristA"))
  {
    targetAngles[3] = clampAngle(doc["wristA"].as<float>(), 3);
  }
  if (doc.containsKey("wristB"))
  {
    targetAngles[4] = clampAngle(doc["wristB"].as<float>(), 4);
  }
  if (doc.containsKey("gripper"))
  {
    targetAngles[5] = clampAngle(doc["gripper"].as<float>(), 5);
  }
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    Serial.printf("WS client %u connected\n", client->id());
    break;
  case WS_EVT_DISCONNECT:
    Serial.printf("WS client %u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
      handleJsonPayload((const char *)data, len);
    }
    break;
  }
  default:
    break;
  }
}

void connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.printf("Connecting to %s", WIFI_SSID);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);

  servo1.attach(servoPins[0]);
  servo2.attach(servoPins[1]);
  servo3.attach(servoPins[2]);
  servo4.attach(servoPins[3]);
  servo5.attach(servoPins[4]);
  servo6.attach(servoPins[5]);
  servo7.attach(servoPins[6]);

  // Initialize state to home
  for (int i = 0; i < 6; i++)
  {
    currentAngles[i] = HOME_ANGLES[i];
    targetAngles[i] = HOME_ANGLES[i];
  }
  writeServos();

  connectWiFi();

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  server.begin();
  Serial.printf("WebSocket at ws://%s:%u%s\n", WiFi.localIP().toString().c_str(), HTTP_PORT, WS_PATH);
}

void loop()
{
  updateServosSlew();
  ws.cleanupClients();

  // Limit to one client to prevent buffer overflow
  if (ws.count() > 1)
  {
    // Keep only the most recent connection - close the oldest
    auto &clients = ws.getClients();
    if (!clients.empty())
    {
      clients.front().close();
    }
  }
}
