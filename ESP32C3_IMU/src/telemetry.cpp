#include <WiFi.h>
#include "telemetry.h"

static WiFiServer telemetryServer(3333);
static WiFiClient telemetryClient;

void telemetryStartAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("JumperAP", "jumper1234");
  telemetryServer.begin();
  Serial.print("Telemetry AP started, IP=");
  Serial.println(WiFi.softAPIP());
}

void telemetrySend(const char* msg) {
  if (!telemetryClient || !telemetryClient.connected()) {
    telemetryClient = telemetryServer.available();
  }
  if (telemetryClient && telemetryClient.connected()) {
    telemetryClient.println(msg);
  }
}
