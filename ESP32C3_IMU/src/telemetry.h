// Simple WiFi SoftAP telemetry sender
#ifndef TELEMETRY_H
#define TELEMETRY_H

void telemetryStartAP();
void telemetrySend(const char* msg);

#endif // TELEMETRY_H
