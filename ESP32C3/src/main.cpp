#include <Arduino.h>

#include <iq_module_communication.hpp>

IqSerial ser(Serial0);
PowerMonitorClient power(0);
PropellerMotorControlClient prop_control(0);
BrushlessDriveClient brushless_drive(0);
MultiTurnAngleControlClient multi_control(0);

void setup() {
  // put your setup code here, to run once:
  ser.begin();
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  Serial.println("Connected");
  //ser.set(prop_control.timeout_, 3.0f);
  //ser.save(prop_control.timeout_);
}



/* 

* start at 45 deg
* timer starts when button pressed (simulated imu launch detection)
* calculate time needed, not implemented yet - precalculate based on some information 
or just set an example time


1. Get launch time        launch_time   - from button press -> countdown
2. Start timer            time_now      - test timing externally
3. At brake_time, brake   brake_time    - works, but need to check timer


*/


void loop() {
  int sensorVal = digitalRead(2);
  //ser.set(multi_control.ctrl_velocity_, 20.0f);
  float volts_now;
   //Check to see if we get a reply. If we do, print the result
  if(ser.get(power.volts_, volts_now)){
    Serial.print("Volts:");
    Serial.println(volts_now);
  }
  float velocity_now;
  if(ser.get(brushless_drive.obs_velocity_, velocity_now)){
    Serial.print("Velocity:");
    Serial.println(velocity_now);
  }

  if(sensorVal == HIGH) {
    ser.set(multi_control.ctrl_brake_);
  } else {
    ser.set(multi_control.ctrl_velocity_, 60.0f);
  }
  delay(50);
}


/*
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}*/