#include <ArduinoJson.h> // JSON library (used v7.0.4)
#include <Ethernet.h>    // Ethernet
#include "ClearCore.h"   // Motor control
// Ethernet 
//byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x33, 0x65};  // First BenchBot, in HFL 
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x31, 0xc0};  // Second BenchBot 
IPAddress ip(10, 95, 76, 21);  // set IP-address 10.95.76.21 
unsigned int localPort = 8888; // port to listen for UDP packets 
#define MAX_PACKET_LENGTH 100  // max size for incoming packet, characters 
char packetReceived[MAX_PACKET_LENGTH]; // buffer for received packets
EthernetUDP Udp; // EthernetUDP instance to send/receive packets over UDP
// Motor connectors 
#define motor0 ConnectorM0  // for X-axis
#define motor1 ConnectorM1  // for Z-axis
#define HANDLE_ALERTS (1)   // ClearPath motor error handling: 1 = Enabled, 0 = Disabled
// define ClearPath motors velocity and acceleration limits 
int velocityLimit = 3000; // pulses per sec, 5000 is max for vertical Z-axis
int accelerationLimit = 30000; // pulses per sec^2
// Teknic motor movement functions
bool motor_0_MoveDistance(int distance);
bool motor_1_MoveDistance(int distance);
// Teknic motor error printing and handling functions  
void motor_0_PrintAlerts();
void motor_1_PrintAlerts();
void motor_0_HandleAlerts();
void motor_1_HandleAlerts();

#define MAX_DIST_LEN 10 // max length of distance, characters
int homing_step = 10; // step size for homing functions  
int32_t dist_X = 0;  // initial value of distance for X-axis
int32_t dist_Z = 0;  // initial value of distance for Z-axis
// Flags for Limit sensors, flag = 1 when sensor is reached
byte X_NegLimitFlag = 0; // flag for Negative Limit sensor, X-axis
byte X_PosLimitFlag = 0; // flag for Positive Limit sensor, X-axis
byte Z_NegLimitFlag = 0; // flag for Negative Limit sensor, Z-axis
byte Z_PosLimitFlag = 0; // flag for Positive Limit sensor, Z-axis
// Flags for capturing movements on axes
byte X_MovingFlag = 0; // X-axis: 1 while moving, 0 = no movement
byte Z_MovingFlag = 0; // Z-axis: 1 while moving, 0 = no movement
byte CarrMovingFlag = 0; // 1 when X- or Z-axis is moving (or both)
// Flags for homing functions
byte X_Homing_Flag = 0; // 1 starting the moment when sensor is reached 
byte X_HomingDoneFlag = 0; // 1 when homing on X-axis is done
byte Z_Homing_Flag = 0; // 1 starting the moment when sensor is reached 
byte Z_HomingDoneFlag = 0; // 1 when homing on Z-axis is done
// BenchBot functions
void send_UDP_msg(char msg[]); // function for sending UDP messages
void Homing_X_axis(); // homing for X-axis 
void Homing_Z_axis(); // homing for Z-axis

void setup() { 
  Serial.begin(9600); // 9600 Kpbs, speed for serial port 
  uint32_t timeout = 5000; // 5 seconds for serial port to open
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout) {
    continue;
  }
  Ethernet.begin(mac, ip); // start connection with mac and IP 
  // Make sure the physical link is up before continuing
  while (Ethernet.linkStatus() == LinkOFF) {
    Serial.print("\nThe Ethernet cable is unplugged...");
    delay(1000); 
  }
  Udp.begin(localPort); // begin listening for UDP datagrams

  // ----------------- Motor Control block -----------------
  // set motor input clocking rate, NORMAL rate is ideal for 
  // ClearPath step and direction applications
  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  // set all motor connectors into step and direction mode.
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
  // set the motor's HLFB mode to bipolar PWM
  motor0.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  motor1.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  // set the HFLB carrier frequency to 482 Hz
  motor0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  motor1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
  // sets the maximum velocity for each motor
  motor0.VelMax(velocityLimit);
  motor1.VelMax(velocityLimit);
  // set the maximum acceleration for each motor
  motor0.AccelMax(accelerationLimit);
  motor1.AccelMax(accelerationLimit);
  // enable motors
  motor0.EnableRequest(true);
  Serial.print("\nMotor 0 Enabled");
  motor1.EnableRequest(true);
  Serial.print("\nMotor 1 Enabled");
  // waits for HLFB to assert (waits for motors to finish enabling)
  uint32_t lastStatusTime = millis();
  Serial.print("\nWaiting for HLFB...");
  while (motor0.HlfbState() != MotorDriver::HLFB_ASSERTED &&
    motor1.HlfbState() != MotorDriver::HLFB_ASSERTED &&
    !motor0.StatusReg().bit.AlertsPresent && 
    !motor1.StatusReg().bit.AlertsPresent) {
      // periodically print out why the program is waiting
      if (millis() - lastStatusTime > 100) {
        Serial.print("\nWaiting for HLFB to assert on both motors");
        lastStatusTime = millis();
      } 
  }
  // check if Motor 0 alert occurred during enabling
  if (motor0.StatusReg().bit.AlertsPresent) {
    Serial.print("\nMotor 0: alert detected.");	
    motor_0_PrintAlerts();
    if (HANDLE_ALERTS) { // clear alert if configured 
      motor_0_HandleAlerts();
    } else {
      Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.print("\nMotor 0: Enabling may not have completed as expected. Proceed with caution."); 
  } else {
    Serial.print("\nMotor 0 is Ready");	
  }
  // Check if Motor 1 alert occurred during enabling
  if (motor1.StatusReg().bit.AlertsPresent) {
    Serial.print("\nMotor 1: alert detected.");		
    motor_1_PrintAlerts();
    if (HANDLE_ALERTS) { // clear alert if configured 
      motor_1_HandleAlerts();
    } else {
      Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.print("\nMotor 1: Enabling may not have completed as expected. Proceed with caution.");		
  } else {
    Serial.print("\nMotor 1 is Ready");	
  }
  // ------------- END of Motor Control block --------------

  // Set Negative and Positive Limit sensors for X- and Z-axes
  if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0)) {
    Serial.print("\nI/O-0 is now set to Negative Sensor for ConnectorM0 and enabled"); // M-0's negative limit switch is now set to IO-0 and enabled
  } if (ConnectorM0.LimitSwitchPos(CLEARCORE_PIN_IO1)) {
    Serial.print("\nI/O-1 is now set to Positive Sensor for ConnectorM0 and enabled"); // M-0's positive limit switch is now set to IO-1 and enabled
  } if (ConnectorM1.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
    Serial.print("\nI/O-2 is now set to Negative Sensor for ConnectorM1 and enabled"); // M-1's negative limit switch is now set to IO-2 and enabled
  } if (ConnectorM1.LimitSwitchPos(CLEARCORE_PIN_IO3)) {
    Serial.print("\nI/O-3 is now set to Positive Sensor for ConnectorM1 and enabled"); // M-1's positive limit switch is now set to IO-3 and enabled
  }
  delay(300); // time for ClearCore to set the limit switches (250+ ms)
} // end of setup()

void loop() { 
  // motor_0 is instance of MotorDriver, motor0 is synonym to "ConnectorM0"
  MotorDriver *motor_0 = &ConnectorM0; // set motor for X-axis
  MotorDriver *motor_1 = &ConnectorM1; // set motor for Z-axis
// -------------- Sending UDP response when movements are finished -------------
  // this block has only one primary reason: to send UDP 
  // message when all the movements are finished
  // AtTargetPosition = 0 when moving, AtTargetPosition = 1 when not moving
  // Check if any of two motors is moving, or both are moving
  if ( ( (motor0.StatusReg().bit.AtTargetPosition == 0) || (motor1.StatusReg().bit.AtTargetPosition == 0) ) && (CarrMovingFlag == 0) ) {
    CarrMovingFlag = 1; // set flag to 1, as X and/or Z is moving
  } 
  // Send UDP msg if there was movement, but now both motors stopped
  if ( (motor0.StatusReg().bit.AtTargetPosition == 1) && (motor1.StatusReg().bit.AtTargetPosition == 1) && (CarrMovingFlag == 1) ) {
    CarrMovingFlag = 0; // set flag to 0 as all movements are finished 
    Serial.print("\nAll movements are finished"); 
    // 3 cases: 1) moved only Z, 2) moved X and Z, 3) moved only X
    if (dist_X == 0) {  // 1st case: movement was ONLY on Z-axis 
      Serial.print("\nMovement was only on Z-axis, covered dist = ");
      Serial.print(dist_Z);
      send_UDP_msg("{\"movement\":\"finished\",\"axis\":\"z\"}"); 
    } else {  // 2nd case: movement was on both X- and Z-axes
      if (dist_X != 0 && dist_Z != 0) { 
        Serial.print("\nMovement was on both X- and Z-axes. X-covered dist = "); 
        Serial.print(dist_X); 
        Serial.print(" , Z-covered dist = "); 
        Serial.print(dist_Z); 
        send_UDP_msg("{\"movement\":\"finished\",\"axis\":\"x\"}");
        send_UDP_msg("{\"movement\":\"finished\",\"axis\":\"z\"}");
      } else {  // 3rd case: movement was ONLY on X-axis (X!=0 && Z==0)
        Serial.print("\nMovement was only on X-axis, covered dist = ");
        Serial.print(dist_X);
        send_UDP_msg("{\"movement\":\"finished\",\"axis\":\"x\"}");
      }
    }
  }
// ----------- END Sending UDP response when movements are finished ------------

// -------------- Sending UDP msg when limit lensors reached -------------------
  // X-axis: when reached Negative Limit sensor, send a message once
  if ((motor0.StatusReg().bit.InNegativeLimit == 1) && (X_NegLimitFlag == 0)) {
    X_NegLimitFlag = 1; // set flag to 1 to show X Neg. limit is reached 
    Serial.print("\nX-axis: Reached Negative Limit sensor");
    send_UDP_msg("{\"status\":\"warning\",\"reachedSensor\":\"xNegative\"}");
  } else if ((motor0.StatusReg().bit.InNegativeLimit == 0) && (X_NegLimitFlag == 1)) {
    X_NegLimitFlag = 0; // set flag to 0 to show X Neg. limit is away
    Serial.print("\nX-axis: moved away from Negative Limit sensor");
  }
  // X-axis: when reached Positive Limit sensor send a message once
  if ((motor0.StatusReg().bit.InPositiveLimit == 1) && (X_PosLimitFlag == 0)) {
    X_PosLimitFlag = 1; // set flag to 1 to show X Pos. limit is reached
    Serial.print("\nX-axis: Reached Positive Limit sensor");
    send_UDP_msg("{\"status\":\"warning\",\"reachedSensor\":\"xPositive\"}");
  } else if ((motor0.StatusReg().bit.InPositiveLimit == 0) && (X_PosLimitFlag == 1)) {
    X_PosLimitFlag = 0; // set flag to 0 to show X Pos. limit is away
    Serial.print("\nX-axis: moved away from Positive Limit Sensor");
  }
  // Z-axis: when reached Negative Limit sensor send a message once
  if ((motor1.StatusReg().bit.InNegativeLimit == 1) && (Z_NegLimitFlag == 0)) {
    Z_NegLimitFlag = 1; // set flag to 1 to show Z Neg. limit is reached
    Serial.print("\nZ-axis: Reached Negative Limit sensor");
    send_UDP_msg("{\"status\":\"warning\",\"reachedSensor\":\"zNegative\"}");
  } else if ((motor1.StatusReg().bit.InNegativeLimit == 0) && (Z_NegLimitFlag == 1)) {
    Z_NegLimitFlag = 0; // set flag to 0 to show Z Neg. limit is away
    Serial.print("\nZ-axis: moved away from Negative Limit Sensor");
  }
  // Z-axis: when reached Positive Limit sensor send a message once
  if ((motor1.StatusReg().bit.InPositiveLimit == 1) && (Z_PosLimitFlag == 0)) {
    Z_PosLimitFlag = 1; // set flag to 1 to show Z Pos. limit is reached
    Serial.print("\nZ-axis: Reached Positive Limit sensor");
    send_UDP_msg("{\"status\":\"warning\",\"reachedSensor\":\"zPositive\"}");
  } else if ((motor1.StatusReg().bit.InPositiveLimit == 0) && (Z_PosLimitFlag == 1)) {
    Z_PosLimitFlag = 0; // set flag to 0 to show Z Pos. limit is away
    Serial.print("\nZ-axis: moved away from Positive Limit Sensor");
  }
// -------------- END Sending UDP msg when Limit Sensors reached ---------------
  // look for a received packet
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    Serial.print("\nNEW PACKET! Received packet of size ");
    Serial.print(packetSize); 
    Serial.print(" bytes");
    // print IP address of received packet
    Serial.print("\nRemote IP address: ");
    IPAddress remote = Udp.remoteIP();
    for (int i = 0; i < 4; i++) { // loop to print IP address 
      if (i < 3) {
        Serial.print(remote[i], DEC);
        Serial.print(".");
      } else {
        Serial.print(remote[i], DEC);
      }
    }
    Serial.print("  Remote port: ");
    Serial.print(Udp.remotePort()); // print port number
    // read the packet
    int bytesRead = Udp.read(packetReceived, MAX_PACKET_LENGTH);
    Serial.print("\nPacket contents: "); // print contents of packet
    Serial.write(packetReceived, bytesRead); // Serial.write(buf_name, length)
    // JSON processing
    JsonDocument doc_output; // variable to store JSON deserialized document
    // deserialize JSON (output, input)
    DeserializationError des_error = deserializeJson(doc_output, packetReceived); 
    if (des_error) {  // JSON Deserialization error handling
      Serial.print(F("JSON deserializeJson() error: "));
      Serial.print(des_error.c_str()); // print extra info about error from ArduinoJson lib. 
    }
    // if JSON x or z contain char command, do the command, if NOT, derive int distance(s)
    const char *content_x = doc_output["x"]; // get X value as char arr
    Serial.print("\n char value of x: ");
    Serial.print(content_x);
    const char *content_z = doc_output["z"]; // get Z value as char arr
    Serial.print("\n char value of z: ");
    Serial.print(content_z);

    // when JSON "x" contains text then int dist_X is empty (zero) and viceversa
    dist_X = doc_output["x"]; // get x value as int distance
    Serial.print("\n int value of x: ");
    Serial.print(dist_X);
    dist_Z = doc_output["z"]; // get z value as int distance
    Serial.print("\n int value of z: ");
    Serial.print(dist_Z);
    byte m = 0; // flag
    if (memcmp(&content_z[0], "home", 4) == 0) {
      Serial.print("\n Z-axis homing"); 
      JsonDocument doc; // create new JsonDocument 
      doc["status"] = "received"; // add member to JSON document
      doc["z"] = "home"; // add member to JSON document
      char output[64]; // create C-string for storing new JSON doc
      serializeJson(doc, output); // conversion to JSON
      send_UDP_msg(output); // send constructed C-string via UDP
      m = 1;
      Homing_Z_axis(); // call homing Z axis function
    } else if (dist_Z != 0) { // when received non-zero distance Z
      Serial.print("\n Z-axis moving, distance = "); 
      Serial.print(dist_Z); 
      JsonDocument doc; // create new JsonDocument for sending UDP msg
      doc["status"] = "received"; // add member to JSON document
      doc["z"] = dist_Z; // add member to JSON document
      char output[64]; // create C-string for storing new JSON doc
      serializeJson(doc, output); // conversion to JSON
      send_UDP_msg(output); // send constructed C-string via UDP 
      m = 1;
      motor_1_MoveDistance(dist_Z); // call Z-axis move function
    } 
    if (memcmp(&content_x[0], "home", 4) == 0) {
      Serial.print("\n X-axis homing"); 
      JsonDocument doc; // create new JsonDocument 
      doc["status"] = "received"; // add member to JSON document
      doc["x"] = "home"; // add member to JSON document
      char output[64]; // create C-string for storing new JSON doc
      serializeJson(doc, output); // conversion to JSON
      send_UDP_msg(output); // send JSON responce
      m = 1;
      Homing_X_axis(); // call homing Z axis function
    } else if (dist_X != 0) { // when received non-zero distance X 
      Serial.print("\n X-axis moving, distance = "); 
      Serial.print(dist_X); 
      JsonDocument doc; // create new JsonDocument for sending UDP msg 
      doc["status"] = "received"; // add member to JSON document
      doc["x"] = dist_X; // add member to JSON document
      char output[64]; // create C-string for storing new JSON doc
      serializeJson(doc, output); // conversion to JSON
      send_UDP_msg(output); // send via UDP constructed C-string
      m = 1;
      motor_0_MoveDistance(dist_X); // call X-axis move function
    } 
    if (m == 0) { // if received message is ungecognized
      Serial.print("\nReceived incorrect UDP message");
      send_UDP_msg("{\"error\":\"received incorrect UDP message\"}");
    }
    m = 0;
  } 
  delay(10); // set ClearCore cycle delay, ms 
} // end of loop() 
//--------------------------- Motor 0 Beginning ------------------------------- 
bool motor_0_MoveDistance(int distance) { // Moving X-axis motor
// check if a motor0 alert is currently preventing motion
  if (motor0.StatusReg().bit.AlertsPresent) {
    Serial.print("\nMotor 0 alert detected.");		
    motor_0_PrintAlerts();
    if (HANDLE_ALERTS) { // clear alert if configured 
      motor_0_HandleAlerts();
    } else {
      Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.print("\nMotor 0: Move canceled.");		
    return false;
  }
  Serial.print("\nMotor 0: Moving distance: ");
  Serial.print(distance);
  motor0.Move(distance); // Command the move of incremental distance
  // wait for HLFB to assert (signaling the move has successfully completed)
  Serial.print("\nMotor 0: Moving.. Waiting for HLFB");
  uint32_t lastStatusTime = millis();
  while ( (!motor0.StepsComplete() || motor0.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor0.StatusReg().bit.AlertsPresent) {
    // periodically print out why the application is waiting
    if (millis() - lastStatusTime > 100) {
      Serial.print("\nMotor 0: Waiting for HLFB to assert");
      lastStatusTime = millis();
    }
    // check if motor alert occurred during move
    if (motor0.StatusReg().bit.AlertsPresent) {
      motor0.MoveStopAbrupt();
      Serial.print("\nMotor 0: alert detected.");		
      motor_0_PrintAlerts();
      if (HANDLE_ALERTS) { // clear alert if configured 
        motor_0_HandleAlerts();
      } else {
        Serial.print("\nMotor 0: Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
      }
      Serial.print("\nMotor 0: Motion may not have completed as expected. Proceed with caution.");
      return false;
    } else {
      Serial.print("\nMotor 0: Move sent to motor");
      return true;
    }
  }
}
// report status of alerts on motor0
void motor_0_PrintAlerts() {
  Serial.print("\nMotor 0: Alerts present: ");
  if (motor0.AlertReg().bit.MotionCanceledInAlert) {
    Serial.print("    MotionCanceledInAlert "); }
  if (motor0.AlertReg().bit.MotionCanceledPositiveLimit) {
    Serial.print("    MotionCanceledPositiveLimit "); }
  if (motor0.AlertReg().bit.MotionCanceledNegativeLimit) {
    Serial.print("    MotionCanceledNegativeLimit "); }
  if (motor0.AlertReg().bit.MotionCanceledSensorEStop) {
    Serial.print("    MotionCanceledSensorEStop "); }
  if (motor0.AlertReg().bit.MotionCanceledMotorDisabled) {
    Serial.print("    MotionCanceledMotorDisabled "); }
  if (motor0.AlertReg().bit.MotorFaulted) {
    Serial.print("    MotorFaulted "); }
}
// handle motor0 alerts
void motor_0_HandleAlerts() {
  if(motor0.AlertReg().bit.MotorFaulted) {
    // if a motor fault is present, clear it by cycling enable
    Serial.print("\nMotor 0: Faults present. Cycling enable signal to motor to clear faults.");
    motor0.EnableRequest(false);
    Delay_ms(10);
    motor0.EnableRequest(true);
  }
  Serial.print("\nMotor 0: Clearing alerts.");
  motor0.ClearAlerts(); // clear alerts
}
//--------------------------- Motor 0 End --------------------------------------

//--------------------------- Motor 1 Beginning ------------------------------- 
bool motor_1_MoveDistance(int distance) { // Moving Z-axis motor
// check if a motor1 alert is currently preventing motion
  if (motor1.StatusReg().bit.AlertsPresent) {
    Serial.print("\nMotor 1 alert detected.");		
    motor_1_PrintAlerts();
    if (HANDLE_ALERTS) { // clear alert if configured 
      motor_1_HandleAlerts();
    } else {
      Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.print("\nMotor 1: Move canceled.");		
    return false;
  }
  Serial.print("\nMotor 1: Moving distance: ");
  Serial.print(distance);
  motor1.Move(distance); // Command the move of incremental distance
  // wait for HLFB to assert (signaling the move has successfully completed)
  Serial.print("\nMotor 1: Moving.. Waiting for HLFB");
  uint32_t lastStatusTime = millis();
  while ( (!motor1.StepsComplete() || motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor1.StatusReg().bit.AlertsPresent) {
    // periodically print out why the application is waiting
    if (millis() - lastStatusTime > 100) {
      Serial.print("\nMotor 1: Waiting for HLFB to assert");
      lastStatusTime = millis();
    }
    // check if motor alert occurred during move
    if (motor1.StatusReg().bit.AlertsPresent) {
      motor1.MoveStopAbrupt();
      Serial.print("\nMotor 1: alert detected.");		
      motor_1_PrintAlerts();
      if (HANDLE_ALERTS) { // clear alert if configured 
        motor_1_HandleAlerts();
      } else {
        Serial.print("\nMotor 1: Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
      }
      Serial.print("\nMotor 1: Motion may not have completed as expected. Proceed with caution.");
      return false;
    } else {
      Serial.print("\nMotor 1: Move sent to motor");
      return true;
    }
  } 
}
// report status of alerts on motor1
void motor_1_PrintAlerts() {
  Serial.print("\nMotor 1: Alerts present: ");
  if (motor1.AlertReg().bit.MotionCanceledInAlert) {
    Serial.print("    MotionCanceledInAlert "); }
  if (motor1.AlertReg().bit.MotionCanceledPositiveLimit) {
    Serial.print("    MotionCanceledPositiveLimit "); }
  if (motor1.AlertReg().bit.MotionCanceledNegativeLimit) {
    Serial.print("    MotionCanceledNegativeLimit "); }
  if (motor1.AlertReg().bit.MotionCanceledSensorEStop) {
    Serial.print("    MotionCanceledSensorEStop "); }
  if (motor1.AlertReg().bit.MotionCanceledMotorDisabled) {
    Serial.print("    MotionCanceledMotorDisabled "); }
  if (motor1.AlertReg().bit.MotorFaulted) {
    Serial.print("    MotorFaulted "); }
}
// handle motor1 alerts
void motor_1_HandleAlerts() {
  if (motor1.AlertReg().bit.MotorFaulted) {
    // if a motor fault is present, clear it by cycling enable
    Serial.print("\nMotor 1: Faults present. Cycling enable signal to motor to clear faults.");
    motor1.EnableRequest(false);
    Delay_ms(10);
    motor1.EnableRequest(true);
  }
  Serial.print("\nMotor 1: Clearing alerts.");
  motor1.ClearAlerts(); // clear alerts
}
//--------------------------- Motor 1 End --------------------------------------

// function for sending UDP messages in one line
void send_UDP_msg(char msg[]) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(msg);
    Udp.endPacket();
}

void Homing_X_axis() {
  while (X_HomingDoneFlag != 1 ) {
    // when reached Negative Limit sensor
    if (motor0.StatusReg().bit.InNegativeLimit == 1) {  
      if (X_Homing_Flag == 0) { 
        X_Homing_Flag = 1;// change direction to away from sensor
      } else if (X_Homing_Flag == 1) {  
        // start or continue to go away from sensor
        motor_0_MoveDistance(homing_step); // positive = RIGHT
      }
    } else if (X_Homing_Flag == 1) { // when away from Negative Limit sensor
      X_Homing_Flag = 0; // homing finisned
      X_HomingDoneFlag = 1; // set exit condition for "while" loop
    } else { // NO_Flag, we need to move negative direction
      motor_0_MoveDistance(-homing_step); //go one step away from sensor(LEFT)
    }
    delay(1); // need this delay to keep normal distance Sensor - Home pos.
  } 
  X_HomingDoneFlag = 0; //set Flag to 0 to be ready for next call Homing X  
  dist_X = 0; // when homing is done set X covered distance to 0
  delay(400); // for correct work of ClearCore. 350 ms works, set 400 ms
  Serial.print("\nX-axis: Homing is done");
  send_UDP_msg("{\"status\":\"finished\",\"x\":\"home\"}"); // send msg: Z homing finished
} 

void Homing_Z_axis() {
  while (Z_HomingDoneFlag != 1 ) {
    // when reached Positive Limit sensor
    if (motor1.StatusReg().bit.InPositiveLimit == 1) {  
      if (Z_Homing_Flag == 0) {      
        Z_Homing_Flag = 1; // change direction to away from sensor
      } else if (Z_Homing_Flag == 1) { 
        motor_1_MoveDistance(-homing_step); //go one step away from sensor(DOWN)
      }
    } else if (Z_Homing_Flag == 1) { // when away from Positive Limit sensor
      Z_Homing_Flag = 0; // homing finisned
      Z_HomingDoneFlag = 1; // set exit condition for "while" loop  
    } else { 
      motor_1_MoveDistance(homing_step); // go one step positive (UP)
    }
    delay(1); // need this delay to keep normal distance Sensor - Home pos.
  } 
  Z_HomingDoneFlag = 0; //set Flag to 0 to be ready for next call Homing Z  
  dist_Z = 0; // when homing is done set Z covered distance to 0 
  delay(400); // for ClearCore's correct work. 350 ms works, set 400 ms
  Serial.print("\nZ-axis: Homing is done");
  send_UDP_msg("{\"status\":\"finished\",\"z\":\"home\"}"); // send msg: Z homing finished
} 


