#include <Ethernet.h>   // Ethernet
#include "ClearCore.h"  // Motor control

// Ethernet
//byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x33, 0x65};  // First BenchBot, in HFL
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x31, 0xc0};  // Second BenchBot
IPAddress ip(10, 95, 76, 21);  // IP-address 10.95.76.21
unsigned int localPort = 8888; // port to listen for connections
#define MAX_PACKET_LENGTH 100  // max size for incoming packet, characters  
char packetReceived[MAX_PACKET_LENGTH]; // buffer for received packets
EthernetUDP Udp; // EthernetUDP instance to send/receive packets over UDP

// Motors 
#define motor0 ConnectorM0  // for X-axis
#define motor1 ConnectorM1  // for Z-axis
#define HANDLE_ALERTS (1)   // 1 = Enabled, 0 = Disabled
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
int parser_buff_size = 100; // parser buffer size
// BBv3 functions
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
// -------------- Send UDP response when movements are finished -------------
  // AtTargetPosition = 0 when moving, AtTargetPosition = 1 when not moving
  // Check if any of two motors is moving, or both are moving
  if ( ( (motor0.StatusReg().bit.AtTargetPosition == 0) || (motor1.StatusReg().bit.AtTargetPosition == 0) ) && (CarrMovingFlag == 0) ) {
    CarrMovingFlag = 1; // set 1, as X and/or Z is moving
  } 
  // Send UDP msg when there was movement, but now both motors stopped
  if ( (motor0.StatusReg().bit.AtTargetPosition == 1) && (motor1.StatusReg().bit.AtTargetPosition == 1) && (CarrMovingFlag == 1) ) {
    CarrMovingFlag = 0; // set 0, all movements are finished 
    Serial.print("\nAll movements are finished"); 
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    // 3 cases: 1) moved only Z, 2) moved X and Z, 3) moved only X
    if (dist_X == 0) {  // 1st case: movement was ONLY on Z-axis 
      Serial.print("\nMovement was only on Z-axis");
      Udp.write("\nAll movements are finished, Z-axis covered distance = ");
      char Z_axis_char[8]; 
      // convert int to char and send a UDP message
      Udp.write(itoa(dist_Z, Z_axis_char, MAX_DIST_LEN));
    } else {  // 2nd case: movement was on both X- and Z-axes
      if (dist_X != 0 && dist_Z != 0) { 
        Serial.print("\nMovement was on both X-axis and Z-axis");
        Udp.write("\nAll movements have finished, X-axis covered distance = ");
        char X_axis_char[8];
        Udp.write(itoa(dist_X, X_axis_char, MAX_DIST_LEN));
        Udp.write(", Z-axis covered distance = ");
        char Z_axis_char[8]; 
        Udp.write(itoa(dist_Z, Z_axis_char, MAX_DIST_LEN));
      } else {  // 3rd case: moved ONLY X-axis
        Serial.print("\nMovement was only on X-axis");
        Udp.write("\nAll movements have finished, X-axis covered distance = ");
        char X_axis_char[8];
        Udp.write(itoa(dist_X, X_axis_char, MAX_DIST_LEN));
      }
    }
    Udp.endPacket();
  }
// ----------- END Send UDP response when movements are finished ------------

// -------------- Send UDP msg when limit lensors reached -------------------
  // X-axis: when reached Negative Limit sensor, send a message once
  if ((motor0.StatusReg().bit.InNegativeLimit == 1) && (X_NegLimitFlag == 0)) {
    X_NegLimitFlag = 1; // set flag to 1 to show X Neg. limit is reached 
    Serial.print("\nX-axis: Reached Negative Limit sensor");
    send_UDP_msg("\nWARNING! Reached Negative Limit sensor on X-axis");      
  } else if ((motor0.StatusReg().bit.InNegativeLimit == 0) && (X_NegLimitFlag == 1)) {
    X_NegLimitFlag = 0; // set flag to 0 to show X Neg. limit is away
    Serial.print("\nX-axis: moved away from Negative Limit sensor");
  }
  // X-axis: when reached Positive Limit sensor send a message once
  if ((motor0.StatusReg().bit.InPositiveLimit == 1) && (X_PosLimitFlag == 0)) {
    X_PosLimitFlag = 1; // set flag to 1 to show X Pos. limit is reached
    Serial.print("\nX-axis: Reached Positive Limit sensor");
    send_UDP_msg("\nWARNING! Reached Positive Limit Sensor on X-axis");   
  } else if ((motor0.StatusReg().bit.InPositiveLimit == 0) && (X_PosLimitFlag == 1)) {
    X_PosLimitFlag = 0; // set flag to 0 to show X Pos. limit is away
    Serial.print("\nX-axis: moved away from Positive Limit Sensor");
  }
  // Z-axis: when reached Negative Limit sensor send a message once
  if ((motor1.StatusReg().bit.InNegativeLimit == 1) && (Z_NegLimitFlag == 0)) {
    Z_NegLimitFlag = 1; // set flag to 1 to show Z Neg. limit is reached
    Serial.print("\nZ-axis: Reached Negative Limit sensor");
    send_UDP_msg("\nWARNING! Reached Negative Limit sensor on X-axis");
  } else if ((motor1.StatusReg().bit.InNegativeLimit == 0) && (Z_NegLimitFlag == 1)) {
    Z_NegLimitFlag = 0; // set flag to 0 to show Z Neg. limit is away
    Serial.print("\nZ-axis: moved away from Negative Limit Sensor");
  }
  // Z-axis: when reached Positive Limit sensor send a message once
  if ((motor1.StatusReg().bit.InPositiveLimit == 1) && (Z_PosLimitFlag == 0)) {
    Z_PosLimitFlag = 1; // set flag to 1 to show Z Pos. limit is reached
    Serial.print("\nZ-axis: Reached Positive Limit sensor");
    send_UDP_msg("\nWARNING! Reached Positive Limit Sensor on Z-axis");
  } else if ((motor1.StatusReg().bit.InPositiveLimit == 0) && (Z_PosLimitFlag == 1)) {
    Z_PosLimitFlag = 0; // set flag to 0 to show Z Pos. limit is away
    Serial.print("\nZ-axis: moved away from Positive Limit Sensor");
  }
// -------------- END Send UDP msg when Limit Sensors reached ---------------

  // look for a received packet
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    Serial.print("\n\nNEW PACKET! Received packet of size ");
    Serial.print(packetSize); 
    Serial.print(" bytes");
    // print IP address 
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
    Serial.print(Udp.remotePort());
    // read the packet
    int bytesRead = Udp.read(packetReceived, MAX_PACKET_LENGTH);
    Serial.print("\nPacket contents: "); // print contents of packet
    Serial.write(packetReceived, bytesRead); // Serial.write(buf, len)
    // parse received packet
    if ( (0 < bytesRead) && (bytesRead < parser_buff_size) ) {  
      // if packetReceived[0-5] equal to "E-STOP"
      if (memcmp(&packetReceived[0], "E-STOP", 6) == 0) {
        Serial.print("\nReceived UDP message: E-STOP");
        send_UDP_msg("Received UDP message: E-STOP");
      // if packetReceived[0-7] equal to "HOME:X,Z"
      } else if (memcmp(&packetReceived[0], "HOME:X,Z", 8) == 0) {
        Serial.print("\nReceived UDP message: HOME:X,Z");
        send_UDP_msg("Received UDP message: HOME:X,Z");
        Homing_Z_axis(); // first: move camera up to protect it
        Homing_X_axis(); // second: move camera on X-axis
      // if packetReceived[0-5] equal to "HOME:X"
      } else if (memcmp(&packetReceived[0], "HOME:X", 6) == 0) {
        Serial.print("\nReceived UDP message: HOME:X");
        send_UDP_msg("Received UDP message: HOME:X");
        Homing_X_axis(); // call Home:X function
      // if packetReceived[0-5] equal to "HOME:Z"
      } else if (memcmp(&packetReceived[0], "HOME:Z", 6) == 0) {
        Serial.print("\nReceived UDP message: HOME:Z");
        send_UDP_msg("Received UDP message: HOME:Z");
        Homing_Z_axis(); // call Home:Z function
      // if received move message: retrieve X distance
      } else if ((packetReceived[0]=='X')&&(packetReceived[1]==':')) {
        int t = 0; // counter for a position in temp char arr for X dist
        char temp_str_X[MAX_DIST_LEN] = ""; // temp char array for X distance
        // take one char until meet space (between "X:10" and "Z:10")
        while ( (packetReceived[2 + t] != ' ')  &&  ( (packetReceived[2] == '-') || (isdigit(packetReceived[2 + t])) ) ) { 
          // number 2 here is a shift for two chars 'X' and ':'
          temp_str_X[t] = packetReceived[t + 2]; // copy char to temp arr
          t++; // move to one position simultaneously in packet and temp arr 
        }
        temp_str_X[t] = '\0'; // terminate temp-string with '\0'
        dist_X = atol(temp_str_X);//convert char to number, works for neg.numbr 
        t = t + 3; // shift for 3 positions: for 'X', ':' and space ' '
        // now t points to the next position after the space ' '
        if ((packetReceived[t]=='Z') && (packetReceived[t+1]==':')) {
          char temp_str_Z[MAX_DIST_LEN] = ""; // temp char array for Z distance
          t = t + 2; // two steps: from 'Z' to ':', from ':' to first digit
          int i = 0;
          while (packetReceived[t] != '\0') { // go till the end of packet
            if ((packetReceived[t] == '-') || (isdigit(packetReceived[t]))) {
              temp_str_Z[i] = packetReceived[t];// copy char to tmp char arr Z
            }
            t++; // move pointer one position inside received packet
            i++; // move pointer one position inside temp char array Z
          }
          temp_str_Z[i-1] = '\0'; // terminate temp char arr Z with '\0' 
          dist_Z = atol(temp_str_Z);//convert char to number, works minus '-'
        } 
        Serial.print((String)"\nint value of dist_X: " + dist_X + ", int value of dist_Z: " + dist_Z);
        // send back UDP message
        send_UDP_msg("Received X:");
        char dist_X_char[MAX_DIST_LEN]; 
        itoa(dist_X, dist_X_char, MAX_DIST_LEN); // int to char convertion
        send_UDP_msg(itoa(dist_X, dist_X_char, MAX_DIST_LEN));
        send_UDP_msg(", Z:");
        char dist_Z_char[MAX_DIST_LEN]; 
        send_UDP_msg(itoa(dist_Z, dist_Z_char, MAX_DIST_LEN));
        // call motors to move, pass them the distances
        motor_0_MoveDistance(dist_X); // X-axis move function
        motor_1_MoveDistance(dist_Z); // Z-axis move function
      } else { // if received ungecognized packet
        Serial.print("\nReceived message was not recognized");
        send_UDP_msg("ERROR! Received message was not recognized.");
      }
    } 
  } 
  delay(10); // set ClearCore cycle delay, in ms 
} 
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

// allows sending UDP messages in one line
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
  X_HomingDoneFlag = 0; //set Flag to be ready for the next call Homing X  
  dist_X = 0; // set X covered distance to 0
  delay(400); // For correct work of ClearCore. 350 ms works, set 400 ms
  Serial.print("\nX-axis: Homing is done");
  send_UDP_msg("\nX-axis Homing is finished"); // UDP msg: X homing finished
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
  Z_HomingDoneFlag = 0; //set Flag to be ready for the next call Homing Z  
  dist_Z = 0; // set Z covered distance to 0
  delay(400); // For correct work of ClearCore. 350 ms works, set 400 ms
  Serial.print("\nZ-axis: Homing is done");
  send_UDP_msg("\nZ-axis Homing is finished"); // UDP msg: Z homing finished
} 


