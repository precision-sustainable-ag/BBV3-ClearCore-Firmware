#include <Ethernet.h>  // needed for Ethernet
#include "ClearCore.h" // needed for Motor control

//byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x33, 0x65};  // First BenchBot, in HFL
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x31, 0xc0};  // Second BenchBot
IPAddress ip(10, 95, 76, 21);  // IP-address 10.95.76.21
unsigned int localPort = 8888; // The local port to listen for connections

#define MAX_PACKET_LENGTH 100 // max number of characters for incoming packet
char packetReceived[MAX_PACKET_LENGTH]; // Buffer for received packets

EthernetUDP Udp; // EthernetUDP instance to send/receive packets over UDP

// ----------------- Motor Control block -----------------
// Specifies motors to move: ConnectorM0 ..... ConnectorM3
#define motor0 ConnectorM0  // for X-axis
#define motor1 ConnectorM1  // for Z-axis

// The baud rate bps for serial-over-USB cable connection
#define baudRate 9600

#define HANDLE_ALERTS (1) // 1 = Enabled, 0 = Disabled

// Define the velocity and acceleration limits for ClearPath motors
int velocityLimit = 3000; // pulses per sec, 5000 is max for vertical Z-axis
int accelerationLimit = 30000; // pulses per sec^2

int homing_step = 10; // step size for homing functions  
int32_t dist_X = 0;  // initial values of move distance for X
int32_t dist_Z = 0;  // initial values of move distance for Z

// Flags for Limit sensors, flag = 1 when sensor is reached
int X_NegLimitFlag = 0;  // Negative Limit sensor for X-axis
int X_PosLimitFlag = 0;  // Positive Limit sensor for X-axis
int Z_NegLimitFlag = 0;  // Negative Limit sensor for Z-axis
int Z_PosLimitFlag = 0;  // Positive Limit sensor for Z-axis

// Flags for capturing movements on axes
int X_MovingFlag = 0; // 1 when movement is on X-axis, 0 = no movement
int Z_MovingFlag = 0; // 1 when movement is on Z-axis, 0 = no movement
int CarrMovingFlag = 0; // 1 when X- or Z-axis is moving (or both)

// Flags for homing functions
int X_Homing_Flag = 0; // 1 starting the moment when sensor is reached 
int X_HomingDoneFlag = 0; // 1 when homing on X-axis is done
int Z_Homing_Flag = 0; // 1 starting the moment when sensor is reached 
int Z_HomingDoneFlag = 0; // 1 when homing on Z-axis is done

int parser_buff_size = 100; // parser buffer size

// movement functions
bool motor_0_MoveDistance(int distance);
bool motor_1_MoveDistance(int distance);
// motor error handling functions  
void motor_0_PrintAlerts();
void motor_1_PrintAlerts();
void motor_0_HandleAlerts();
void motor_1_HandleAlerts();
// ------------- end of Motor Control block --------------

// homing functions
int Homing_X_axis(); 
int Homing_Z_axis();

int send_UDP_msg(char msg[]); // sending UDP messages

// this loop executes once, when ClearCore turns on
void setup() { 
    Serial.begin(9600);
    uint32_t timeout = 5000; // 5 seconds for serial port to open
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }
    Ethernet.begin(mac, ip);
    // Make sure the physical link is up before continuing
    while (Ethernet.linkStatus() == LinkOFF) {
        Serial.print("\nThe Ethernet cable is unplugged...");
        delay(1000);
    }
    Udp.begin(localPort); // Begin listening for UDP datagrams

    // ----------------- Motor Control block -----------------
    // Sets the input clocking rate. This normal rate is ideal for ClearPath
    // step and direction applications.
    MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
    // Sets all motor connectors into step and direction mode.
    MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL,
                          Connector::CPM_MODE_STEP_AND_DIR);
    // Set the motor's HLFB mode to bipolar PWM
    motor0.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor1.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    // Set the HFLB carrier frequency to 482 Hz
    motor0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    // Sets the maximum velocity for each motor
    motor0.VelMax(velocityLimit);
    motor1.VelMax(velocityLimit);
    // Set the maximum acceleration for each motor
    motor0.AccelMax(accelerationLimit);
    motor1.AccelMax(accelerationLimit);
    // Enable motors
    motor0.EnableRequest(true);
    Serial.print("\nMotor 0 Enabled");
    motor1.EnableRequest(true);
    Serial.print("\nMotor 1 Enabled");
    // Waits for HLFB to assert 
    // this means: waits for both motors to finish enabling
    uint32_t lastStatusTime = millis();
    Serial.print("\nWaiting for HLFB...");
    while (motor0.HlfbState() != MotorDriver::HLFB_ASSERTED &&
        motor1.HlfbState() != MotorDriver::HLFB_ASSERTED &&
        !motor0.StatusReg().bit.AlertsPresent && 
        !motor1.StatusReg().bit.AlertsPresent) {
        // Periodically prints out why the application is waiting
        if (millis() - lastStatusTime > 100) {
            Serial.print("\nWaiting for HLFB to assert on both motors");
            lastStatusTime = millis();
        }    
    }
    // Check if motor alert occurred during enabling
    // Clear alert if configured to do so 
    // Motor 0: if there is an alert, then motor_0_HandleAlerts();
    if (motor0.StatusReg().bit.AlertsPresent) {
        Serial.print("\nMotor 0: alert detected.");	
        motor_0_PrintAlerts();
        if (HANDLE_ALERTS) {
            motor_0_HandleAlerts();
        } else {
            Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.print("\nMotor 0: Enabling may not have completed as expected. Proceed with caution."); 
    } else {
        Serial.print("\nMotor 0 is Ready");	
    }

    // Motor 1: if there is an alert, then motor_1_HandleAlerts();
    if (motor1.StatusReg().bit.AlertsPresent) {
        Serial.print("\nMotor 1: alert detected.");		
        motor_1_PrintAlerts();
        if (HANDLE_ALERTS) {
            motor_1_HandleAlerts();
        } else {
            Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.print("\nMotor 1: Enabling may not have completed as expected. Proceed with caution.");		
    } else {
        Serial.print("\nMotor 1 is Ready");	
    }
    // ----------------- Motor Control block -----------------

    // Setting Negative and Positive Limit Sensors for X- and Z-axes
    if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0)) {
        // M-0's negative limit switch is now set to IO-0 and enabled.
        Serial.print("\nI/O-0 is now set to Negative Sensor for ConnectorM0 and enabled");	
    }
    if (ConnectorM0.LimitSwitchPos(CLEARCORE_PIN_IO1)) {
        // M-0's positive limit switch is now set to IO-1 and enabled.
        Serial.print("\nI/O-1 is now set to Positive Sensor for ConnectorM0 and enabled");
    }
    if (ConnectorM1.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
        // M-1's negative limit switch is now set to IO-2 and enabled.
        Serial.print("\nI/O-2 is now set to Negative Sensor for ConnectorM1 and enabled");	
    }
    if (ConnectorM1.LimitSwitchPos(CLEARCORE_PIN_IO3)) {
        // M-1's positive limit switch is now set to IO-3 and enabled.
        Serial.print("\nI/O-3 is now set to Positive Sensor for ConnectorM1 and enabled");
    }
    delay(300); // give ClearCore time to set limit switches (250+ ms)
}  // END of setup() loop

// this loop is executed all the time after the setup() loop
void loop() { 
    // motor_0 is instance of MotorDriver, motor0 is just "ConnectorM0"
    MotorDriver *motor_0 = &ConnectorM0; // motor for X-axis
    MotorDriver *motor_1 = &ConnectorM1; // motor for Z-axis

// ------------- Send UDP response when movements are finished ------------
    // AtTargetPosition = 0 when moving, AtTargetPosition = 1 when not moving
    // Track if any of two motors is moving, or both are moving
    if ( ( (motor0.StatusReg().bit.AtTargetPosition == 0) || (motor1.StatusReg().bit.AtTargetPosition == 0) ) && (CarrMovingFlag == 0) ) {
        CarrMovingFlag = 1; // if X or Z is moving, set Flag to 1
        Serial.print("\nSome movement is being done");
    } 
    // Send UDP message when there was movement, but now both motors stopped
    if ( (motor0.StatusReg().bit.AtTargetPosition == 1) && (motor1.StatusReg().bit.AtTargetPosition == 1) && (CarrMovingFlag == 1) ) {
        CarrMovingFlag = 0; // then all movements are finished 
        Serial.print("\nAll movements are finished"); 
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        // 3 cases: 1) moved only Z, 2) moved X and Z, 3) moved only X
        if (dist_X == 0) {  // 1st case: movement was ONLY on Z-axis 
            Serial.print("\nMovement was only on Z-axis");
            Udp.write("\nAll movements are finished, Z-axis covered distance = ");
            char Z_axis_char[8];// these two line is just int-to-char convertion
            Udp.write(itoa(dist_Z, Z_axis_char, 10) );// and sending a UDP msg
        } 
        else {  // 2nd case: movement was on both X- and Z-axes
            if (dist_X != 0 && dist_Z != 0) { 
                Serial.print("\nMovement was on both X-axis and Z-axis");
                Udp.write("\nAll movements have finished, X-axis covered distance = ");
                char X_axis_char[8];
                Udp.write(itoa(dist_X, X_axis_char, 10) );
                Udp.write(", Z-axis covered distance = ");
                char Z_axis_char[8]; 
                Udp.write(itoa(dist_Z, Z_axis_char, 10) ); 
            } else {  // 3rd case: moved ONLY X-axis
                Serial.print("\nMovement was only on X-axis");
                Udp.write("\nAll movements have finished, X-axis covered distance = ");
                char X_axis_char[8];
                Udp.write(itoa(dist_X, X_axis_char, 10) );
            }
        }
        Udp.endPacket();
    }
// ---------- END Send UDP response when movements are finished -----------

// -------------- Send UDP msg when limit lensors reached -------------------
    // X-axis: when reach Negative Limit sensor, send a message once
    if ((motor0.StatusReg().bit.InNegativeLimit == 1) && (X_NegLimitFlag == 0)) {
        X_NegLimitFlag = 1; // set flag to show X Neg. limit is reached 
        Serial.print("\nX-axis: Hit Negative Limit sensor");
        send_UDP_msg("\nWARNING! Hit Negative Limit sensor on X-axis");      
    } // set Flag back to 0 when NegativeLimit sensor is not active any more
    if ((motor0.StatusReg().bit.InNegativeLimit == 0) && (X_NegLimitFlag == 1)) {
        X_NegLimitFlag = 0; // set flag to show X Neg. limit is away
        Serial.print("\nX-axis: moved away from Negative Limit sensor");
    }
    // X-axis: when reach Positive Limit sensor send a message once
    if ((motor0.StatusReg().bit.InPositiveLimit == 1) && (X_PosLimitFlag == 0)) {
        X_PosLimitFlag = 1; // set flag to show X Pos. limit is reached
        Serial.print("\nX-axis: Hit Positive Limit sensor");
        send_UDP_msg("\nWARNING! Hit Positive Limit Sensor on X-axis");   
    } // set Flag back to 0 when Positive Limit sensor is not active any more
    if ((motor0.StatusReg().bit.InPositiveLimit == 0) && (X_PosLimitFlag == 1)) {
        X_PosLimitFlag = 0; // set flag to show X Pos. limit is away
        Serial.print("\nX-axis: moved away from Positive Limit Sensor");
    }

    // Z-axis: when reach Negative Limit sensor send a message once
    if ((motor1.StatusReg().bit.InNegativeLimit == 1) && (Z_NegLimitFlag == 0)) {
        Z_NegLimitFlag = 1; // set flag to show Z Neg. limit is reached
        Serial.print("\nZ-axis: Hit Negative Limit sensor");
        send_UDP_msg("\nWARNING! Hit Negative Limit sensor on X-axis");
    } // set Flag back to 0 when Negative Limit sensor is not active any more
    if ((motor1.StatusReg().bit.InNegativeLimit == 0) && (Z_NegLimitFlag == 1)) {
        Z_NegLimitFlag = 0; // set flag to show Z Pos. limit is away
        Serial.print("\nZ-axis: moved away from Negative Limit Sensor");
    }
    //  Z-axis: when hit the InPositiveLimit Sensor, send a message once
    if ((motor1.StatusReg().bit.InPositiveLimit == 1) && (Z_PosLimitFlag == 0)) {
        Z_PosLimitFlag = 1; // set flag to show Z Pos. limit is reached
        Serial.print("\nZ-axis: Hit Positive Limit sensor");
        send_UDP_msg("\nWARNING! Hit Positive Limit Sensor on Z-axis");
    } // set Flag back to 0 when Positive Limit sensor is not active any more
    if ((motor1.StatusReg().bit.InPositiveLimit == 0) && (Z_PosLimitFlag == 1)) {
        Z_PosLimitFlag = 0;
        Serial.print("\nZ-axis: moved away from Positive Limit Sensor");
    }
// -------------- END Send UDP msg when Limit Sensors reached ---------------

// ---------------- Parser for received UDP messages ------------------------
    // Look for a received packet.
    int packetSize = Udp.parsePacket();
    if (packetSize > 0) {
        Serial.print("\n\nNEW PACKET! Received packet of size ");
        Serial.print(packetSize); 
        Serial.print(" bytes.");
        // printing IP address 
        Serial.print("\nRemote IP: ");
        IPAddress remote = Udp.remoteIP();
        for (int i = 0; i < 4; i++) {
            if (i < 3) {
                Serial.print(remote[i], DEC);
                Serial.print(".");
            }
            else {
                Serial.print(remote[i], DEC);
            }
        }
        Serial.print("  Remote port: ");
        Serial.print(Udp.remotePort());

        // Read the packet.
        int bytesRead = Udp.read(packetReceived, MAX_PACKET_LENGTH);
        Serial.print("\nPacket contents: ");
        // Normally if you sent text to be read by humans, you would use
        // Serial.print(). When sending raw data to be received by a machine, 
        // you would normally use Serial.write().
        Serial.write(packetReceived, bytesRead); // Serial.write(buf, len)

// Check this again, measue MAX distance value on BB
        // !!! IMPORTANT 1-20 bytes is the valid size for a received packet !
        if ( (0 < bytesRead) && (bytesRead < parser_buff_size) ) {  
            // CONFIG, E-STOP, STATUS, Home:X or Home:Z, X:50 and Z:50
            // if packetReceived[0-5] equal to "CONFIG"   
            if (memcmp(&packetReceived[0], "CONFIG", 6) == 0) {
                Serial.print("\nReceived UDP message: CONFIG");
                send_UDP_msg("Received UDP message: CONFIG");

            // if packetReceived[0-5] equal to "E-STOP"
            } else if (memcmp(&packetReceived[0], "E-STOP", 6) == 0) {
                Serial.print("\nReceived UDP message: E-STOP");
                send_UDP_msg("Received UDP message: E-STOP");

            // if packetReceived[0-5] equal to "STATUS"
            } else if (memcmp(&packetReceived[0], "STATUS", 6) == 0) {
                Serial.print("\nReceived UDP message: STATUS");
                send_UDP_msg("Received UDP message: STATUS");

                // Reading ClearCore registers:
                Serial.print((String)"\nmotor0.StatusReg().bit.AtTargetPosition value: " + motor0.StatusReg().bit.AtTargetPosition);
                send_UDP_msg("\nmotor0.StatusReg().bit.AtTargetPosition value: ");
                char bit_val[1]; 
                send_UDP_msg(itoa(motor0.StatusReg().bit.AtTargetPosition, bit_val, 2));

                Serial.print((String)"\nmotor1.StatusReg().bit.AtTargetPosition value: " + motor1.StatusReg().bit.AtTargetPosition);
                send_UDP_msg("\nmotor1.StatusReg().bit.AtTargetPosition value: ");
                send_UDP_msg(itoa(motor1.StatusReg().bit.AtTargetPosition, bit_val, 2));

            // if packetReceived[0-7] equal to "HOME:X,Z"
            } else if (memcmp(&packetReceived[0], "HOME:X,Z", 8) == 0) {
                // "HOME:X,z" will be interpreted as "HOME:X"
                Serial.print("\nReceived UDP message: HOME:X,Z");
                send_UDP_msg("Received UDP message: HOME:X,Z");
                
                // Home:X and Home:Z functions
                Homing_Z_axis(); // Go UP first to protect camera moving X-axis
                Homing_X_axis();
                
            // if packetReceived[0-5] equal to "HOME:X"
            } else if (memcmp(&packetReceived[0], "HOME:X", 6) == 0) {
                //Serial.println("memcmp: HOME:X");
                Serial.print("\nReceived UDP message: HOME:X");
                send_UDP_msg("Received UDP message: HOME:X");

                Homing_X_axis(); // call Home:X function

            // if packetReceived[0-5] equal to "HOME:Z"
            } else if (memcmp(&packetReceived[0], "HOME:Z", 6) == 0) {
                Serial.print("\nReceived UDP message: HOME:Z");
                send_UDP_msg("Received UDP message: HOME:Z");

                Homing_Z_axis(); // call Home:Z function

            // retrieving X distance:   //int t = 0 ;
            } else if ( (packetReceived[0] == 'X') && (packetReceived[1] == ':') ) {
                Serial.print("\nReceived 'X:'");
                int t = 0 ;
                char temp_str_X[10] = ""; // 8 positions to save a distance

                // take one symbol until we hit ' ' (space between X:1000 and Z:1000) and as long as the symbol is a digit or '-' sign
                // while [2+t] symbol ((not space) AND (is '-' OR digit))
                while ( (packetReceived[2 + t] != ' ')  &&  ( (packetReceived[2] == '-') || (isdigit(packetReceived[2 + t])) ) ) { 
                    // 2 is a shift for 'X:'

                // ! Remove after testing. Old check. 26 Mar 2024. This works.
                //while (packetReceived[2 + t] != ' ') { // 2 is a shift for 'X:'
                    temp_str_X[t] = packetReceived[t + 2];
                    //Serial.print((String)"\nvalue of packetReceived[" + (t+2) + "]: " + packetReceived[t+2]);
                    t++;
                }
                temp_str_X[t] = '\0'; // termination for temp-string
                // or temp[0]

                Serial.print("\nFull X string: ");
                Serial.write(temp_str_X, strlen(temp_str_X)); 
                //Serial.println();
                // atol() works perfectly with the minus sign '-' and digits
                dist_X = atol(temp_str_X); 
                Serial.print("\nlong int value of dist_X: ");
                Serial.print(dist_X);
                               
                Serial.print((String)"\nvalue of packetReceived[" + (t+2) + "]: " + packetReceived[t+2]); // this value should be space ' '

                // Check this
                t = t + 3; //shift for 3 positions: for X, for : and for for ' '
                // now t points to the next position after the space ' '
                if ((packetReceived[t]=='Z') && (packetReceived[t+1]==':')) {
                    Serial.print("\nReceived 'Z:'");
                    char temp_str_Z[10] = "";
                    t++; // step from Z to :
                    t++; // step from : to first digit
                    int i = 0;

                    // ALT: Added Apr 2
                    //while ( (packetReceived[t] != '\0') && ( (packetReceived[t] == '-') || (isdigit(packetReceived[t])) ) ) { 

                    while (packetReceived[t] != '\0') { // !! single quotes only
                        Serial.print((String)"\nvalue of t: " + t + "   value of i: " + i);
                        // if a character is the minus sign '-' or a digit, then add it to the character array
                        if ( (packetReceived[t] == '-') || (isdigit(packetReceived[t])) ) {//test if char is a digit or ' '
                            temp_str_Z[i] = packetReceived[t];
                            Serial.print((String)"\nvalue of packetReceived[" + t + "]: " + packetReceived[t]);
                            Serial.print((String)"\nvalue of temp_str_Z[" + i + "]: " + temp_str_Z[i]);
                            //Serial.println("The character is a number");
                        }
                        t++;
                        i++;
                    }
                    // !!! NOT SURE
                    temp_str_Z[i-1] = '\0';
                    Serial.print("\nFull Z string: ");
                    Serial.write(temp_str_Z, strlen(temp_str_Z)); // !!! pay attention: Serial.WRITE()
                    dist_Z = atol(temp_str_Z); 
                    // 2. Test this                
                    Serial.print("\nlong int value of dist_Z: ");
                    Serial.print(dist_Z);
                } // closing if (Z:)
                // send back UDP message
                send_UDP_msg("Received X:");
                char dist_X_char[8]; 
                itoa(dist_X, dist_X_char, 10);
                send_UDP_msg(itoa(dist_X, dist_X_char, 10));
                send_UDP_msg(", Z:");
                char dist_Z_char[8] ; 
                send_UDP_msg(itoa(dist_Z, dist_Z_char, 10));
                // send movements to motors
                motor_0_MoveDistance(dist_X); // X-axis move function
                motor_1_MoveDistance(dist_Z); // Z-axis move function

            //if all previous conditions are false
            } else { 
                Serial.print("\nReceived message was not recognized");
                send_UDP_msg("ERROR! Received message was not recognized.");
            }
            
        }  // closing if ( (0 < bytesRead) && (bytesRead < 21) )

        // for debug purposes
        //Serial.print((String)"\nInteger dist_X: " + dist_X);
        //Serial.print((String)"\nInteger dist_Z: " + dist_Z);
          
    } // closing if (packetSize > 0)
// ---------------- END of Parser for received UDP messages --------------------

    delay(10);    
}  // end of loop()

// this func. allows sending UDP messages in one line
// MUST be called after init with the two commands:
// "Ethernet.begin(mac, ip);" and "Udp.begin(localPort);"
int send_UDP_msg(char msg[]) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(msg); //Udp.write("Received UDP message: HOME:X,Z");
    Udp.endPacket();
}


//--------------------------- Motor 0 Beginning ------------------------------- 
// Moving X-axis motor 
bool motor_0_MoveDistance(int distance) {
// Check if a motor0 alert is currently preventing motion
// Clear alert if configured to do so 
    if (motor0.StatusReg().bit.AlertsPresent) {
        Serial.print("\nMotor 0 alert detected.");		
        motor_0_PrintAlerts();
        if (HANDLE_ALERTS) {
            motor_0_HandleAlerts();
        } else {
            Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.print("\nMotor 0: Move canceled.");		
        return false;
    }
    Serial.print("\nMotor 0: Moving distance: ");
    Serial.print(distance);

    // Command the move of incremental distance
    motor0.Move(distance);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.print("\nMotor 0: Moving.. Waiting for HLFB");

    uint32_t lastStatusTime = millis();
    while ( (!motor0.StepsComplete() || motor0.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor0.StatusReg().bit.AlertsPresent) {
        /////continue;
        // Periodically print out why the application is waiting
        if (millis() - lastStatusTime > 100) {
            Serial.print("\nMotor 0: Waiting for HLFB to assert");
            lastStatusTime = millis();
        }

        // Check if motor alert occurred during move
        // Clear alert if configured to do so 
        if (motor0.StatusReg().bit.AlertsPresent) {
            motor0.MoveStopAbrupt();
            Serial.print("\nMotor 0: alert detected.");		
            motor_0_PrintAlerts();
            if (HANDLE_ALERTS) {
                motor_0_HandleAlerts();
            } else {
                Serial.print("\nMotor 0: Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
            }
            Serial.print("\nMotor 0: Motion may not have completed as expected. Proceed with caution.");
            //Serial.println();
            return false;
        } else {
            // this message uppears before move is done
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
    Serial.print("    MotorFaulted ");
  }
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
// Moving Z-axis motor
bool motor_1_MoveDistance(int distance) {
// Check if a motor1 alert is currently preventing motion
// Clear alert if configured to do so 
  if (motor1.StatusReg().bit.AlertsPresent) {
    Serial.print("\nMotor 1 alert detected.");		
    motor_1_PrintAlerts();
    if (HANDLE_ALERTS) {
      motor_1_HandleAlerts();
    } else {
      Serial.print("\nEnable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.print("\nMotor 1: Move canceled.");		
    return false;
  }
  Serial.print("\nMotor 1: Moving distance: ");
  Serial.print(distance);
  // Command the move of incremental distance
  motor1.Move(distance);
  // Waits for HLFB to assert (signaling the move has successfully completed)
  Serial.print("\nMotor 1: Moving.. Waiting for HLFB");
    uint32_t lastStatusTime = millis();
    while ( (!motor1.StepsComplete() || motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor1.StatusReg().bit.AlertsPresent) {
      // Periodically print out why the application is waiting
      if (millis() - lastStatusTime > 100) {
        Serial.print("\nMotor 1: Waiting for HLFB to assert");
        lastStatusTime = millis();
      }
      // Check if motor alert occurred during move
      // Clear alert if configured to do so 
      if (motor1.StatusReg().bit.AlertsPresent) {
        motor1.MoveStopAbrupt();
        Serial.print("\nMotor 1: alert detected.");		
        motor_1_PrintAlerts();
        if (HANDLE_ALERTS) {
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
    Serial.print("    MotorFaulted ");
  }
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


int Homing_X_axis() {
    Serial.print("\nBeginning of function Homing_X_axis()");
    while (X_HomingDoneFlag != 1 ) {
        // when reached Positive Limit sensor
        if (motor0.StatusReg().bit.InNegativeLimit == 1) {  
            // 2 cases: NO_Flag or Flag
            // 1) Flag=0 case, set Flag to 1
            // 2. Flag=1 case, go one step away from sensor
            if (X_Homing_Flag == 0) { // 1st case
                X_Homing_Flag = 1;// set Flag 1 and go away from sensor
            }
            else if (X_Homing_Flag == 1) { // 2nd case 
                // start or continue to go away from sensor
                motor_0_MoveDistance(homing_step); // positive = RIGHT
            }
        }
        // now away from Negative Limit sensor, homing is done !
        else if (X_Homing_Flag == 1) { 
            X_Homing_Flag = 0; // set Flag to 0 for next homing
            Serial.print("\nX-axis: Homing is done");
            X_HomingDoneFlag = 1; // set 1 to exit the while loop
        }
        // Negative Limit sensor is not reached, X_Homing_Flag == 0
        // we go towards the Neg Limit sensor (one step LEFT)
        else { // NO_Flag, we need to move negative direction
            Serial.print("\nX_Homing_Flag:  "); // debug
            Serial.print(X_Homing_Flag); // debug
            Serial.print("\nNeed to move negative direction, step size: ");
            Serial.print(homing_step); 
            motor_0_MoveDistance(-homing_step);// one step negative=LEFT
        }
        delay(1); // This delay is very important: wihout it a ClearCore is slow with reading the Sensor state inside the while() loop. Removing this delay causes increase (around doubling) of distance from Sensor to Home position.

        // by this point Z_HomingDoneFlag must always be == 1
        Serial.print("\nX_HomingDoneFlag = ");
        Serial.print(X_HomingDoneFlag);
    } 

    X_HomingDoneFlag = 0; //this Flag to be ready for the next call Homing X  

    Serial.print("\nEnd of Homing_X_axis() func, X_HomingDoneFlag = ");  //debug
    Serial.print(X_HomingDoneFlag);                                    //debug
    dist_X = 0; // set X covered distance to 0, no distance in Homing function

    delay(400); // Need this delay for ClearCore correct work
    
    // send UDP message when X-axis Homing is finished
    send_UDP_msg("\nX-axis Homing is finished");
} 



// Homing Z-axis
// Each Homing function uses two different Flags
// These flags are used in four Homing functions, flag == 0 when we moving 
// towards a Limit sensor. After reaching the Limit Sensor, flag == 1 and we 
// move away form the Limit sensor
int Homing_Z_axis() {
    Serial.print("\nBeginning of function Homing_Z_axis()");
    Serial.print("\nZ_HomingDoneFlag = "); // for debug
    Serial.print(Z_HomingDoneFlag);        // for debug
    Serial.print("\nZ_Homing_Flag = ");    // for debug
    Serial.print(Z_Homing_Flag);           // for debug
    
    // !!!!!! CHECK case when we reach the sensor outside of Homing Function
    while (Z_HomingDoneFlag != 1 ) {
        // when reached Positive Limit sensor
        if (motor1.StatusReg().bit.InPositiveLimit == 1) {  
            // 2 cases: NO_Flag or Flag
            // 1. NO_Flag case, we set Flag to 1
            // 2. Flag==1 case,  
            Serial.print("\nmotor1.StatusReg().bit.InPositiveLimit: "); // debug
            Serial.print(motor1.StatusReg().bit.InPositiveLimit);    // debug
            if (Z_Homing_Flag == 0) {     // NO_Flag case: 
                Z_Homing_Flag = 1;// Flag==1, we go away from sensor
                Serial.print("\nSet Z_Homing_Flag = 1"); 
                Serial.print("\nZ_Homing_Flag:  "); // debug
                Serial.print(Z_Homing_Flag);      // debug
            }
            else if (Z_Homing_Flag == 1) { 
                // we start or continue to go negative=DOWN
                // we go one step away from the sensor
                Serial.print("\nZ_Homing_Flag:  ");  // debug
                Serial.print(Z_Homing_Flag);     // debug
                Serial.print("\nMoving DOWN, one step away from the sensor"); 
                motor_1_MoveDistance(-homing_step);
            }
        }
        // we are away from Positive Limit Sensor
        // motor1.StatusReg().bit.InPositiveLimit == 0
        else if (Z_Homing_Flag == 1) {// PosLimit == 0 // cases: Flag or NO_Flag
            Serial.print("\nmotor1.StatusReg().bit.InPositiveLimit: "); // debug
            Serial.print(motor1.StatusReg().bit.InPositiveLimit);    // debug
            Z_Homing_Flag = 0; //we are done
            Serial.print("\nZ_Homing_Flag:  "); // debug
            Serial.print(Z_Homing_Flag); // debug
            Serial.print("\nZ-axis: Homing is done");
            Z_HomingDoneFlag = 1;
        }
        // Positive Limit sensor is not reached, Z_Homing_Flag == 0
        // we go towards the Pos Limit sensor (go UP)
        else { // NO_Flag, we need to move positive direction
            Serial.print("\nZ_Homing_Flag:  "); // debug
            Serial.print(Z_Homing_Flag); // debug
            Serial.print("\nWe need to move positive direction, step size: ");
            Serial.print(homing_step); 
            motor_1_MoveDistance(homing_step);// go one step positive=UP
        }
        delay(1); // This delay is very important: wihout it a ClearCore is slow with reading the Sensor state inside the while() loop. Removing this delay causes increase (around doubling) of distance from Sensor to Home position.

        // by this point Z_HomingDoneFlag must always be == 1
        Serial.print("\nZ_HomingDoneFlag = ");
        Serial.print(Z_HomingDoneFlag);
    } 
    Z_HomingDoneFlag = 0; //set Flag to be ready for the next call Homing Z  

    Serial.print("\nEnd of Homing_Z_axis() func, Z_HomingDoneFlag = ");  //debug
    Serial.print(Z_HomingDoneFlag);                                    //debug
    dist_Z = 0; // set Z covered distance to 0, no distance in Homing function
    
    delay(400); // 250 surely does not work, 350 surely works, set 400 ms
    // 280 does not work, 285 ms works, a Fluctuation takes place

    // send UDP message when Z-axis Homing is finished
    send_UDP_msg("\nZ-axis Homing is finished");
} 



