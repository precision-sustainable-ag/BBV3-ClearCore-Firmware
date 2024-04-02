// 29 Mar 2024   BenchBot v3 ClearCore firmware_v0.1.1
// Sends a UDP response when movements are finished.
// This ver is close to being stable, but requires more testing with hardware
// Possibly, it has some bugs (saving old distances over cycles).

#include <Ethernet.h>  // needed only for Ethernet
#include "ClearCore.h" // needed only for Motor control

// --------- Ethernet block ---------
// Change the MAC address and IP address below to match your ClearCore's
// MAC address and IP.

//byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x33, 0x65};  // First BenchBot, in HFL
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x31, 0xc0};  // Second BenchBot

IPAddress ip(10, 95, 76, 21);  // IP-address 10.95.76.21

// The local port to listen for connections on.
unsigned int localPort = 8888;

// The maximum number of characters to receive from an incoming packet
#define MAX_PACKET_LENGTH 100
// Buffer for holding received packets.
char packetReceived[MAX_PACKET_LENGTH];

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Set this false if not using DHCP to configure the local IP address.
bool usingDhcp = false; // in BenchBot v3 we don't use DHCP, use static IP
// --------- end of Ethernet block ---------


// --------- Motor Control block ---------
// Specifies what motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
#define motor0 ConnectorM0  // for X-axis
#define motor1 ConnectorM1  // for Z-axis

// Select the baud rate to match the target serial device
// Speed in bps for serial-over-USB cable connection
#define baudRate 9600

// This example has built-in functionality to automatically clear motor alerts, including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately 
// when encountered and return a motor to a state in which motion is allowed. 
// Before enabling this functionality, be sure to understand this behavior and 
// ensure your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1) // in original sketch "MovePositionRelative" by 
// Teknic, the value of HANDLE_ALERTS was (0)

// Define the velocity and acceleration limits to be used for each move
// speed of 10000 is too high for Z-axis motor (Z-axis motor takes up to 5000 pulses per sec) 
// 3000 is too high for X axis
int velocityLimit = 3000; // pulses per sec, 5000 is max for vertical Z-axis

int accelerationLimit = 30000; //50000;// 100000; // pulses per sec^2

int homing_step = 10; // Step size for the Homing function, was 5 

// ClearCore has ARM Cortex-M4 32 bit processor. 32 bit Signed Integer can house a number from −2,147,483,648 to 2,147,483,647
int32_t dist_X = 0;
int32_t dist_Z = 0;

int X_NegLimitFlag = 0;  // flag == 1 while NegLimit sensor is reached
int X_PosLimitFlag = 0;
int Z_NegLimitFlag = 0;
int Z_PosLimitFlag = 0;

int X_MovingFlag = 0; // flag to state X-axis is moving
int Z_MovingFlag = 0; // flag to state Z-axis is moving

int CarrMovingFlag = 0; // 1 when X- or Z-axis is moving (or both)

// Each Homing function uses two different Flags
int Z_Homing_Flag = 0; //this Flag is used inside the Homing on Z-axis function 
int Z_HomingDoneFlag = 0; // 1 when homing is done

// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
bool motor_0_MoveDistance(int distance);
bool motor_1_MoveDistance(int distance);

void motor_0_PrintAlerts();
void motor_1_PrintAlerts();
void motor_0_HandleAlerts();
void motor_1_HandleAlerts();
// --------- end of Motor Control block ---------


// this loop in executed once, after turning ClearCore ON
void setup() {
    // Put your setup code here, it will run once:
 
    // Set up serial communication at a baud rate of 9600 bps then wait up to
    // 5 seconds for a port to open.
    Serial.begin(9600);
    uint32_t timeout = 5000;
    uint32_t startTime = millis();
    while (!Serial && millis() - startTime < timeout) {
        continue;
    }
    
    // --------- Ethernet block ---------
    // Get the Ethernet module up and running.
    if (usingDhcp) {
        int dhcpSuccess = Ethernet.begin(mac);
        if (dhcpSuccess) {
            Serial.println("DHCP configuration was successful.");
        }
        else {
            Serial.println("DHCP configuration was unsuccessful!");
            Serial.println("Try again using a manual configuration...");
            while (true) {
                // UDP will not work without a configured IP address.
                continue;
            }
        }
    }
    else {
        Ethernet.begin(mac, ip);
    }
    // Make sure the physical link is up before continuing.
    while (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("The Ethernet cable is unplugged...");
        delay(1000);
    }
    // Begin listening on the local port for UDP datagrams
    Udp.begin(localPort);
    // --------- end of Ethernet block ---------


    // --------- Motor Control block ---------
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

    // Set the maximum velocity for each move
    motor0.VelMax(velocityLimit);
    motor1.VelMax(velocityLimit);

    // Set the maximum acceleration for each move
    motor0.AccelMax(accelerationLimit);
    motor1.AccelMax(accelerationLimit);

    // Enables the motor; homing will begin automatically if enabled
    motor0.EnableRequest(true);
    Serial.println("Motor 0 Enabled");
    motor1.EnableRequest(true);
    Serial.println("Motor 1 Enabled");

    // Waits for HLFB to assert 
    // this means: waits for both motors to finish enabling
    uint32_t lastStatusTime = millis();
    Serial.println("Waiting for HLFB...");
    while (motor0.HlfbState() != MotorDriver::HLFB_ASSERTED &&
        motor1.HlfbState() != MotorDriver::HLFB_ASSERTED &&
        !motor0.StatusReg().bit.AlertsPresent && 
        !motor1.StatusReg().bit.AlertsPresent) {
        // Periodically prints out why the application is waiting
        if (millis() - lastStatusTime > 100) {
            Serial.println("Waiting for HLFB to assert on both motors");
            lastStatusTime = millis();
        }    
    }
    // Check if motor alert occurred during enabling
    // Clear alert if configured to do so 

    // Motor 0: if there is an alert, then motor_0_HandleAlerts();
    if (motor0.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor 0: alert detected.");		
        motor_0_PrintAlerts();
        if(HANDLE_ALERTS){
            motor_0_HandleAlerts();
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Motor 0: Enabling may not have completed as expected. Proceed with caution.");		
        Serial.println();
    } else {
        Serial.println("Motor 0 is Ready");	
    }

    // Motor 1: if there is an alert, then motor_1_HandleAlerts();
    if (motor1.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor 1: alert detected.");		
        motor_1_PrintAlerts();
        if(HANDLE_ALERTS) {
            motor_1_HandleAlerts();
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Motor 1: Enabling may not have completed as expected. Proceed with caution.");
        Serial.println();
    } else {
        Serial.println("Motor 1 is Ready");	
    }
    // --------- end of Motor Control block ---------


    // Setting Negative and Positive Limit Sensors for X- and Z-axes
    // Set Negative Limit switch for Motor 0 (X-axis)
    if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0)) {
        // M-0's negative limit switch is now set to IO-0 and enabled.
        Serial.println("I/O-0 is now set to Negative Sensor for ConnectorM0 and enabled");	
    }
    // Set Positive Limit switch for Motor 0 (X-axis)
    if (ConnectorM0.LimitSwitchPos(CLEARCORE_PIN_IO1)) {
        // M-0's positive limit switch is now set to IO-1 and enabled.
        Serial.println("I/O-1 is now set to Positive Sensor for ConnectorM0 and enabled");
    }
    // Set Negative Limit switch for Motor 1 (Z-axis)
    if (ConnectorM1.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
        // M-1's negative limit switch is now set to IO-2 and enabled.
        Serial.println("I/O-2 is now set to Negative Sensor for ConnectorM1 and enabled");	
    }
    // Set Positive Limit switch for Motor 1 (Z-axis)
    if (ConnectorM1.LimitSwitchPos(CLEARCORE_PIN_IO3)) {
        // M-1's positive limit switch is now set to IO-3 and enabled.
        Serial.println("I/O-3 is now set to Positive Sensor for ConnectorM1 and enabled");
    }
    delay(300); // this delay is needed to give ClearCore the time for
    // assigning values to the Registers (could be 250+ ms)
} // END of setup() loop


// this loop in executed all the time, after executing the setup() loop
void loop() {    // Put your main code here, it will run repeatedly:
    /************************ Reading Motor Register ************************/
    MotorDriver *motor_0 = &ConnectorM0; // X-axis
    MotorDriver *motor_1 = &ConnectorM1; // Z-axis
    // A variable should be declared volatile whenever its value can be changed by something beyond the control of the code section in which it appears

    // motor_0 is instance of MotorDriver, motor0 is just "ConnectorM0" 
    // scope of statusReg_0 and statusReg_1 is only the main loop()
    //volatile const MotorDriver::StatusRegMotor &statusReg_0 = motor_0->StatusReg();
    //volatile const MotorDriver::StatusRegMotor &statusReg_1 = motor_1->StatusReg();
    
    // An example of how to read the Motor Registers
    // (motor0.StatusReg().bit.InNegativeLimit == 1)

// -------------- Sending UDP response when movements are finished -------------
    // This if block tracks the state of the carriage: is it moving or not.
    // if any of two motors is moving, or both are moving
    // if ((X is moving OR Z is moving) AND CarrMovingFlag == 0) 
    if ( ( (motor0.StatusReg().bit.AtTargetPosition == 0) || (motor1.StatusReg().bit.AtTargetPosition == 0) ) && (CarrMovingFlag == 0) ) {
        // then we set the CarrMovingFlag to 1. Means, some movement is being done.
        CarrMovingFlag = 1; // X or Z is moving, set this Flag to 1
        // some movement has started = some movement is being done
        //Serial.print("Some movement has started, CarrMovingFlag = "); // debug
        //Serial.println(CarrMovingFlag); // debug
    } 

    // This block sends a UDP message, when movements are finished.
    // Particularly, in case when there was a moving, but now both motors stopped.
    // if (X is not moving AND Z is not moving AND CarrMovingFlag == 1)
    if ( (motor0.StatusReg().bit.AtTargetPosition == 1) && (motor1.StatusReg().bit.AtTargetPosition == 1) && (CarrMovingFlag == 1) ) {
        // then we have finished all movements
        CarrMovingFlag = 0;
        // send an UDP message that all movements were finished
        Serial.print("All movements have finished, CarrMovingFlag = "); // for debug
        Serial.println(CarrMovingFlag);   // for debug

        // Sending a response about finishing all movements
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());

        // CHECK !!!!!!!!!!!!!!!!!!!!!!!
        // A possible bug is here, in next movement cycle we can have old dist values
        // when moving only X, dist_Z may stay from the prev. move, not 0 !
  
        // 3 cases: 1) moved only Z, 2) moved X and Z, 3) moved only X
        if (dist_X == 0) {    // 1st case: moved only Z
            // movement was ONLY on Z-axis
            Serial.print("Movement was only on Z-axis"); 
            // send UDP message
            Udp.write("\nAll movements have finished, Z-axis covered distance = ");
    
            char Z_axis_char[8];   // these two line is just int-to-char convertion
            Udp.write(itoa(dist_Z, Z_axis_char, 10) );  // and sending a UDP msg 
        } 
        else { // 2nd case: moved both X-axis and Z-axis OR only X-axis
            if (dist_X != 0 && dist_Z != 0) {
                // movement was on both X-axis and Z-axis
                Serial.print("Movement was on both X- and Z-axes"); 
                // send UDP message
                Udp.write("\nAll movements have finished, X-axis covered distance = ");
                char X_axis_char[8];   // these two line is just int-to-char convertion 
                Udp.write(itoa(dist_X, X_axis_char, 10) );  // and sending a UDP msg
                Udp.write(", Z-axis covered distance = ");
                char Z_axis_char[8];   // these two line is just int-to-char convertion 
                Udp.write(itoa(dist_Z, Z_axis_char, 10) );  // and sending a UDP msg
            } else { 
                // 3rd case: moved ONLY X-axis
                Serial.print("Movement was only on X-axis"); 
                // send UDP message
                Udp.write("\nAll movements have finished, X-axis covered distance = ");
                char X_axis_char[8];  // these two line is just int-to-char convertion 
                Udp.write(itoa(dist_X, X_axis_char, 10) );  // and sending a UDP msg
            }
        }
 
        Udp.endPacket();
    }
// -------------- END Sending UDP response when movements are finished ---------

    
// -------------- Sending UDP msg when Limit Sensors reached -------------------
    // X-axis: when reach the InNegativeLimit Sensor, send a message once
    // if (NegLimit == reached AND X_NegLimitFlag == 0)
    if ((motor0.StatusReg().bit.InNegativeLimit == 1) && (X_NegLimitFlag == 0)) {
        Serial.print("InNegativeLimit:  ");
        Serial.println(motor0.StatusReg().bit.InNegativeLimit);
        X_NegLimitFlag = 1; // set flag to show X Neg Limit is reached 
        Serial.print("X_NegLimitFlag:  "); 
        Serial.println(X_NegLimitFlag);
        // send a UDP message about reaching Negative Limit on X-axis
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Negative Limit Sensor on axis X");
        Udp.endPacket();        
    } 
    // setting Flag back to 0 when NegativeLimit Sensor is not active any more
    if ((motor0.StatusReg().bit.InNegativeLimit == 0) && (X_NegLimitFlag == 1)) {
        X_NegLimitFlag = 0;
        Serial.print("X-axis: moved out from the Negative Limit Sensor");
        Serial.print("X_NegLimitFlag: "); 
        Serial.println(X_NegLimitFlag);
    }

    // X-axis: when reach the InPositiveLimit Sensor, send a message once
    // if (PosLimit == reached AND X_PosLimitFlag == 0)
    if ((motor0.StatusReg().bit.InPositiveLimit == 1) && (X_PosLimitFlag == 0)) {
        Serial.print("InPositiveLimit:  ");
        Serial.println(motor0.StatusReg().bit.InPositiveLimit);
        X_PosLimitFlag = 1;
        Serial.print("X_PosLimitFlag:  "); 
        Serial.println(X_PosLimitFlag);
        // send a UDP message about reaching Positive Limit on X-axis
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Positive Limit Sensor on axis X");
        Udp.endPacket();        
    } 
    // setting Flag back to 0 when PositiveLimit Sensor is not active any more
    if ((motor0.StatusReg().bit.InPositiveLimit == 0) && (X_PosLimitFlag == 1)) {
        X_PosLimitFlag = 0;
        Serial.print("X-axis: moved out out from the Positive Limit Sensor");
        Serial.print("X_PosLimitFlag: "); 
        Serial.println(X_PosLimitFlag);
    }


    // Z-axis: when reach the InNegativeLimit Sensor, send a message once
    // if (NegLimit == reached AND Z_NegLimitFlag == 0)
    if ((motor1.StatusReg().bit.InNegativeLimit == 1) && (Z_NegLimitFlag == 0)) {
        Serial.print("InNegativeLimit:  ");
        Serial.println(motor1.StatusReg().bit.InNegativeLimit);
        Z_NegLimitFlag = 1;
        Serial.print("Z_NegLimitFlag:  "); 
        Serial.println(Z_NegLimitFlag);
        // send a UDP message about reaching Negative Limit on Z-axis
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Negative Limit Sensor on axis Z");
        Udp.endPacket();        
    } // setting Flag back to 0 when NegativeLimit Sensor is not active any more
    if ((motor1.StatusReg().bit.InNegativeLimit == 0) && (Z_NegLimitFlag == 1)) {
        Z_NegLimitFlag = 0;
        Serial.print("Z-axis: moved out from the Negative Limit Sensor");
        Serial.print("Z_NegLimitFlag: "); 
        Serial.println(Z_NegLimitFlag);
    }

    //  Z-axis: when hit the InPositiveLimit Sensor, send a message once
    // if (PosLimit == reached AND Z_PosLimitFlag == 0)
    if ((motor1.StatusReg().bit.InPositiveLimit == 1) && (Z_PosLimitFlag == 0)) {
        Serial.print("InPositiveLimit:  ");
        Serial.println(motor1.StatusReg().bit.InPositiveLimit);
        Z_PosLimitFlag = 1;
        Serial.print("Z_PosLimitFlag:  "); 
        Serial.println(Z_PosLimitFlag);
        // send a UDP message about reaching Positive Limit on Z-axis
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Positive Limit Sensor on axis Z");
        Udp.endPacket();        
    } // setting Flag back to 0 when PositiveLimit Sensor is not active any more
    if ((motor1.StatusReg().bit.InPositiveLimit == 0) && (Z_PosLimitFlag == 1)) {
        Z_PosLimitFlag = 0;
        Serial.print("Z-axis: moved out from the Positive Limit Sensor"); // add Axis
        Serial.print("Z_PosLimitFlag: "); 
        Serial.println(Z_PosLimitFlag);
    }
// -------------- END Sending UDP msg when Limit Sensors reached ---------------


    // Look for a received packet.
    int packetSize = Udp.parsePacket();
    if (packetSize > 0) {
        Serial.print("Received packet of size ");
        Serial.print(packetSize); 
        Serial.println(" bytes.");
        // printing IP address 
        Serial.print("Remote IP: ");
        IPAddress remote = Udp.remoteIP();
        for (int i = 0; i < 4; i++) {
            if (i < 3) {
                Serial.print(remote[i], DEC);
                Serial.print(".");
            }
            else {
                Serial.println(remote[i], DEC);
            }
        }

        Serial.print("Remote port: "); // print port 
        Serial.println(Udp.remotePort());

        // Read the packet.
        int bytesRead = Udp.read(packetReceived, MAX_PACKET_LENGTH);
        Serial.print("Packet contents: ");
        // Normally when we send text to be read by humans, we would use
        // Serial.print(). When send raw data to be received by a machine, 
        // we would normally use Serial.write()
        Serial.write(packetReceived, bytesRead); // Serial.write(buf, len)
        Serial.println();
                  
        //Serial.print("\n  bytesRead value: "); // for debug
        //Serial.print(bytesRead);               // for debug
        //Serial.print("\n");                    // for debug
        int i = 0;
        while (i <= bytesRead) {
            Serial.println((String)"packetReceived[" + i + "]: " + packetReceived[i]);
            
            // retrieving X distance:
            if ( (packetReceived[i] == 'X') && (packetReceived[i+1] == ':') ) {
                Serial.print("Received 'X:'");
                int t = 0 ;  
                char temp_str[bytesRead - i];
                // while (not space symbol OR end of char array symbol)
                while ( (packetReceived[t + i + 2] != ' ') && (packetReceived[t + i + 2] != '\0')) {
                    temp_str[t] = packetReceived[t + i + 2];
                    //Serial.print("\nvalue of t: "); Serial.print(t); // debug
                    t++;
                }
                temp_str[t] = '\0'; // put '\0' at the end char array
                Serial.print("\nFull X string: ");
                Serial.write(temp_str, strlen(temp_str)); 
                Serial.println();
                //Serial.println((String)"temp_str:" + temp_str); // for debug
                dist_X = atol(temp_str); // transform char array to integer
            } 
            // retrieving Z distance:
            if ( (packetReceived[i] == 'Z') && (packetReceived[i+1] == ':') ) {
                Serial.print("Received 'Z:'");
                int t = 0 ;  
                char temp_str[bytesRead - i];
                // while (not space symbol OR end of char array symbol)
                while ( (packetReceived[t + i + 2] != ' ') && (packetReceived[t + i + 2] != '\0') ) {
                    temp_str[t] = packetReceived[t + i + 2];
                    //Serial.print("\nvalue of t: "); Serial.print(t); // debug
                    t++;
                }
                temp_str[t] = '\0'; // put '\0' at the end char array
                Serial.print("\nFull Z string: ");
                Serial.write(temp_str, strlen(temp_str)); 
                Serial.println();
                //Serial.println((String)"temp_str:" + temp_str);
                dist_Z = atol(temp_str); // transform char array to integer
            } 
           
            i++;    
        }
        // for debug purposes
        //Serial.println((String)"Integer dist_X: " + dist_X);
        //Serial.println((String)"Integer dist_Z: " + dist_Z);
        

        // !!! Feb-2, this block is UNTESTED
        // An attempt to implement Homing on X and Z-axises
        // if received string starts from "Home:"
        if ( (packetReceived[0] == 'H') && (packetReceived[1] == 'o') && (packetReceived[2] == 'm') && (packetReceived[3] == '3') && (packetReceived[4] == ':') ) {
            if ((packetReceived[5] == 'X')) {
                // Homing on X-axis
                Serial.print("Homing: X-axis");

            } else {
                if ((packetReceived[5] == 'Z')) {
                  // Homing on Z-axis
                  Serial.print("Homing: Z-axis");
                }
            }
        } 


        // Sending a UDP message back to the sender
        // with the retrieved X and Z distances
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Received X:");
        char X_dist_char[8];                      // these two line is just 
        Udp.write(itoa(dist_X, X_dist_char, 10) );  // int-to-char convertion
        Udp.write(" Received Z:");
        char Z_dist_char[8];   // these two line is just int-to-char convertion
        Udp.write(itoa(dist_Z, Z_dist_char, 10) );  // and sending a UDP msg
        Udp.endPacket();
        
        // if receive X:999 Z:999, call Homing for Z-axis function
        if ( (dist_X == 999) && (dist_Z == 999) ) {
            Homing_Z_axis();
        } else {
            // if received distances are not X:999 Z:999, then pass the
            // distances to motors in order to move
            motor_0_MoveDistance(dist_X);
            motor_1_MoveDistance(dist_Z);
        }   
    }

    delay(10);    
}  // end of loop()


// Functions, needed for control of Motor 0 and Motor 1
//--------------------------- Motor 0 Beginning ------------------------------- 
bool motor_0_MoveDistance(int distance) {
// Check if a motor0 alert is currently preventing motion
// Clear alert if configured to do so 
    if (motor0.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor 0 alert detected.");		
        motor_0_PrintAlerts();
        if (HANDLE_ALERTS) {
            motor_0_HandleAlerts();
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Motor 0: Move canceled.");		
        Serial.println();
        return false;
    }
    Serial.print("Motor 0: Moving distance: ");
    Serial.println(distance);

    // Command the move of incremental distance
    motor0.Move(distance);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Motor 0: Moving.. Waiting for HLFB");

    uint32_t lastStatusTime = millis();
    while ( (!motor0.StepsComplete() || motor0.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor0.StatusReg().bit.AlertsPresent) {
        /////continue;
        // Periodically print out why the application is waiting
        if (millis() - lastStatusTime > 100) {
            Serial.println("Motor 0: Waiting for HLFB to assert");
            lastStatusTime = millis();
        }
        // Check if motor alert occurred during move
        // Clear alert if configured to do so 
        if (motor0.StatusReg().bit.AlertsPresent) {
            motor0.MoveStopAbrupt();
            Serial.println("Motor 0: alert detected.");		
            motor_0_PrintAlerts();
            if (HANDLE_ALERTS) {
                motor_0_HandleAlerts();
            } else {
                Serial.println("Motor 0: Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
            }
            Serial.println("Motor 0: Motion may not have completed as expected. Proceed with caution.");
            Serial.println();
            return false;
        } else {
            // this message uppears before move is done
            Serial.println("Motor 0: Move sent to motor");
            return true;
        }
    }
}

/*
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void motor_0_PrintAlerts() {
    // report status of alerts on motor0
    Serial.println("Motor 0: Alerts present: ");
    if (motor0.AlertReg().bit.MotionCanceledInAlert) {
        Serial.println("    MotionCanceledInAlert "); }
    if (motor0.AlertReg().bit.MotionCanceledPositiveLimit) {
        Serial.println("    MotionCanceledPositiveLimit "); }
    if (motor0.AlertReg().bit.MotionCanceledNegativeLimit) {
        Serial.println("    MotionCanceledNegativeLimit "); }
    if (motor0.AlertReg().bit.MotionCanceledSensorEStop) {
        Serial.println("    MotionCanceledSensorEStop "); }
    if (motor0.AlertReg().bit.MotionCanceledMotorDisabled) {
        Serial.println("    MotionCanceledMotorDisabled "); }
    if (motor0.AlertReg().bit.MotorFaulted){
        Serial.println("    MotorFaulted ");
    }
}

/*
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void motor_0_HandleAlerts() {
    if (motor0.AlertReg().bit.MotorFaulted) {
        // if a motor fault is present, clear it by cycling enable
        Serial.println("Motor 0: Faults present. Cycling enable signal to motor to clear faults.");
        motor0.EnableRequest(false);
        Delay_ms(10);
        motor0.EnableRequest(true);
    }
    // clear alerts
    Serial.println("Motor 0: Clearing alerts.");
    motor0.ClearAlerts();
}
//--------------------------- Motor 0 End --------------------------------------


//--------------------------- Motor 1 Beginning ------------------------------- 
bool motor_1_MoveDistance(int distance) {
    // Check if a motor1 alert is currently preventing motion
    // Clear alert if configured to do so 
    if (motor1.StatusReg().bit.AlertsPresent) {
        Serial.println("Motor 1 alert detected.");		
        motor_1_PrintAlerts();
        if (HANDLE_ALERTS) {
            motor_1_HandleAlerts();
        } else {
            Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
        }
        Serial.println("Motor 1: Move canceled.");		
        Serial.println();
        return false;
    }
    Serial.print("Motor 1: Moving distance: ");
    Serial.println(distance);

    // Command the move of incremental distance
    motor1.Move(distance);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Motor 1: Moving.. Waiting for HLFB");

    uint32_t lastStatusTime = millis();
    while ( (!motor1.StepsComplete() || motor1.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor1.StatusReg().bit.AlertsPresent) {
        /////continue;
        // Periodically print out why the application is waiting
        if (millis() - lastStatusTime > 100) {
            Serial.println("Motor 1: Waiting for HLFB to assert");
            lastStatusTime = millis();
        }

        // Check if motor alert occurred during move
        // Clear alert if configured to do so 
        if (motor1.StatusReg().bit.AlertsPresent) {
            motor1.MoveStopAbrupt();
            Serial.println("Motor 1: alert detected.");		
            motor_1_PrintAlerts();
            if (HANDLE_ALERTS) {
                motor_1_HandleAlerts();
            } else {
                Serial.println("Motor 1: Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
            }
            Serial.println("Motor 1: Motion may not have completed as expected. Proceed with caution.");
            Serial.println();
            return false;
        } else {
            Serial.println("Motor 1: Move sent to motor");
            return true;
        }
    }
}

/*
 * PrintAlerts
 *
 *    Prints active alerts.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void motor_1_PrintAlerts() {
    // report status of alerts on motor0
    Serial.println("Motor 1: Alerts present: ");
    if (motor1.AlertReg().bit.MotionCanceledInAlert) {
        Serial.println("    MotionCanceledInAlert "); }
    if (motor1.AlertReg().bit.MotionCanceledPositiveLimit) {
        Serial.println("    MotionCanceledPositiveLimit "); }
    if (motor1.AlertReg().bit.MotionCanceledNegativeLimit) {
        Serial.println("    MotionCanceledNegativeLimit "); }
    if (motor1.AlertReg().bit.MotionCanceledSensorEStop) {
        Serial.println("    MotionCanceledSensorEStop "); }
    if (motor1.AlertReg().bit.MotionCanceledMotorDisabled) {
        Serial.println("    MotionCanceledMotorDisabled "); }
    if (motor1.AlertReg().bit.MotorFaulted) {
        Serial.println("    MotorFaulted ");
    }
}

/*
 * HandleAlerts
 *
 *    Clears alerts, including motor faults. 
 *    Faults are cleared by cycling enable to the motor.
 *    Alerts are cleared by clearing the ClearCore alert register directly.
 *
 * Parameters:
 *    requires "motor" to be defined as a ClearCore motor connector
 *
 * Returns: 
 *    none
 */
void motor_1_HandleAlerts() {
    if (motor1.AlertReg().bit.MotorFaulted) {
        // if a motor fault is present, clear it by cycling enable
        Serial.println("Motor 1: Faults present. Cycling enable signal to motor to clear faults.");
        motor1.EnableRequest(false);
        Delay_ms(10);
        motor1.EnableRequest(true);
    }
    // clear alerts
    Serial.println("Motor 1: Clearing alerts.");
    motor1.ClearAlerts();
}
//--------------------------- Motor 1 End --------------------------------------


// Homing for Z-axis. Works.
int Homing_Z_axis() {
    //Serial.print("Beginning of function Homing_Z_axis(). Z_HomingDoneFlag = ");   // for debug
    //Serial.println(Z_HomingDoneFlag); // for debug

    // while Homing is not finished
    while (Z_HomingDoneFlag != 1 ) {
        // if reached Negative Limit Sensor
        if (motor1.StatusReg().bit.InNegativeLimit == 1) {  
        // 2 cases: Z_Homing_Flag == 0 or Z_Homing_Flag == 1
            
            // for debug
            Serial.print("motor1.StatusReg().bit.InNegativeLimit:  "); 
            Serial.println(motor1.StatusReg().bit.InNegativeLimit); 

            // 1st case: if Z_Homing_Flag is 0, we set it to 1 and start Homing
            if (Z_Homing_Flag == 0) { 
                Z_Homing_Flag = 1;// set Flag=1, we just starting going positive dir
                Serial.println("Set Z_Homing_Flag = 1"); 

                Serial.println("Z_Homing_Flag:  "); // for debug
                Serial.println(Z_Homing_Flag);      // for debug
            }
            // 2nd case: if Z_Homing_Flag is 1, we started homing earlier
            // so, we do a homing step
            else if (Z_Homing_Flag == 1) { 
                // go one step in POSITIVE direction
                //Serial.print("Z_Homing_Flag:  "); // for debug
                //Serial.println(Z_Homing_Flag); // for debug
                Serial.println("Moving one step to positive direction"); 
                motor_1_MoveDistance(homing_step); //one step in POSITIVE direct
            }
        }

        // when Negative Limit sensor is not reached
        // 2 cases: moved to Neg Limit or moved away from Neg Limit sensor
        else if (Z_Homing_Flag == 1) {// NegLimit == 0 // cases: Flag or NO_Flag
            // Z_Homing_Flag == 1, when moved away from Neg Limit sensor.
            // Homing is Done.
            
            // for debug
            Serial.print("motor1.StatusReg().bit.InNegativeLimit:  "); 
            Serial.println(motor1.StatusReg().bit.InNegativeLimit); 

            Z_Homing_Flag = 0; //we are done
            //Serial.print("Z_Homing_Flag:  ");   // for debug 
            //Serial.println(Z_Homing_Flag);      // for debug
            Serial.println("Z-axis: Homing is done");
            Z_HomingDoneFlag = 1; // finished Homing
        }
        else { 
            // Z_Homing_Flag == 0, we need to move negative direction 
            // (towards the Negative Limit sensor)
            //Serial.print("Z_Homing_Flag:  ");   // for debug
            //Serial.println(Z_Homing_Flag);      // for debug
            Serial.println("We need to move negative direction, step size:");
            Serial.println(-homing_step);

            motor_1_MoveDistance(-homing_step); // go 1 step in negative direction
        }
        delay(1); // This delay is very important: wihout it a ClearCore
        // is slow with reading the Sensor state inside the while() loop.
        // Removing this delay causes doubling of the distance between 
        // Sensor and Home position.
    }
    
    Z_HomingDoneFlag = 0;
     
    Serial.print("End of function Homing_Z_axis(). Z_HomingDoneFlag = ");
    Serial.println(Z_HomingDoneFlag);
}



