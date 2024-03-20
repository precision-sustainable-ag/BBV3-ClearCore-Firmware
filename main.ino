// 11 Jan 2024 ver. 0.06 : Homing on Z-axis implemented
// Homing logic. Home is the leftmost position without activating NegLimit sensor. We go left (negative direction) until we hit NegLimit sensor. Next we go right (positive direction) one small step at a time until NegLimit sensor becomes not active. Done when reached the leftmost position with NO active NegLimit sensor. Home position is reached.  
// On Z-axis, "Home" position is with the carriage in the upmost position.

/*
Homing (Z-axis)
// input 2 cases: 1. NegSensor is active, 2. NegSensor isn't active
Flag = 0
if NegLimit == 1 // 2 cases: Flag or NO_Flag
  if NO_Flag: 
    Flag = 1 // we just starting going positive direction
  else (if Flag)
    one step in positive direction // we're starting or continuing to go positiv

else // NegLimit == 0 // cases: Flag or NO_Flag
  if Flag:
    we are done, Flag = 0
  else // NO_Flag, 
    // we need to move negative direction
    go 1 step i negative direction  // Question if one step is enough?
     

// alt
if ( (NegLimit == 1) && (NegRchdFlag == 0) ) {
    NegRchdFlag = 1
    go one step right
} else {
    go one step left
}
*/

// ver. 0.05 - flag for reaching NegLimit was added. Program sends a UDP message when NegLimit is reached.
// 20 Dec 2023   ver. 0.03 : UDP-based, can move 2 motors. Implemented a receiver of two coordinates (X and Z) through UDP.

// To control two motors, please, send a UDP packet to 192.168.0.121, port 8888.
// in Linux can use netcat: "nc -u 192.168.0.121 8888"
// Message format: X:number Z:number, number is an integet, can be negative
// Opposite order also works: Z:number X:number
// 0-th motor controls X-axis (horiz.), 1-st motor controls Z-axis (vertical)

/*Links:
 * ** ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
 * ** ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
 */

#include <Ethernet.h>  // needed only for Ethernet
#include "ClearCore.h" // needed only for Motor control

// --------- Ethernet block ---------
// Change the MAC address and IP address below to match your ClearCore's
// MAC address and IP.
byte mac[] = {0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01};
IPAddress ip(192, 168, 0, 121);

// The local port to listen for connections on.
unsigned int localPort = 8888;

// The maximum number of characters to receive from an incoming packet
#define MAX_PACKET_LENGTH 100
// Buffer for holding received packets.
char packetReceived[MAX_PACKET_LENGTH];

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Set this false if not using DHCP to configure the local IP address.
bool usingDhcp = false;
// --------- end of Ethernet block ---------


// --------- Motor Control block ---------
// Specifies what motor to move.
// Options are: ConnectorM0, ConnectorM1, ConnectorM2, or ConnectorM3.
/////#define motor ConnectorM0
#define motor0 ConnectorM0
#define motor1 ConnectorM1

// Select the baud rate to match the target serial device
#define baudRate 9600

// This example has built-in functionality to automatically clear motor alerts, 
//  including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when 
//	encountered and return a motor to a state in which motion is allowed. Before 
//	enabling this functionality, be sure to understand this behavior and ensure 
//	your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1) // !!! in original sketch "MovePositionRelative" value of HANDLE_ALERTS was (0)

// Define the velocity and acceleration limits to be used for each move
// speed of 10000 is too high for Z-axis motor (Z-axis motor takes up to 5000 pulses per sec) 
int velocityLimit = 3000; // pulses per sec, 5000 is max for vertical Z-axis
////int Homing_velocityLimit = 500;
int accelerationLimit = 100000; // pulses per sec^2

// 20 -> 18mm distance after homing
int homing_step = 10; // Size of a step for Homing function. Was 10, was 5. 

int X_NegLimitFlag = 0;  // flag = 1 while NegLimit sensor is reached
int X_PosLimitFlag = 0;
int Z_NegLimitFlag = 0;
int Z_PosLimitFlag = 0;

// Each Homing function uses two different Flags
int Z_Homing_Flag = 0; //This Flag is used inside the Homing on Z-axis function 
int Z_HomingDoneFlag = 0; //

// Declares user-defined helper functions.
// The definition/implementations of these functions are at the bottom of the sketch.
/////bool MoveDistance(int distance);
bool motor_0_MoveDistance(int distance);
bool motor_1_MoveDistance(int distance);

void motor_0_PrintAlerts();
void motor_1_PrintAlerts();
void motor_0_HandleAlerts();
void motor_1_HandleAlerts();

// --------- end of Motor Control block ---------

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
    /////motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor0.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
    motor1.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);

    // Set the HFLB carrier frequency to 482 Hz
    /////motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor0.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);
    motor1.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

    // Sets the maximum velocity for each move
    /////motor.VelMax(velocityLimit);
    motor0.VelMax(velocityLimit);
    motor1.VelMax(velocityLimit);

    // Set the maximum acceleration for each move
    /////motor.AccelMax(accelerationLimit);
    motor0.AccelMax(accelerationLimit);
    motor1.AccelMax(accelerationLimit);

    // Enables the motor; homing will begin automatically if enabled
    /////motor.EnableRequest(true);
    /////Serial.println("Motor Enabled");
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
		    if(HANDLE_ALERTS){
			      motor_1_HandleAlerts();
		    } else {
			      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		    }
		    Serial.println("Motor 1: Enabling may not have completed as expected. Proceed with caution.");		
 		    Serial.println();
	  } else {
		    Serial.println("Motor 1 is Ready");	
	  }
// old version from original example by Teknic ("DualAxisSyncronized" sketch)
/* old version was not able to say from what motor an alert is 
    if (motor0.StatusReg().bit.AlertsPresent || 
		motor1.StatusReg().bit.AlertsPresent) {
		    Serial.println("Motor alert detected.");		
		    motor_0_PrintAlerts();
		    if(HANDLE_ALERTS){
			      motor_0_HandleAlerts();
            motor_1_HandleAlerts(); // not sure if a call of HandleAlerts() for a motor having no alerts 

		    } else {
			      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		    }
		    Serial.println("Enabling may not have completed as expected. Proceed with caution.");		
 		    Serial.println();
	  } else {
		    Serial.println("Motors Ready");	
	  }
*/    
  // --------- end of Motor Control block ---------

  // Negative and Positive end Sensors
  // EXAMPLE. Return The pin representing the digital output connector configured to be this motor's positive limit, or CLEARCORE_PIN_INVALID if no such connector has been configured.

    //if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
        //// M-0's negative limit switch is now set to IO-2 and enabled.
    //} // Returns: True if the negative limit switch was successfully set and enabled
    //if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_INVALID)) {
        //// M-0's negative limit switch is now disabled.
    //} // Returns: True if the negative limit switch was successfully disabled; false if a pin other than CLEARCORE_PIN_INVALID was supplied that isn't a valid digital input pin.

    //ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0);
    // port I/O-0 = NegLimit for X-axis (motor 0)
    /*
    if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0)) {
        // M-0's negative limit switch is now set to IO-0 and enabled.
        Serial.println("I/O-0 was successfully set for Negative Sensor for ConnectorM0");	
    }
    // port I/O-1 = PosLimit for X-axis (motor 0)
    if (ConnectorM0.LimitSwitchPos(CLEARCORE_PIN_IO1)) {
        // M-0's positive limit switch is now set to IO-1 and enabled.
        Serial.println("I/O-1 was successfully set for Positive Sensor for ConnectorM0");
    }
    */
    // port I/O-2 = NegLimit for Z-axis (motor 1)
    if (ConnectorM1.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
        // M-1's negative limit switch is now set to IO-2 and enabled.
        Serial.println("I/O-2 was successfully set for Negative Sensor for ConnectorM1");	
    }
    // port I/0-3 = PosLimit for Z-axis (motor 1)
    if (ConnectorM1.LimitSwitchPos(CLEARCORE_PIN_IO3)) {
        // M-1's positive limit switch is now set to IO-3 and enabled.
        Serial.println("I/O-3 was successfully set for Positive Sensor for ConnectorM1");
    }

   // here we need to set 2 sensors for motor_1

} // END of setup() loop



void loop() {    // Put your main code here, it will run repeatedly:
    /************************ Reading Motor Register ************************/
    MotorDriver *motor_0 = &ConnectorM0; // X-axis
    MotorDriver *motor_1 = &ConnectorM1; // Z-axis
    // A variable should be declared volatile whenever its value can be changed by something beyond the control of the code section in which it appears

    // motor_0 is instance of MotorDriver, motor0 is just "ConnectorM0" #define
    // scope of statusReg_0 and statusReg_1 is only the main loop()
    volatile const MotorDriver::StatusRegMotor &statusReg_0 = motor_0->StatusReg();
    volatile const MotorDriver::StatusRegMotor &statusReg_1 = motor_1->StatusReg();

    //  uint32_t ClearCore::MotorDriver::StatusRegMotor::InNegativeLimit
    //  when we register hitting the InNegativeLimit, we send a message once
    if ((statusReg_0.bit.InNegativeLimit == 1) && (X_NegLimitFlag == 0) ) {
        Serial.print("InNegativeLimit:  ");
        Serial.println(statusReg_0.bit.InNegativeLimit);
        X_NegLimitFlag = 1;
        // temp-4
        Serial.print("X_NegLimitFlag:  "); 
        Serial.println(X_NegLimitFlag);

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nHit Negative Limit Sensor on axis X");
        Udp.endPacket();        
    }
    if ((statusReg_0.bit.InNegativeLimit == 0) && (X_NegLimitFlag == 1) ) {
        X_NegLimitFlag = 0;
        // temp-4
        Serial.print("X_NegLimitFlag: "); 
        Serial.println(X_NegLimitFlag);
    }

    // This block tells us when a motor is moving ("Moving") or is not moving ("Ready")
    /*if (statusReg.bit.ReadyState == MotorDriver::MOTOR_MOVING) {
        Serial.println("Motor is Moving");
    }*/
    /*Serial.print("Ready state:   ");
    switch (statusReg.bit.ReadyState) {
        case MotorDriver::MOTOR_DISABLED:
            Serial.println("Disabled");
            break;
        case MotorDriver::MOTOR_ENABLING:
            Serial.println("Enabling");
            break;
        case MotorDriver::MOTOR_FAULTED:
            Serial.println("Faulted");
            break;
        case MotorDriver::MOTOR_READY:
            Serial.println("Ready");
            break;
        case MotorDriver::MOTOR_MOVING:
            Serial.println("Moving");
            break;
        default:
            // something has gone wrong if this is printed
            Serial.println("??? (Undefined)");
    }*/
    /************************ Reading Motor Register ************************/


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

        Serial.print("Remote port: ");
        Serial.println(Udp.remotePort());

        // Read the packet.
        int bytesRead = Udp.read(packetReceived, MAX_PACKET_LENGTH);
        Serial.print("Packet contents: ");
        // Normally if you are sending text to be read by humans, you would use
        // Serial.print(). When sending raw data to be received by a machine, 
        // you would normally use Serial.write().
        Serial.write(packetReceived, bytesRead); // Serial.write(buf, len)
        Serial.println();
        
        /* // for debug purposes
        // Here we do motor control
        Serial.print("packetReceived[0] and packetReceived[1]: ");
        Serial.print(packetReceived[0]);
        Serial.print(packetReceived[1]);
        // int32_t dist is signed (can have negative value)
        int32_t dist = atol(packetReceived);
        Serial.print("\natol dist value: ");
        Serial.print(dist);
        Serial.println(); */

        int32_t dist_X = 0;
        int32_t dist_Z = 0;
        
        int i = 0;
        while (i <= bytesRead) {
		        //Serial.println("Waiting for HLFB to assert on both motors");
            // example: Serial.println((String)"x:"+x+" y:"+y);
            Serial.println((String)"packetReceived[" + i + "]: " + packetReceived[i]);
            
            // retrieving X distance:
            if ( (packetReceived[i] == 'X') && (packetReceived[i+1] == ':') ) {
                Serial.print("Received 'X:'");
                int t = 0 ;  
                char temp_str[bytesRead - i];
                while ( (packetReceived[t + i + 2] != ' ') && (packetReceived[t + i + 2] != '\0')) {
                    temp_str[t] = packetReceived[t + i + 2];
                    //Serial.print("\nvalue of t: "); Serial.print(t);
                    t++;
                }
                temp_str[t] = '\0';
                Serial.print("\nFull X string: ");
                Serial.write(temp_str, strlen(temp_str)); // !!! pay attention: Serial.WRITE()
                Serial.println();
                //Serial.println((String)"temp_str:" + temp_str);
                dist_X = atol(temp_str);
            } 

            if ( (packetReceived[i] == 'Z') && (packetReceived[i+1] == ':') ) {
                Serial.print("Received 'Z:'");
                int t = 0 ;  
                char temp_str[bytesRead - i];
                //while (packetReceived[t + i + 2] != '\0') {
                while ( (packetReceived[t + i + 2] != ' ') && (packetReceived[t + i + 2] != '\0') ) {
                    temp_str[t] = packetReceived[t + i + 2];
                    //Serial.print("\nvalue of t: "); Serial.print(t);
                    t++;
                }
                temp_str[t] = '\0';
                Serial.print("\nFull Z string: ");
                Serial.write(temp_str, strlen(temp_str)); // !!! pay attention: Serial.WRITE()
                Serial.println();
                //Serial.println((String)"temp_str:" + temp_str);
                dist_Z = atol(temp_str);
            } 
           
            i++;    
	      }
        // for debug purposes
        //Serial.println((String)"Integer dist_X: " + dist_X);
        //Serial.println((String)"Integer dist_Z: " + dist_Z);

        // Sending back reply packet back to the sender.
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("Received X:");
        //char dist_char[8];                      // these two line is just 
        //Udp.write(itoa(dist, dist_char, 10) );  // int-to-char conversion
        char X_dist_char[8];                      // these two line is just 
        Udp.write(itoa(dist_X, X_dist_char, 10) );  // int-to-char convertion
        Udp.write(" Received Z:");
        char Z_dist_char[8];                      // these two line is just 
        Udp.write(itoa(dist_Z, Z_dist_char, 10) );  // int-to-char conversion
        Udp.endPacket();
        
        if ( (dist_X == 999) && (dist_Z == 999) ) {
            Homing_Z_axis();
        } else {
            motor_0_MoveDistance(dist_X);
            motor_1_MoveDistance(dist_Z);
        }   
    }

    delay(10);    
}  // end of loop()




// Motor Control functions
/*------------------------------------------------------------------------------
 * MoveDistance
 *
 *    Command "distance" number of step pulses away from the current position
 *    Prints the move status to the USB serial port
 *    Returns when HLFB asserts (indicating the motor has reached the commanded
 *    position)
 *
 * Parameters:
 *    int distance  - The distance, in step pulses, to move
 *
 * Returns: True/False depending on whether the move was successfully triggered.
 */
/*bool MoveDistance(int distance) {
    // Check if a motor alert is currently preventing motion
	// Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");		
		PrintAlerts();
		if(HANDLE_ALERTS){
			HandleAlerts();
		} else {
			Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Move canceled.");		
		Serial.println();
        return false;
    }

    Serial.print("Moving distance: ");
    Serial.println(distance);

    // Command the move of incremental distance
    motor.Move(distance);

    // Waits for HLFB to assert (signaling the move has successfully completed)
    Serial.println("Moving.. Waiting for HLFB");
    while ( (!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) &&
			!motor.StatusReg().bit.AlertsPresent) {
        continue;
    }
	// Check if motor alert occurred during move
	// Clear alert if configured to do so 
    if (motor.StatusReg().bit.AlertsPresent) {
		Serial.println("Motor alert detected.");		
		PrintAlerts();
		if(HANDLE_ALERTS){
			HandleAlerts();
		} else {
			Serial.println("Enable automatic fault handling by setting HANDLE_ALERTS to 1.");
		}
		Serial.println("Motion may not have completed as expected. Proceed with caution.");
		Serial.println();
		return false;
    } else {
		Serial.println("Move Done");
		return true;
	}
}
//------------------------------------------------------------------------------
*/

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


/*------------------------------------------------------------------------------
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
	if(motor0.AlertReg().bit.MotionCanceledInAlert){
		Serial.println("    MotionCanceledInAlert "); }
	if(motor0.AlertReg().bit.MotionCanceledPositiveLimit){
		Serial.println("    MotionCanceledPositiveLimit "); }
	if(motor0.AlertReg().bit.MotionCanceledNegativeLimit){
		Serial.println("    MotionCanceledNegativeLimit "); }
	if(motor0.AlertReg().bit.MotionCanceledSensorEStop){
		Serial.println("    MotionCanceledSensorEStop "); }
	if(motor0.AlertReg().bit.MotionCanceledMotorDisabled){
		Serial.println("    MotionCanceledMotorDisabled "); }
	if(motor0.AlertReg().bit.MotorFaulted){
		Serial.println("    MotorFaulted ");
	}
 }
//------------------------------------------------------------------------------


/*------------------------------------------------------------------------------
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
 void motor_0_HandleAlerts(){
	if(motor0.AlertReg().bit.MotorFaulted){
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
//------------------------------------------------------------------------------

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


/*------------------------------------------------------------------------------
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
	if(motor1.AlertReg().bit.MotionCanceledInAlert){
		Serial.println("    MotionCanceledInAlert "); }
	if(motor1.AlertReg().bit.MotionCanceledPositiveLimit){
		Serial.println("    MotionCanceledPositiveLimit "); }
	if(motor1.AlertReg().bit.MotionCanceledNegativeLimit){
		Serial.println("    MotionCanceledNegativeLimit "); }
	if(motor1.AlertReg().bit.MotionCanceledSensorEStop){
		Serial.println("    MotionCanceledSensorEStop "); }
	if(motor1.AlertReg().bit.MotionCanceledMotorDisabled){
		Serial.println("    MotionCanceledMotorDisabled "); }
	if(motor1.AlertReg().bit.MotorFaulted){
		Serial.println("    MotorFaulted ");
	}
 }
//------------------------------------------------------------------------------


/*------------------------------------------------------------------------------
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
 void motor_1_HandleAlerts(){
	if(motor1.AlertReg().bit.MotorFaulted){
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
//------------------------------------------------------------------------------

//--------------------------- Motor 1 End --------------------------------------



//if( motor1.AlertReg().bit.MotorFaulted == 1)
// Possible input are 2 cases: 1. NegSensor is active, 2. NegSensor isn't active
int Homing_Z_axis() {
    Serial.print("Beginning of function Homing_Z_axis(). Z_HomingDoneFlag = ");
    Serial.println(Z_HomingDoneFlag);

    ////motor1.VelMax(Homing_velocityLimit); // slow down to 1Homing_velocityLimit

    while (Z_HomingDoneFlag != 1 ) {
        if (motor1.StatusReg().bit.InNegativeLimit == 1) {  // 2 cases: NO_Flag or Flag
            // temp-5
            Serial.print("motor1.StatusReg().bit.InNegativeLimit:  "); 
            Serial.println(motor1.StatusReg().bit.InNegativeLimit);

            if (Z_Homing_Flag == 0) {     // NO_Flag case: 
                Z_Homing_Flag = 1;// set Flag=1, we just starting going positive dir
                Serial.println("Set Z_Homing_Flag = 1"); 
                // temp-5
                Serial.println("Z_Homing_Flag:  "); 
                Serial.println(Z_Homing_Flag);
            }
            else if (Z_Homing_Flag == 1) { 
                // we're starting or continuing to go positive direction
                // one step in positive direction
                // temp-5
                Serial.print("Z_Homing_Flag:  "); 
                Serial.println(Z_Homing_Flag);

                Serial.println("Moving one step to positive direction"); 
                motor_1_MoveDistance(homing_step);
            }
        }
        else if (Z_Homing_Flag == 1) {// NegLimit == 0 // cases: Flag or NO_Flag
            // temp-5
            Serial.print("motor1.StatusReg().bit.InNegativeLimit:  "); 
            Serial.println(motor1.StatusReg().bit.InNegativeLimit);

            Z_Homing_Flag = 0; //we are done
            // temp-5
            Serial.print("Z_Homing_Flag:  "); 
            Serial.println(Z_Homing_Flag);
            Serial.println("Z-axis: Homing is done");
            Z_HomingDoneFlag = 1;
        }
        else { // NO_Flag, we need to move negative direction
            // temp-5
            Serial.print("Z_Homing_Flag:  "); 
            Serial.println(Z_Homing_Flag);
            Serial.println("We need to move negative direction, step size:");
            Serial.println(-homing_step);

            motor_1_MoveDistance(-homing_step); // go 1 step i negative direction
        }
        delay(1); // This delay is very important: wihout it a ClearCore is slow with reading the Sensor state inside the while() loop. Removing this delay causes doubling of distance from Sensor to Home position.
    }
    
    Z_HomingDoneFlag = 0;
    
    ////motor1.VelMax(velocityLimit); // set velocity back to normal velocityLimit
    
    Serial.print("End of function Homing_Z_axis(). Z_HomingDoneFlag = ");
    Serial.println(Z_HomingDoneFlag);
}



