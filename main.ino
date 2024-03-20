// 20 Mar 2024 ver. 0.12-adv-full an attempt to add all the code needed to the modified parser in order to get a working program.

// 12 Feb 2024 ver. 0.12-adv : This works. Modified parser.

// An attempt to clarify the situation with Positive/Negative direction on X-axis, also Z-axis. Checking everything from scratch. 
// This works (ver. 0.10): sending back only one feedback message when we send to ClearCore a movement for 2 motors in one command, like "X:1000 Z:3000".
// ip address of ClearCore is now 10.95.76.21

// 12 Jan 2024 ver. 0.08 : an attempt to implement Homing on X and Z axises
// Homing logic. On Z axis "Home" is the leftmost position without activating NegLimit sensor. We go left (negative direction) until we hit NegLimit sensor. Next we go right (positive direction) until NegLimit sensor becomes not active. Done when reached the leftmost position with NO active NegLimit sensor. Home position is reached.  
// On Z-axis, "Home" position is when the carriage is in the upmost position.

// To control two motors, please, send a UDP packet to 10.95.76.21, port 8888.
// in Linux can use netcat: "nc -u 10.95.76.21 8888"
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


// old Mac addr !!! Insert MAC-addr of an actual ClearCore unit
//byte mac[] = {0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01};

// !!! Test this  
//byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x33, 0x65};  // First BenchBot
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x31, 0xc0};  // Second BenchBot
IPAddress ip(10, 95, 76, 21);  //10.95.76.21

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
#define motor0 ConnectorM0  // X-axis
#define motor1 ConnectorM1  // Z-axis

// Select the baud rate to match the target serial device
#define baudRate 9600

// This example has built-in functionality to automatically clear motor alerts, 
//  including motor shutdowns. Any uncleared alert will cancel and disallow motion.
// WARNING: enabling automatic alert handling will clear alerts immediately when encountered and return a motor to a state in which motion is allowed. Before enabling this functionality, be sure to understand this behavior and ensure your system will not enter an unsafe state. 
// To enable automatic alert handling, #define HANDLE_ALERTS (1)
// To disable automatic alert handling, #define HANDLE_ALERTS (0)
#define HANDLE_ALERTS (1) // in original sketch HANDLE_ALERTS was (0)

// Define the velocity and acceleration limits to be used for each move
// speed of 10000 is too high for Z-axis motor (Z-axis motor takes up to 5000 pulses per sec) 
// 3000 is too high for X axis
int velocityLimit = 3000; // pulses per sec, 5000 is max for vertical Z-axis
//int X_velocityLimit = 1000;
//int Z_velocityLimit = 3000;

////int Homing_velocityLimit = 500;
int accelerationLimit = 30000; //50000;// 100000; // pulses per sec^2

// 20 -> 18mm distance after homing
int homing_step = 10; // Size of a step for Homing function. Was 10, was 5. 

// ClearCore has ARM Cortex-M4 32 bit processor. 32 bit Signed Integer can house a number from −2,147,483,648 to 2,147,483,647
int32_t dist_X = 0;
int32_t dist_Z = 0;

int X_axis_negative_is_left_Flag = 1; // 1 when we can see rear side of Motor
                                      // 0 when we can see shaft of Motor

int X_NegLimitFlag = 0;  // flag = 1 while NegLimit sensor is reached
int X_PosLimitFlag = 0;
int Z_NegLimitFlag = 0;
int Z_PosLimitFlag = 0;

int X_MovingFlag = 0;
int Z_MovingFlag = 0;

int CarrMovingFlag = 0;

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

  // --------- end of Motor Control block ---------

  // Setting Negative and Positive end Sensors for X and Z axises
  // EXAMPLE. Return The pin representing the digital output connector configured to be this motor's positive limit, or CLEARCORE_PIN_INVALID if no such connector has been configured.
    //if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
        //// M-0's negative limit switch is now set to IO-2 and enabled.
    //} // Returns: True if the negative limit switch was successfully set and enabled
    //if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_INVALID)) {
        //// M-0's negative limit switch is now disabled.
    //} // Returns: True if the negative limit switch was successfully disabled; false if a pin other than CLEARCORE_PIN_INVALID was supplied that isn't a valid digital input pin.
    //ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0);

    // port I/O-0 = NegLimit for X-axis (motor 0)
    if (ConnectorM0.LimitSwitchNeg(CLEARCORE_PIN_IO0)) {
        // M-0's negative limit switch is now set to IO-0 and enabled.
        Serial.println("I/O-0 is now set to Negative Sensor for ConnectorM0 and enabled");	
    }
    // port I/O-1 = PosLimit for X-axis (motor 0)
    if (ConnectorM0.LimitSwitchPos(CLEARCORE_PIN_IO1)) {
        // M-0's positive limit switch is now set to IO-1 and enabled.
        Serial.println("I/O-1 is now set to Positive Sensor for ConnectorM0 and enabled");
    }
    // port I/O-2 = NegLimit for Z-axis (motor 1)
    if (ConnectorM1.LimitSwitchNeg(CLEARCORE_PIN_IO2)) {
        // M-1's negative limit switch is now set to IO-2 and enabled.
        Serial.println("I/O-2 is now set to Negative Sensor for ConnectorM1 and enabled");	
    }
    // port I/0-3 = PosLimit for Z-axis (motor 1)
    if (ConnectorM1.LimitSwitchPos(CLEARCORE_PIN_IO3)) {
        // M-1's positive limit switch is now set to IO-3 and enabled.
        Serial.println("I/O-3 is now set to Positive Sensor for ConnectorM1 and enabled");
    }
  delay(500); // this delay is needed to give time to ClearCore for assigning proper values to Registers (it could be 250+ ms)

} // END of setup() loop



void loop() {    // Put your main code here, it will run repeatedly:
    /************************ Reading Motor Register ************************/
    // Can we move these two lines to setup loop ? - check in the last turn !
    MotorDriver *motor_0 = &ConnectorM0; // X-axis
    MotorDriver *motor_1 = &ConnectorM1; // Z-axis
    // A variable should be declared volatile whenever its value can be changed by something beyond the control of the code section in which it appears

    //Serial.print("AtTargetPosition:  ");
    //Serial.println(motor1.StatusReg().bit.AtTargetPosition);

    //int32_t dist_X = 0;
    //int32_t dist_Z = 0;

// If movement for only one axis is received, we use the current code (2 if-else statements)
// When we have a movement for two axises, we go another way (possibly, a Flag) 

// This block peforms sending back of a packet when all the movementes are done.
// if any of two motors is moving, or both are moving
if ( ( (motor0.StatusReg().bit.AtTargetPosition == 0) || (motor1.StatusReg().bit.AtTargetPosition == 0) ) && (CarrMovingFlag == 0) ) {
  CarrMovingFlag = 1;
  // some movement has started
  Serial.print("Some movement has started, CarrMovingFlag = ");
  Serial.println(CarrMovingFlag);
} 
// when something was moving, but now both motors stopped
if ( (motor0.StatusReg().bit.AtTargetPosition == 1) && (motor1.StatusReg().bit.AtTargetPosition == 1) && (CarrMovingFlag == 1) ) {
  CarrMovingFlag = 0;
  // send an UDP message that any movement finished
  Serial.print("All movements have finished, CarrMovingFlag = ");
  Serial.println(CarrMovingFlag);

  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  if (dist_X == 0) { 
    // movement was ONLY on Z-axis
    Serial.print("Movement was only on Z-axis");
    // send UDP message
    Udp.write("\nAll movements have finished, Z-axis covered distance = ");
    char Z_axis_char[8];                        // these two line is just 
    Udp.write(itoa(dist_Z, Z_axis_char, 10) );  // int-to-char convertion
  } 
  else { // movement was on both X-axis and Z-axis OR only on X-axis
    if (dist_X != 0 && dist_Z != 0) {
      // movement was on both X-axis and Z-axis
      Serial.print("Movement was on both X-axis and Z-axis");
      // send UDP message
      Udp.write("\nAll movements have finished, X-axis covered distance = ");
      char X_axis_char[8];                        // these two line is just 
      Udp.write(itoa(dist_X, X_axis_char, 10) );  // int-to-char convertion
      Udp.write(", Z-axis covered distance = ");
      char Z_axis_char[8];                        // these two line is just 
      Udp.write(itoa(dist_Z, Z_axis_char, 10) );  // int-to-char convertion
    } else { 
      // movement was ONLY on X-axis
      Serial.print("Movement was only on X-axis");
      // send UDP message
      Udp.write("\nAll movements have finished, X-axis covered distance = ");
      char X_axis_char[8];                        // these two line is just 
      Udp.write(itoa(dist_X, X_axis_char, 10) );  // int-to-char convertion
    }
  }
  
  Udp.endPacket();
}

    
// -------------- Processing of Limit Sensors encountering ---------------------
    //  X-axis: when hit the InNegativeLimit Sensor, send a message once
    if ((motor0.StatusReg().bit.InNegativeLimit == 1) && (X_NegLimitFlag == 0)) {
        Serial.print("InNegativeLimit:  ");
        Serial.println(motor0.StatusReg().bit.InNegativeLimit);
        X_NegLimitFlag = 1;
        Serial.print("X_NegLimitFlag:  "); 
        Serial.println(X_NegLimitFlag);
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Negative Limit Sensor on axis X");
        Udp.endPacket();        
    } // setting Flag back to 0 when NegativeLimit Sensor is not active any more
    if ((motor0.StatusReg().bit.InNegativeLimit == 0) && (X_NegLimitFlag == 1)) {
        X_NegLimitFlag = 0;
        Serial.print("We moved out from the Negative Limit Sensor");
        Serial.print("X_NegLimitFlag: "); 
        Serial.println(X_NegLimitFlag);
    }

    //  X-axis: when hit the InPositiveLimit Sensor, send a message once
    if ((motor0.StatusReg().bit.InPositiveLimit == 1) && (X_PosLimitFlag == 0)) {
        Serial.print("InPositiveLimit:  ");
        Serial.println(motor0.StatusReg().bit.InPositiveLimit);
        X_PosLimitFlag = 1;
        Serial.print("X_PosLimitFlag:  "); 
        Serial.println(X_PosLimitFlag);

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Positive Limit Sensor on axis X");
        Udp.endPacket();        
    } // setting Flag back to 0 when PositiveLimit Sensor is not active any more
    if ((motor0.StatusReg().bit.InPositiveLimit == 0) && (X_PosLimitFlag == 1)) {
        X_PosLimitFlag = 0;
        Serial.print("We moved out from the Positive Limit Sensor");
        Serial.print("X_PosLimitFlag: "); 
        Serial.println(X_PosLimitFlag);
    }


    //  Z-axis: when hit the InNegativeLimit Sensor, send a message once
    if ((motor1.StatusReg().bit.InNegativeLimit == 1) && (Z_NegLimitFlag == 0)) {
        Serial.print("InNegativeLimit:  ");
        Serial.println(motor1.StatusReg().bit.InNegativeLimit);
        Z_NegLimitFlag = 1;
        Serial.print("Z_NegLimitFlag:  "); 
        Serial.println(Z_NegLimitFlag);
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Negative Limit Sensor on axis Z");
        Udp.endPacket();        
    } // setting Flag back to 0 when NegativeLimit Sensor is not active any more
    if ((motor1.StatusReg().bit.InNegativeLimit == 0) && (Z_NegLimitFlag == 1)) {
        Z_NegLimitFlag = 0;
        Serial.print("We moved out from the Negative Limit Sensor");
        Serial.print("Z_NegLimitFlag: "); 
        Serial.println(Z_NegLimitFlag);
    }

    //  Z-axis: when hit the InPositiveLimit Sensor, send a message once
    if ((motor1.StatusReg().bit.InPositiveLimit == 1) && (Z_PosLimitFlag == 0)) {
        Serial.print("InPositiveLimit:  ");
        Serial.println(motor1.StatusReg().bit.InPositiveLimit);
        Z_PosLimitFlag = 1;
        Serial.print("Z_PosLimitFlag:  "); 
        Serial.println(Z_PosLimitFlag);

        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write("\nWARNING! Hit Positive Limit Sensor on axis Z");
        Udp.endPacket();        
    } // setting Flag back to 0 when PositiveLimit Sensor is not active any more
    if ((motor1.StatusReg().bit.InPositiveLimit == 0) && (Z_PosLimitFlag == 1)) {
        Z_PosLimitFlag = 0;
        Serial.print("We moved out from the Positive Limit Sensor");
        Serial.print("Z_PosLimitFlag: "); 
        Serial.println(Z_PosLimitFlag);
    }

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

                
        //int i = 0;
        char return_message[300] = ""; // max size message to be sent back
                           
        // 1-20 bytes is the valid size received packet
        if ( (0 < bytesRead) && (bytesRead < 21) ) {  
            // CONFIG, E-STOP, STATUS, Home:X or Home:Z, X:50 and Z:50
            // if packetReceived[0-5] equal to "CONFIG"   
            if (memcmp(&packetReceived[0], "CONFIG", 6) == 0) {
                Serial.print("memcmp: CONFIG");
                //Serial.print("\n");
                //Serial.print("initial value of return_message: ");
                //Serial.print(return_message);
                strcpy(return_message,"UDP: CONFIG message received");

            // if packetReceived[0-5] equal to "E-STOP"
            } else if (memcmp(&packetReceived[0], "E-STOP", 6) == 0) {
                Serial.print("memcmp: E-STOP");
                strcpy(return_message,"UDP: E-STOP message received");
                //Serial.print(return_message);

            // if packetReceived[0-5] equal to "STATUS"
            } else if (memcmp(&packetReceived[0], "STATUS", 6) == 0) {
                Serial.print("memcmp: STATUS");
                strcpy(return_message,"UDP: STATUS message received");
                //Serial.print(return_message);

            // if packetReceived[0-7] equal to "CONFIG"
            } else if (memcmp(&packetReceived[0], "HOME:X,Z", 8) == 0) {
                // "HOME:X,z" will be interpreted as "HOME:X"
                Serial.println("memcmp: HOME:X,Z");
                strcpy(return_message,"UDP: HOME:X,Z message received");
                // Home:X,Z function

            // if packetReceived[0-5] equal to "HOME:X"
            } else if (memcmp(&packetReceived[0], "HOME:X", 6) == 0) {
                Serial.println("memcmp: HOME:X");
                strcpy(return_message,"UDP: HOME:X message received");
                // Home:X function

            // if packetReceived[0-5] equal to "HOME:Z"
            } else if (memcmp(&packetReceived[0], "HOME:Z", 6) == 0) {
                Serial.println("memcmp: HOME:Z");
                strcpy(return_message,"UDP: HOME:Z message received");
                // Home:Z function
                // Homing_Z_axis();

            // retrieving X distance:   //int t = 0 ;
            } else if ( (packetReceived[0] == 'X') && (packetReceived[1] == ':') ) {
                Serial.println("Received 'X:'");
                int t = 0 ;
                char temp_str_X[10] = ""; // 8 positions to save an axis distance
                while ( (packetReceived[2 + t] != ' ') && (isDigit(packetReceived[2 + t])) ) { // 2 is a shift for 'X:'
                    temp_str_X[t] = packetReceived[t + 2];
                    //Serial.println((String)"value of packetReceived[" + (t+2) + "]:" + packetReceived[t+2]);
                    t++;
                }
                temp_str_X[t] = '\0'; // termination for temp-string
                Serial.print("Full X string: ");
                Serial.write(temp_str_X, strlen(temp_str_X)); // !!! pay attention for Serial.write !
                Serial.println();
                // ! possible checking if only digits are in temp_str
                dist_X = atol(temp_str_X);
                Serial.print("long int value of dist_X: ");
                Serial.print(dist_X);
                // X-axis move function:
                //motor_0_MoveDistance(dist_X);
                               
                Serial.println((String)"\nvalue of packetReceived[" + (t+2) + "]:" + packetReceived[t+2]); // this value should be space ' '
                t = t + 3; //shift for 3 positions: for X, for : and for for ' '
                // now t points to the next position after the space ' '
                if ((packetReceived[t]=='Z') && (packetReceived[t+1]==':')) {
                    Serial.println("Received 'Z:'");
                    char temp_str_Z[10] = "";
                    t++; // step from Z to :
                    t++; // step from : to first digit
                    int i = 0;
                    while (packetReceived[t] != '\0') { // !! single quotes only
                        Serial.println((String)"value of t: " + t + "   value of i: " + i);
                        if (isDigit(packetReceived[t])) {//tests if char is a digit
                            temp_str_Z[i] = packetReceived[t];
                            Serial.println((String)"value of packetReceived[" + t + "]:" + packetReceived[t]);
                            Serial.println((String)"value of temp_str_Z[" + i + "]:" + temp_str_Z[i]);
                            //Serial.println("The character is a number");
                        }
                        t++;
                        i++;
                    }
                    temp_str_Z[i-1] = '\0';
                    Serial.print("\nFull Z string: ");
                    Serial.write(temp_str_Z, strlen(temp_str_Z)); // !!! pay attention: Serial.WRITE()
                    dist_Z = atol(temp_str_Z);
                    // 2. Test this                
                    Serial.print("\nlong int value of dist_Z: ");
                    Serial.print(dist_Z);
                    // Z-axis move function:
                    // motor_1_MoveDistance(dist_Z);
                } // closing if (Z:)

                // construct return_message, composite of words and numbers 
                Serial.print("\n");
                Serial.print("initial value of return_message: ");
                Serial.print(return_message);

                strcat(return_message, "Received X:");
                char dist_X_char[8]; 
                itoa(dist_X, dist_X_char, 10);
                    
                //strcat(text_line, itoa(dist_X, X_dist_char, 10));
                //strcpy(result, s1);
                strcat(return_message, dist_X_char);
                strcat(return_message, ", Z:");
                char dist_Z_char[8] ; 
                itoa(dist_Z, dist_Z_char, 10);
                strcat(return_message, dist_Z_char);

                Serial.print("\n");
                Serial.print("Result: ");
                Serial.print(return_message);

            } else { //if all previous conditions are false
                Serial.println("Received message was not recognized");
                strcpy(return_message,"UDP: Received message was not recognized. Possible messages are: 'CONFIG', 'E-STOP', 'STATUS', 'HOME:X', 'HOME:Z', 'HOME:X,Z' and 'X:[int number] Z:[int number]'\n");
            }
            
        }  // closing if ( (0 < bytesRead) && (bytesRead < 21) )

        // for debug purposes
        //Serial.println((String)"Integer dist_X: " + dist_X);
        //Serial.println((String)"Integer dist_Z: " + dist_Z);

        // Sending a reply packet back to the sender
        Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
        Udp.write(return_message);
        Udp.endPacket();
          
    } // closing if (packetSize > 0)

    delay(10);    
}  // end of loop()


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
//--------------------------- Motor 1 End --------------------------------------

//if( motor1.AlertReg().bit.MotorFaulted == 1)
// Possible input are 2 cases: 1. NegSensor is active, 2. NegSensor isn't active
int Homing_Z_axis() {
    Serial.print("Beginning of function Homing_Z_axis(). Z_HomingDoneFlag = ");
    Serial.println(Z_HomingDoneFlag);
    //motor1.VelMax(Homing_velocityLimit); //slow down to 1Homing_velocityLimit
    while (Z_HomingDoneFlag != 1 ) {
        if (motor1.StatusReg().bit.InNegativeLimit == 1) {  // 2 cases: NO_Flag or Flag
            
            Serial.print("motor1.StatusReg().bit.InNegativeLimit:  "); // temp-5
            Serial.println(motor1.StatusReg().bit.InNegativeLimit);    // temp-5

            if (Z_Homing_Flag == 0) {     // NO_Flag case: 
                Z_Homing_Flag = 1;// set Flag=1, we just starting going positive dir
                Serial.println("Set Z_Homing_Flag = 1"); 
                Serial.println("Z_Homing_Flag:  "); // temp-5
                Serial.println(Z_Homing_Flag);      // temp-5
            }
            else if (Z_Homing_Flag == 1) { 
                // we're starting or continuing to go positive direction
                // one step in positive direction
                Serial.print("Z_Homing_Flag:  ");  // temp-5
                Serial.println(Z_Homing_Flag);     // temp-5

                Serial.println("Moving one step to positive direction"); 
                motor_1_MoveDistance(homing_step);
            }
        }
        else if (Z_Homing_Flag == 1) {// NegLimit == 0 // cases: Flag or NO_Flag
            
            Serial.print("motor1.StatusReg().bit.InNegativeLimit:  "); // temp-5
            Serial.println(motor1.StatusReg().bit.InNegativeLimit);    // temp-5

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
        delay(1); // This delay is very important: wihout it a ClearCore is slow with reading the Sensor state inside the while() loop. Removing this delay causes increase (around doubling) of distance from Sensor to Home position.
    }
    
    Z_HomingDoneFlag = 0;
    
    ////motor1.VelMax(velocityLimit); // set velocity back to normal velocityLimit
    
    Serial.print("End of function Homing_Z_axis(). Z_HomingDoneFlag = ");
    Serial.println(Z_HomingDoneFlag);
} 



