# Description for Firmware program v0.1 for BBv3.

29 Mar 2024

This is probably the most stable version, currently flashed on the semi-field setup at HFL.

Firmware program v0.1 is implemented as an Arduino sketch (.ino file) and is written in a C-based programming language. This program uses the UDP protocol for communication over Ethernet. 

The program waits for UDP packets with incoming commands. When a message is received, it performs the commands and sends a response as a UDP packet together with some debug serial-over-USB messages via a USB cable.


**Reading debug messages via connected USB cable**

Additionally, a user can connect their computer to ClearCore via a USB cable (serial-over-USB connection) to read debug messages and some extra information about what is going on at ClearCore. To be able to read these messages, connect ClearCore to the computer via a USB cable, in Arduino IDE (with an installed ClearCore wrapper for Arduino IDE), go to "Tools" -> "Teknic ClearCore board" and choose "Teknic ClearCore". Next, choose the COM port, connected to ClearCore: on the Arduino IDE go to "Tools" -> "Port" and choose the port with the comment "ClearCore". After that, the user can go to "Tools" -> "Serial Monitor" to see the window at the bottom of the Arduino IDE. This window will show all the serial output from ClearCore.
The user doesn't have to read the serial output. Neither reading of this is needed for the correct work of ClearCore. It's just an option to get more information about the work of the program, the work of ClearCore and the Ethernet communication with ClearCore.


**Setting up MAC Address**

Each ClearCore module has a unique MAC address, flashed on it by the producer. A MAC address is a 6 byte sequence like "24-15-10-b0-81-d0". Different MAC addresses for devices operating in the same network are needed to prevent traffic collisions. It is possible to use a random MAC address or something like "AA-BB-CC-DD-00-11". In this case, the program will still work and the ClearCore will be responding normally, but it will take slightly more time for the UDP packets to be recognized in ClearCore. That is why we should state in this program a real MAC-address of an actual ClearCore module we are going to use with the program.
To set the MAC address of your ClearCore module in the program, use the command:
```
byte mac[] = {0x24, 0x15, 0x10, 0xb0, 0x81, 0xd0};
```
Here, we set the ClearCore MAC address to "24-15-10-b0-81-d0".


**Setting IP-address and port number**

The ClearCore module should be set to an IP-address from the same subnetwork the BenchBot v3 system uses. To set the IP-address we use the command:
```
IPAddress ip(10, 80, 65, 55);
```
Here, we set IP-address 10.80.65.55.

The port number is by default the same on all the BenchBot systems; port number 8888.
We set the port number with the command:
```
unsigned int localPort = 8888;
```

**Functionality**

The program can independently move both motors for different distances and in different directions. In this version of the firmware program, it is expected that UDP messages for movements are sent to ClearCore with the delay needed to complete the movements. When a command to move "X:3000 Z:10000" is sent, it is expected that the next command to move (or the Homing command) will be sent after the amount of time needed to perform the move "X:3000 Z:10000".

The BenchBot v3 has four associated limit sensors: Negative and Positive for X-axis, Negative and Positive for Z-axis. For example, the X-axis is controlled by Motor0 and there are Negative and Positive sensors associated with Motor0 (i.e. associated with the X-axis).

These four limit sensors are controlled "automatically" by ClearCore at a lower level. They are not controlled by this program. But in this program we can read the state of each of the sensors (through the Registers).

When a limit sensor is reached, the associated motor immediately stops and this functionality is implemented in the low-level ClearCore firmware flashed to ClearCore by its producer. This functionality is already built-in in ClearCore and is not implemented in this program.

When on X-axis (and only on X-axis!), the carriage reaches the Negative or the Positive Limit sensor, the ClearCore sends a message about reaching the Limit sensor through Ethernet UDP together with sending a debug message over an optional USB cable connected to a computer. When the carriage moves away from a Limit sensor, the ClearCore also sends a serial debug message via a USB cable.
For example, if the Negative Limit sensor is reached, the program sends a UDP message: 
```
"\nHit Negative Limit Sensor on axis X"
```
together with a serial (over an optional USB cable) debug message: 
```
"InNegativeLimit:  "
```


**Sending messages to ClearCore over Ethernet**

In the BenchBot v3 system, ClearCore controls movements of the camera on X- and Z-axes. ClearCore is connected with limit sensors and with some other equipment (the "E-stop" system etc.). In a normal operation mode, UDP messages are sent over Ethernet to ClearCore by the "Brain" computing unit of the BenchBot v3.
Also, it is possible to send UDP messages through Ethernet directly from a computer. To do so, a user will need a computer with software for sending UDP messages. One of the options is to use the "netcat" program or the improved and more advanced analog, the "ncat".
The "netcat" traditionally works on Linux, but can be easily installed on a Mac using the "brew" software package management system.
To send a UDP message to ClearCore with the "netcat", the user needs to know the IP address of the ClearCore module and the port number. For example, the command to move on the X-axis for 1000 steps in positive direction and simultaneously to move on the Z-axis for 5000 in negative direction to a ClearCore module with IP address 10.95.76.21 and port 8888 will be:
```
nc -u 10.95.76.21 8888
```
Here "nc" stands for "netcat", "-u" stands for UDP, "10.95.76.21" is the IP address and 8888 is the port number.
After typing this command in a terminal, the user should press Enter and type the message you want to send to the ClearCore module. When the message is ready, pressing Enter again will initiate sending of the message.


**Message to move**

When the program receives a message to move, for example "X:1000 Z:3000", after parsing this message and deriving the distances, the program sends back a UDP message with the derived distances. In this way, the sending side can know what distances (numbers) the ClearCore program derived.
For example, when we send a message to ClearCore: 
```
X:1000 Z:5000
```
The ClearCore will send a UDP response:
```
Received X:1000 Received Z:5000
```
The same moment, the corresponding carriage movements start.

For the X-axis, the negative direction is LEFT, the positive direction is RIGHT.
For the Z-axis, the negative direction is DOWN, the positive direction is UP.

IMPORTANT: This verion of the program does not send a message when movements are finished.


**Homing logic: how to get to the "Home" position**

This version of the program has a homing function only on the Z-axis (the vertical axis), the "Home" is the highest position of the camera without activating the Positive Limit sensor on the Z-axis. To get this position, the camera should move up (positive direction) until it reaches and activates the Positive Limit sensor. The next step is to go down (negative direction) until the Positive Limit sensor becomes inactive. Homing is done when the camera has reached the upmost position with the INACTIVE Positive Limit sensor on the Z-axis. This position is called the "Home" position.


**Homing for Z-axis**

Implemented, tested and ready for use a homing function for the Z-axis. Homing or returning to "Home" is the movement of a camera to its initial position on an axis. For the Z-axis, the initial position is the upmost position of the camera. I.e. when the homing function for Z-axis is called, the camera will immediately attempt to move to its highest position.
To call the homing on the Z-axis function, send this message to ClearCore:
```
X:999 Z:999
```
Z-axis homing was implemented for testing purposes.


**List of functions with a brief description**

void setup()
An Arduino function, called immediately of turning ClearCore on and executes one every time the device is turned on.

void loop()
An Arduino function. The main function in the program. This function starts execution immediately after the setup() function and is executed in a loop as long as the ClearCore is turned on.

bool motor_0_MoveDistance(int distance)
This function takes a distance (negative or positive integer) and moves the camera on the X-axis.
As was mentioned before, Motor 0 is associated with the X-axis.
Positive movement directed RIGHT, negative movement directed LEFT.

void motor_0_PrintAlerts()
An original function by Teknic from examples of sketches for ClearCore. It prints alerts for the Motor 0 (X-axis) if any of them arise.

void motor_0_HandleAlerts()
An original function by Teknic from examples of sketches for ClearCore. It resolves alerts for Motor 0 (X-axis), when any of them arises.

bool motor_1_MoveDistance(int distance)
This function takes a distance (negative or positive integer) and moves the camera on the Z-axis.
As was mentioned before, Motor 1 is associated with the Z-axis.
Positive movement directed UP, negative movement directed DOWN.

void motor_1_PrintAlerts()
An original function by Teknic from examples of sketches for ClearCore. It prints alerts for the Motor 1 (Z-axis) if any of them arise.

void motor_1_HandleAlerts()
An original function by Teknic from examples of sketches for ClearCore. It resolves alerts for Motor 1 (Z-axis), when any of them arises.

int Homing_Z_axis()
This function performs homing for Z-axis. Was tested, works.


**ClearCore Documentation**

ClearCore is an industrial I/O and motion controller. Made by "Teknic", the producer of the ClearPath motors which are used in BenchBot v3 for X- and Z-axes.

Links for ClearCore documentation and manual:

```
ClearCore Documentation: https://teknic-inc.github.io/ClearCore-library/
ClearCore Manual: https://www.teknic.com/files/downloads/clearcore_user_manual.pdf
``` 
