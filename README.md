# Description for Firmware program v0.1.1 for BBv3.

26 Mar 2024

This is probably the most stable version, currently flashed on the semi-field setup at HFL.

Firmware program v.0.1.1 is implemented as an Arduino sketch (.ino file) and is written in a C-based programming language. This program uses the UDP protocol for communication over the Ethernet. The program waits for UDP packets with incoming commands. When a message received, it performs the commands and sends a response as a UDP packet together with some debug serial-over-USB messages via a USB cable.


**Setting MAC Address**

Each ClearCore module has unique MAC address, flashed in it by the producer. A MAC address is a 6 byte sequence like "24-15-10-b0-31-c0". Different MAC addresses for devices operating in the same network needed to prevent traffic collisions. It is possible to use a random MAC address or something like "AA-BB-CC-DD-00-11". In this case program will still works and the ClearCore will be responding normally, but it takes slightly more time for the UDP packets to be recognized in ClearCore. That is why we should state in this program a real MAC-address of an actual ClearCore module, we are going to upload to program to.
To set the MAC address of your ClearCore module in the program, we use the line:
```
byte mac[] = {0xAA, 0xBB, 0xCC, 0x00, 0x00, 0x01};
```
Here, we set MAC address "24-15-10-b0-81-d0".


**Setting IP-address and Port number**
We should set to a ClearCore an IP-address from the same subnetwork the BenchBot v3 system uses. To set the IP-address we use the line:
```
IPAddress ip(10, 80, 65, 55);
```
Here, we set IP-address 10.80.65.55.

The Port number is by default the same on all the BenchBot systems, number 8888.
We set the port number in the line:
```
unsigned int localPort = 8888;
```

**Functionality**

The program can independently move both motors for different distances.
It is expected that UDP messages to move are sent to ClearCore with the delay, needed to complete the movements. When a command to move "X:3000 Z:10000" is sent, it is expected that the next command to move (or a Homing command) will be sent after amount of time neede to perform the move "X:3000 Z:10000".

BenchBot v3 has four associated limit sensors (or switches): Negative and Positive for X-axis, Negative and Positive for Z-axis. For example, the X-axis in controlled by Motor0 and there are Negative and Positive sensors, associated with the Motor0 (i.e. associated with the X-axis).

These four limit sensors are controlled "automatically" by ClearCore on a lower level, they are not controlled by this program. But in this program we can read the state of each of the sensors (through the Registers).

When a limit sensor is reached, the associated motor immediately stops and this functionality is implemented in the low-level ClearCore firmware flashed to ClearCore by it's producer. This fucntionality is already built-in in ClearCore and is not implemented in this program!.

When on X-axis (and only on X-axis!) a Negative Limit sensor is reached, the program sends a UDP message "\nHit Negative Limit Sensor on axis X" together with a serial (over an optional USB cable) debug message "InNegativeLimit:  ". When the carriage moves away from the reached sensor, the program send a serial debug message: "We moved out from the Negative Limit Sensor".
The same works for the Positive Limit sensor on X-axis. When this sensor is reached, the program sends a UDP message "\nHit Positive Limit Sensor on axis X" together with a serial debug message "InPositiveLimit:  ". When the carriage moves away from the sensor, the program sends a serial debug message "We moved out from the Positive Limit Sensor" over USB.


**Message to move**

When program receives message to move, for example "X:1000 Z:3000", after parsing this message and deriving the distances the program sends back a UDP message with the derived distances. In this way the sending side can know what distances (numbers) ClearCore program derived.
For example, when we send to ClearCore the message: 
```
X:1000 Z:5000
```
The ClearCore will send an UDP response:
```
Received X:1000 Received Z:5000
```
The same moment corresponding carriage movements start.

This program does not send a message when movements are finished.


**Homing for Z-axis**

Implemented, tested and ready for use a homing function for Z-axis.
Homing or returning home is a movement of a carriage to it's initial position on an axis. For Z-axis the initial position is the upmost position of the carrige. I.e. when homing function for Z-axis is called, the carriage will immediately attempt to move to it's upmost position.
To start homing on Z-axis, send to ClearCore message:
```
X:999 Z:999
```
Z-axis homing was implemented for testing purposes.
