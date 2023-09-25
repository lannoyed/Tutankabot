# Working of a Robot Motor with CAN Protocol

## Contents
1. [Introduction](#introduction)
2. [CAN Protocol Files](#can-protocol-files)
3. [Explanation of CAN Messages](#explanation-of-can-messages)
4. [Motor Control](#motor-control)

## Introduction
This document explains the working of a robot motor using the CAN (Controller Area Network) protocol.

## CAN Protocol Files
In this section, we list the important files related to the CAN protocol implementation:

- `can.cpp`: Implements CAN communication using C code.
- `hexa.cpp`: Converts double values to hexadecimal characters.
- `can_functions`: Combines the functionalities of the above two files for motor control via CAN.

## Explanation of CAN Messages
CAN messages are structured as follows:

cansend <device> <recipient ID>#<message>

For this project, communication is established with a motor driver equipped with the MCP2515 chip. 
The address used is 708, and the device is can0. 
Messages, which represent modifications of register values, follow this format:

- `<addr>`: Address of the register.
- `<mask>`: Bits to be modified within the register.
- `<value>`: Value whose bits, indicated by the mask, will replace the old values in the register.

Knowing this, all the work that is missing is only to read the datasheet to find the value that should be given to all parameters to obtain the desired motors' behavior.

## Examples:

### Led control

- To turn on the LED:
~~~~
cansend can0 708#1EFF40
~~~~

- To turn off the LED:
~~~~
cansend can0 708#1EFF00
~~~~

### Motor Control
To initialize a motor, use:
cansend can0 708#1CFF80
To adjust its speed (Duty cycle of the hashers), use: 
~~~~
cansend can0 708#25FFXX
~~~~
where `XX`is the duty cycle of the hasher (22 for zero speed, 00 for maximum negative speed, 44 or higher for maximum positive speed).

For the second motor:
- To initialize:
~~~~
cansend can0 708#1DFF80
~~~~

- To adjust its speed:
~~~~
cansend can0 708#26FFXX
~~~~

## Conclusion
This document should have introduced to the reader enough contend to be able to use the CAN protocol to control the motors. 
This priciple was implemented in the file Tutankabot/Code_final/Raspberry/cpp-project/Sources/CanCom
/canCom.cpp 

It should be noted that, the use of bash injection in the terminal is the easyest way to use the can protocol but certainely not the most efficient one.
A faster way (arround 100 times faster) is implemented in the file Code_final/Raspberry/cpp-project/Sources/CanCom/ctrlOut.cpp but this other way (completely controlled in c++) increase the complexity. 
For us this new communication protocol arrived too late in the prosses and leaded to incompatibilities with controler. It thus was discarded. 
But, if this improvement was deployed it would have highly increase the speed of the robot software main loop. This would have increase the trajectory stability of the robot and the performances in obstacle avoidance which would have enabled the robot to increase its travel speed. 

The improved code will not be explained to the reader as the objective of this GITHUB is to provide the keys to succed the project not to give everything on a golden plate ^^.








