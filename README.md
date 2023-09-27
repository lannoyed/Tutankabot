[![MIT License](https://img.shields.io/badge/license-MIT-blue.svg)](/LICENSE)
[![Python](https://img.shields.io/badge/Python-3.7.9-blue.svg)](https://www.python.org/downloads/release/python-385/)
[![C++](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)



# Tutankabot - Mechatronics Project


**Project Group: Group 4**
**Academic Year: 2021-2022**

## Overview

Welcome to the Tutankabot Mechatronics Project repository! This project focuses on the development of an autonomous robot, and this README provides an overview of the project's structure and key components.

## Project Structure

- **/Code_final/:** This directory contains the complete source code deployed on the Raspberry Pi and Arduino.

- **/Botfinal_Quartus/:** The source code used for flashing the De0-Nano can be found here.

- **/Report/:** Explore the 'Report' directory to access various reports that detail the robot's design and development process.

- **/communications_protocols/CAN/:** Inside this directory, you'll find a README file that provides basic information about the CAN (Controller Area Network) protocol and its implementation within the project. 

- **/Path_Planning/:** This directory includes a README file explaining the functionality and usage of the path planning system employed in the project.

- **/Other folder/:** These directories hold code snippets developed during the project's development stages before integration into the final codebase. These snippets may offer simpler insights into specific functionalities but might have undergone changes in the final version. 

REAME.md files should be preferred over README.txt files as the second ones were intended for internal communications.

## Getting Started

To get started with the Tutankabot project, refer to the relevant directories and documentation mentioned above. If you have any questions or need further assistance, please feel free to reach out to our project team members.

## Project tips 

### Groupe organisation 
As this project is bigger than any other university project, it is important to have a good organisation. Furthermore, a verification of task completion and quality is crucial. 

It is basic, but the decision of a leader is important. This leader will have to organise the group and make sure that the project is going in the right direction. This additional work is not negligible and should be taken into account when the group is formed and when task are distributed.

A tracking of task distribution and good realisation should be held by the leader. This will allow the leader to know the progress of the project and to help the group member that are struggling. It also avoids the situation where a group member is not doing his work and the other group member has to do it for him.

### Work load distribution

This project is divided in 3 parts: the mechanical part, the electronic part and the software part. It is important to distribute the work load equally between the group member. This distribution should be done in a way that allows the group member to work on the part that they are the most interested in.

An example of the distribution could be the following (this is just an example and the tasks are not exhaustive):

#### Q1
**Electrical part** ( 2 person ) 
- Design of the PCB 
- Power network 
- Communication protocol
- Sensor selection and integration
- Electrical material ordering

**Mechanical part** ( 2 or 3 person )
- Design of the robot
- Selection of all mechanical parts
- Design, modelisation and fabrication of the mechanical parts
- Layering of the robot
- Mechanical material ordering

**Software part** ( 1 person )
- Motor control
- Obstacle detection
- Robot localisation
- Trajectory following


#### Q2

**Electrical part** ( 2 person )
- PCB soldering
- PCB testing
- Robot assembly

**Mechanical part** ( 2 or 3 person )
- Robot assembly

**Software part** (4 person)
- Robot localisation
- Opponent detection
- Sensor signal processing (FPGA) and fusion (Kalman filter)
- Path planning
- Main loop (thread)
- Robot interaction with the environment
- FSM controller of the states dependent programs (Interaction with object, displacement of the robot, calibration mode ...)

Globally Q1 is more focused on the design of the robot and Q2 is more focused on the implementation of the robot. All components should be ordered during Q1 to be able to start the implementation of the robot during Q2. The group member would switch from there Q1 task to the software part as both the mechanical and electrical tasks in Q2 are relaxed.

Do not underestimate the time needed to assemble the robot and the massive amount of time needed to debug and tune the software part. At some point, using day and night teams could be a good idea to speed up the process.

### Code organisation

The code organisation is also very important. The code should be organised in a way that allows the group member to work on different parts of the code without having to worry about the other part. This organisation should be decided at the beginning of the project and should be respected by all the group member.

NEVER PUSH CODE THAT DOES NOT COMPILE OR THAT IS NOT WORKING PROPERLY. This is a very important rule that should be respected by all the group member. If you push code that does not compile or that is not working properly, you will make the work of the other group member more difficult.

### Code quality

The code quality is also very important. The code should be well commented and should be easy to read. The code should also be well organised and should be easy to understand.

If a part of the code contains short comings or assumptions, it should be noted and documented to avoid confusion and horrible debugging session.

Finally, the implementation of the code should take into account the real-time constraint on the project. The code should be as fast as possible. This means that the language used should be C or C++ and not python (even if I love this language). The main program should be thread based.  The local based algorithms should be favoured over the global based algorithms unless the global based algorithms gives substantial improvements in term of robot robustness and completeness.


### Robot design

The design of the robot is crucial as it define 2 things : 
- The potential of the robot
- The difficulty of the project

The robot should be designed in a way that allows the robot to be as fast as possible and as robust as possible. The robot should also be designed in a way that allows the group member to work on different parts of the robot without having to worry about the other part.

Think simple and stupid as much as possible. System if one degree of freedom that is composed of one step rarely fails. And make you save a lot of time in the implementation and debugging process.

Still, everything can not be so simple when looking for performance. In this case, increase the complexity of the system but make sure that the complexity is correctly assessed and that the system is well designed.

It is important to note that adding features later in the process is very difficult thus this design process is very important. Also, problems like overconstrained system, lack of space, lack of power, lack of money, should be taken into account during the design process as it could lead to failure of the project.

For example, a robot with overconstrain on its wheels could be unable to follow a trajectory. This makes all control algorithm useless.


## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

---

Thank you for your interest in Tutankabot! We hope you find this repository informative and useful for your understanding of our mechatronics project. If you have any feedback or suggestions for improvement, please don't hesitate to let us know.