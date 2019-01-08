# ROCO504: Advanced Robot Design
## Active Robot Fall-Absorption Mechanism
### Mason Carter, Luka Danilovic, Elliott White

This is a repository to hold all the files for the ROCO504 project.

#### Requisites
The NUCLEO-F746ZG code needs to be loaded into https://os.mbed.com/ online compiler for it to work. 

mbed-os.lib revision 4928:e1bea44 also needs to be added to the project. 

#### Project Assignment
For the assignment the group will need to design, implement and evaluate an advanced robot
design. Typical skills to be acquired/demonstrated include the use of CAD, FEA, 3D printing,
materials, mechanisms, robot programming and structural analysis. The project will enable the participents to
study, implement and test various aspects of robotics, including soft and exotic materials, novel
actuation principles, 3D-printable structures, and the interaction of a robot with its environment. 
A budget of £70 for the project is given.

#### Abstract
The purpose of this research project was to design and build a prototype robot to serve as a proof of concept for a Active Robot Fall-Absorption Mechanism. Large robots with a small foot-base will inevitably fall over, so we have designed a mechanism to dampen the fall of a robot and reduce any damage caused to it from the fall. Upon successful completion of the building of the mechanism and conducting tests, our findings indicate that with slight improvements, originally not possible due to budget constraints, this project would yield a beneficial mechanism capable of performing its task of dampening a fall of a much larger robot.

#### Introduction
As the trend of Robotics leans towards creating taller robots, more researchers are struggling with the task of trying to keep a robot with a small foot-base upright. The robot will, inevitably, fall over. This can cause massive damage to the robot, mostly consisting of mechanical damage to the links between joints, or servo damage. Until fall prevention becomes more effective, a simpler fall-damage-reduction system would be incredibly useful. 

We propose to design and create a robotic body and leg, for the provided budget of £70.00. The main body will consist of agonist-antagonist actuators and can detect when a robot has fallen past its self-balancing point. The leg will then reach out and use artificial tendons to reduce the falling speed of the body. This can be comparable to how a human might brace themselves when falling. The artificial tendons are used to store potential spring energy. 


### Design
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Renders/Main.png)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Renders/Side.png)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Renders/Main_with_leg.png)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Photos/MAIN_BODY.jpg)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Photos/IMG_20190102_153236.jpg)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Photos/IMG_20190105_160925.jpg)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Photos/IMG_20190105_160858.jpg)
![alt text](https://github.com/ElliWhite/ROCO504/blob/master/Photos/IMG_20190105_160833.jpg)

### Conclusion
Video: https://youtu.be/zlB8UxlkwFY

We have built a platform that, with more work, will be capable of demonstrating the use of agonist-antagonist joints and artificial soft tendons to absorb a fall of a robot. The software algorithms that are in place are capable of successfully executing a fall-reduction system, but it is the hardware that causes limitations. The motors do not have enough torque to slowly lower the body down to the floor, nor have a high enough RPM to tension the leg to a high enough degree before the body reaches the floor. 

The biggest improvement to be made would be changing the DC motors to a more powerful version. This would enable a faster actuation of the linear actuators and also allow the actuators to move when under heavy load and not stall. This change would also require a change to the motor driver IC to a part that would be able to handle the higher current.
A new pulley would have to be designed to stop the nylon-to-Fila-Flex adapter from slipping off during leg deployment. This would be a simple redesign and the modification that would need to be made would be to widen the pulley to the extent that there is no space between the pulley and the body for the adapter to fall into.

Another modification would be the leg pre-tensioning and release mechanism. The new control algorithm and mechanism would pre-tension both tendons at the same time regardless of which direction the body is falling. Then, upon detecting a fall, the mechanism would release one tendon and tighten the other one further. This would allow a quicker response time as there is no wait for one tendon to be tensioned and the other one un-tensioned during the time before a fall is detected.

The budget of £70 was a major limiting factor, and with a larger budget and with the previously-mentioned changes made, we believe the system would be successful in its ability to absorb the impact from a fall and not allow any damage to be caused to the robot it is attached to.
