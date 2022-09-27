# advanced-robot-control
 Testing and example code for FRC team 3341's Advanced Robot Control module
## SYNOPSIS OF CURRICULUM
### Week 1
Veteran (>=1 year) students taking the Advanced module will refresh their basic systems control skills, starting off with WPILib's SysID tool and feedforward implementation. Most of the effort here is to copy this basic example code and to characterize both of the flywheels in a limited amount of time, similar to the environment during the FRC Competition Season.
### Week 2
Students will learn how to add on-the-fly Shuffleboard inputs to their existing PIDF code, and how to tune and test the PID controller. The intent is that they spend time to look at the ShuffleBoard graphs of the PIDF in action, versus just a basic idea of PID.
### Week 3
Students will apply their knowledge about WPILib PIDF from Weeks 1 and 2, in order to implement PIDF for a new gravity arm based system, the pivot of 3341's shooter from the 2021-2022 season. This involves not only characterizing the pivot, but also tuning the PID as well, during the limited Saturday session time allocated.
### Week 4
Using Motion Magic, students will learn how to obtain constants such as CruiseVelocity, maxHorizontalVoltage, and the kF, and will learn how to tune the PID controller of a new approach to systems control. CTRE Motion Magic seems more convenient than WPILib's implementation, as it is more responsive, and more fine-tunable.