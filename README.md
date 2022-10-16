# Arduino I2C Robot

This is a C++ Arduino project for a rover type of robot that is meant to be commanded via I2C communication. It also provides scan status.
This project uses the L293D motor shield. Because of the current limitation of the L293D this program powers the same motor with two sets of outputs. It also makes use of one servo for steering, and one servo for panning the ping sensor. 

The power drain from the steering servo was an issue, so the motor shield board was modified so that a separate power supply could be used to drive the steering servo.

