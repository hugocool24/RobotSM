# Robot SM 2018
![](../Downloads/IMG_20180503_143456.jpg)
## Overall architecture
We used an arduino with a motor shield. The motor shield had an external battery connected to it which powered up the arduino, motor, sensors and servo.
### The pitfalls we had with the architecture
Using an arduino uno we don't have any free slots for additional sensors since when using a motor shield two of the analog dignal ports gets used by the shield.

We first tried to control the arduino with an raspberry pi however it was hard to get the signalling working in a good way and also we were not able to power up the raspberry with an external battery.
### What was good with the architecture
The architecture was simple so the code for controlling the motor, servo, sensors are simple to use, it was just to read/write normally nothing special or fancy.
## Mechanical structure


Whereas the car could move quite fast, big turns were a problem, as well as elevations at the track due to its size and weight. For the next time, both the chassis and the top part would have to be decreased in size significantly in order to allow the car move around the track more easily. Another necessary thing to change would be to increase the distance between the bottom of the car and the ground since the car would occasionally get stuck moving up the elevation after scraping the ground with its bottom. 

For CAD models you can find them [here](https://github.com/hugosjoberg/RobotSM/blob/master/RobotCar.zip)

## Software

### Where to find the software

[https://github.com/hugosjoberg/RobotSM](https://github.com/hugosjoberg/RobotSM) in the 2018 folder using only the main.ino file in the end. For previous year code look in other folders.
### SW architecture

Overall we tried to write the software in functions so the code was cleaner and also for debugging purposes since arduino doesn't come with a debugger.

1. Initialize all the variables.
2. Start the input/output on the arduino
3. The void loop starts that controlls the car
    4. Check the start signal and kill switch from the start module
    5. Read in sensor values
    6. Find a target point to drive towards (The point furthest away from the sensors)
    7. Controller calculates how much we should steer towards the target point.
    8. Motor control decides on the speed and also controlls the motor (duh :) )
    9. Servo control steers the car toward the target point.
10. Repeat step 3


### What was working
Reading in all the sensors was working but it is important to follow the specification of the sensors if not the value they spit out will not make any sense. We also used a moving average to filter out some of the bad values the sensors gives us from time to time.

### What was not working
All the code worked perfectly until we reached servo control, that part mysteriously changed the value of the point that we wanted to drive towards randomly to 90 degrees.

### How to improve
Make the servo controll work better.

Implement a function that reverses the car when it has bumped into a wall.

Maybe try a different algo for steering and driving.
## Sensors

Three analog sensors were used in the car setup, located on the front of the car (one directly at the front and two others at small angles to the left and right from the first one). 

### Good

Using analog sensors instead of digital ones seems to be a good idea since they are faster in general, however filtering the data coming from them poses quite a challenge. 

### Bad

Multiple sensors were either broken or giving values obviously not representing the real situation at the track and therefore had to be changed. 

## Summary

### What can we do better next year

There are two main ideas for performance improvement for Robot SM 2019. First is to base the design on the old architecture with several important changes/improvements, and the second one is to drastically alter the architecture for (hopefully) better overall performance on track.

#### Old architecture

With some of the sensors failing completely and others not being too reliable major attention should be paid to purchase and testing of the new sensors and increasing the number of them installed on the car in order to improve its "location awareness". At the same time, the car should be smaller and lighter while having a larger distance between ground and its bottom in order to avoid situation where it would get stuck on some elevations. Finally, the car should get a special "reverse" function which would be used in case it gets too close to multiple blocks in its way. 

#### New architecture

The other idea is very time consuming but might pay off. It would include using camera instead of sensors and analysis of the images received with it by neural network. Since Arduino would probably lack the computational power necessary to run it, Raspberry Pi might be necessary to use in this car iteration. Neural network is to be designed in Python, perhaps using TensorFlow,  and then trained in order to be able to analyze situation on the track. One of potential improvements would be distinction between different objects on track and possibility to behave differently based on what object is in front of the car (opponent's car or wall, for example). 