# Manual Movement
This repository is designed to handle anything regarding the rover's drivetrain or arm system. 


## Movement Scripts

### Controller:

```bash
ros2 run manual_movement controller
```

#### Script: controller.py
#### Published Topics: 
/movement/Controller

#### Published Messages: 
std_msgs/Int32MultiArray

#### Description:
This script takes the Y-Axis of both Xbox Controller joysticks.

Currently, it publishes all axes and buttons for the controller, but only the first two elements have been properly tested. It publishes numbers in the input range of [-32768, 32768] where -32768 is a joystick pushed fully forward

The goal of this script is to make sure the controller's state is accurately and quickly published with the ROS Node. When developing, there were some issues with overloading the ROS Node with the amount of data being published which causes the data to have extreme latency. The solution was to implement an incremented counter which only publishes every X attempts.

This uses the inputs package, and doesn't properly use the ROS timer. Improvements are to switch to using PyGame and set the timer to 50Hz. Although the incrementing counter idea works, it is an unreliable implementation.

### Joystick:

```bash
ros2 run manual_movement joystick
```

#### Script: joystick.py 

#### Published Topics: 
/movement/joystick

#### Published Messages: 
std_msgs/Float32MultiArray

#### Description:

This script takes all axes, 2 buttons, and 1 hat from the joystick.

This script proved to be reliable during competition. There are occasional latency issues which have happened, but it seems to be a problem on the network or the teensy, as the publish rate does not fall when it happens.

### Rover Movement:

```bash
ros2 run manual_movement rover_movement
```

#### Script: neo_teensy.py

#### Subscribed Topics: 
/movement/Controller

#### Subscribed Messages: 
std_msgs/Int32MultiArray

#### Description:
This script communicates with the drivetrain teensy which talks to the motors. The range of values which can be sent is a float of [-40.0, 40.0]. The left side's values have to be negated to go forward

Example Message: '-Motor1,-Motor2,-Motor3,Motor4,Motor5,Motor6'

### Arm Movement:

```bash
ros2 run manual_movement arm
```

#### Script: arm_subscriber.py

#### Subscribed Topics: 
/movement/joystick

#### Subscribed Messages: 
std_msgs/Float32MultiArray

#### Description:
This script communicates with the arm teensy which talks to the arm's motors. I wouldn't focus too much on this code becuase the arm teensy is going to be refactored very soon. 

### Encoders:

```bash
ros2 run manual_movement encoder
```

#### Script: encoders.py

#### Published Topics: 
/movement/Encoder

#### Published Messages: 
std_msgs/Float64MultiArray

#### Description:

This script communicates with the drivetrain teensy and grabs encoder values. 

Because the gearboxes are 48:1, the change in the encoders is extremely small. This is why the published message is a Float64MultiArray, and also why the chosen units are inches and not meters. When doing meters, the change is so small, that it simplifies to 0.

There was an issue where the encoder script stops working if a single movement command is sent, but I don't remember what the solution is. It is somewhere in the autonomous code and it is a very simple solution.

### Autonomous Movement

```bash
ros2 run manual_movement autonomous
```

#### Script: autonomous_movement.py
#### Subscribed Topics:
/odom  
/odom/target_point  
/map/grid_raw

#### Subscribed Messages:
nav_msgs/Odometry  
geometry_msgs/PoseStamped  
nav_msgs/OccupancyGrid

#### Description:

This code uses a finite state machine with switch cases in python. It only satisfies the first mission of autonomous which provides an accurate GNSS location.

An important improvement which can be made is swapping to a client/service architecture with the target point and cost map. This will allow for only generating the cost map when it is needed. It will also solve the problem of a low refresh rate for the cost map. 

The asyncronous subscriber calls compounds with the inherently low refresh rate of the cost map publisher requiring the rover to wait 2 seconds to get an updated cost map. 

Upgrading to an Orin AGX and doing this swap should fix the problem.

