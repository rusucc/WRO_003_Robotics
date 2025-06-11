# WRO_003_Robotics

# Table of contents
# Meet the team
## Alex
## Cosmin
## Andrei
# Introduction

	Welcome to the engineering portfolio for our team's entry into the World Robot Olympiad (WRO) 2025 Future Engineers category. This document pre our journey in designing, building, and programming an autonomous vehicle to tackle the"Self-Driving Car Challenge." The prospect of this competition was both exciting and daunting. We knew that creating a car capable of navigating the course correctly and executing a perfect parallel park would be a great challenge.
	Our approach was shaped by some constraints, presented in the documentation. This portfolio details our entire engineering process, from the initial brainstorming sessions and chassis choice to the integration of our hardware.
	As you'll see in our system architecture diagrams, we chose a Raspberry Pi 4B as the central brain, interfacing it with a camera for computer vision, an IMU for orientation, and distance sensors for precise navigation. The development of our control logic, illustrated in our software flowcharts, was a significant challenge, requiring us to translate complex rules into robust, efficient code that could react in real-time.

	We embraced the WRO's emphasis on thorough documentation, and we hope this portfolio clearly communicates our passion, our process, and our commitment to growing as future engineers.
	We really wish to continue thoroughly docummenting our projects in the future.
# Considered constraints
	Our design and development process was guided by the official WRO 2025 Future Engineers rules. The most critical constraints that shaped our vehicle are:

- **Dimensions & Weight**: The vehicle must not exceed **300 mm x 200 mm x 300 mm** (Length x Width x Height) and a maximum weight of **1.5 kg**.
    
- **Drive System**: The vehicle must be a **4-wheeled vehicle** with one driving axle and one steering actuator. Critically, differential wheeled robots
    
- **Control System**: The vehicle must be **fully autonomous** during a run, with no wireless communication of any kind. Closed loop control must be implemented to ensure proper functionality
    
- **Startup Procedure**: The vehicle must be switched on with a single switch and then wait for a single start button press to begin its run.
- 
- **Perception**: The vehicle must be able to detect the walls of the field and track the position of green and red obstacles. The parking spot should also be properly detected.
- 
- **Documentation**: A significant portion of the score comes from comprehensive documentation, including an engineering journal and a public GitHub repository with commented code and commit history.
# Vehicle description
	The design is based on a heavily modified prebuilt RC car. Necessary modifications were made. Sensing and control capabilities were added.
## Starting point

## Mechanical
	Having a solid starting point was essential for us. We used a prebuilt RC car chasis which was available. It was designed for very high speed, off-road capabilities and was also over the size constraints, so we made some modifications:
- Conversion to rear wheel drive (from 4WD) - the Ackermann steering approximation can cause slippage when high steering angles are necessary 
- new motor - high gear ration makes the car slow and easily controllable. The encoder on the motor gives us the possibility of velocity control.
- new servomotor - the motor has a small movement angle, to get better steering „precision”
- new structure for mounting electronics 
### Kinematics

In accordance with the competition rules, our vehicle utilizes a car-like steering system. Its motion is modeled using bicycle kinematics, which simplifies the complex dynamics of a four-wheeled vehicle into a model with a single front and rear wheel. This approach is effective for planning paths and controlling turns at the speeds required for this competition.
- ![[Pasted image 20250611100528.png]]

The key parameters of this model, as shown in the diagram, are:

- **L**: The wheelbase length, which is the distance between the front and rear axles. For our vehicle, L is 250mm.
- **δ (delta)**: The steering angle of the front wheel relative to the car's chassis. Our mechanical system allows for a maximum steering angle of about 25 degrees.
- **θ (theta)**: The orientation or heading angle of the vehicle, measured with respect to a fixed global coordinate frame.
- **ICR**: The Instantaneous Center of Rotation. This is the point around which the entire vehicle rotates at a given moment. The vehicle turns by rotating around this point.
- **R**: The turning radius, defined as the distance from the ICR to the center of the rear axle. mm
- **ω (omega)**: The yaw rate (θ dot ) radians/sec
- **(Xr, Yr)**: Position of the center of the vehicle's rear axle in a 2D plane.
- **v**: The linear velocity of the vehicle. mm/sec
### geometry
The geometry had to be modified:
- the shock absorbers were changed for rigid 3D printed linkages. Suspension was not needed for the current application and would introduce unwanted noise in the system. Additionally, the car was originally very close to the 200mm width limit, so we made it narrower by raising the suspension.
- proper alignment of the tires was ensured 
## Electronics

### Power management
The robot  powered by a 2S 2200 mAh **LiPo**. The battery output is regulated by a DC-DC converter, which supplies a stable 5V voltage to the Raspberry Pi SBPC.
The same battery also powers a DC motor driver that controls a single drive motor. This driver receives control signals directly from the Raspberry Pi, enabling the robot to move forward and backward. In addition, the Raspberry Pi controls a servo motor responsible for steering, allowing the robot to navigate its environment
<img src = "Poze/bat.jpg" width = "300" height = "600" style="transform: rotate(270deg);" >


### Sensor management


- A raspberry pi camera 2 wide is used because:
	- It was already available to us
	- Ease of use
	- Good viewing angle
	- Low distortion
	- previous experience
	- small, compact size
	- Link : https://www.raspberrypi.com/products/camera-module-v2/
- 
- two Time-of-Flight (ToF) 8x8 multizone ranging sensor with 90 degrees FoV where used because:
	- They were already available to us
	- They are an alternative to a LIDAR sensor, however:
		- cheaper
		- higher sampling rate
		- it has 2D capabilities (compared to a spinning LIDAR)
		- Link : https://www.st.com/en/imaging-and-photonics-solutions/vl53l7cx.html
- Motor encoder: a two-channel hall effect encoder, preinstalled on the back of the motor was used because:
	- It was already installed
	- It allows for velocity control
	- It allows for calculation of displacement
	- Link: [Pololu - 12V Motor with 64 CPR Encoder for 37D mm Metal Gearmotors (No Gearbox, Helical Pinion)](https://www.pololu.com/product/4750)
- MPU 6050 IMU was used because:
	- It was already available
	- We found an easy to use RPI Python library
	- It allows for future development of advanced positioning algorithms
#
## Software
### Mobility management
### Obstacle management
## Current issues
## Future improvements
