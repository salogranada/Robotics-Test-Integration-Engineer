Wheel Odometry
---

**Maintainer:** _[Eng. Davidson Daniel Rojas Cediel](https://www.linkedin.com/in/dadaroce/)_ \
**Mail:** _davidson@kiwibot.com_ \
**Kiwi Campus / AI & Robotics Team**

# Wheel Odometry
Wheel odometry stands for the calculation of the vehicle motion based on the information provided by wheel encoders which represent the wheel velocities (Directly related to the Robot motion).
For the simplicity of our system and controllers, our current chassis is represented as a differential mobile robot. This representation is widely used in robotics as it keeps things simple. 

Our wheel odometry calculation relies on the data provided by the motors (RPM for extracting the velocity and error for handling undesired data); however, we also use the IMU angular velocity to calculate a slip factor and hence improve the overall odometry.

## Environment variables
This node uses the following environment variables:

* **ODOMETRY_WHEEL_RADIUS** [double]: 0.074 m. Corresponds to the wheel radius.
* **ODOMETRY_CHASSIS_TRACK** [double]: 0.392 m. Corresponds to the wheel track distance.

## Technical implementation
### Robot's Kinematics
A brief description of the Robot's kinematics is provided in order to show how the Odometry calculation is performed in this node. The following picture shows the conventions for the kinematic model of the robot in a 2D plane.


![image](https://user-images.githubusercontent.com/49252525/105538211-1246b780-5cc1-11eb-939e-8a33de655515.png)


Considering the orthonormal global frame *I = [O; X<sub>O</sub>, Y<sub>O</sub>, Z<sub>O</sub>]* centered at point O and an orthonormal local frame fixed at the robot’s Center of Mass (COM or call it the Robot frame) *J = [COM; X<sub>COM</sub>, Y<sub>COM</sub>, Z<sub>COM</sub>]*.

Describing and denoting the position of the Robots COM relative to the origin of the global frame O as *R<sub>COM</sub>(t) = [X(t), Y (t), Z(t)] ∈ R<sup>3</sup>* , and the velocity of the Robot COM concerning the global frame O as *V<sub>COM</sub>(t) = [ Ẋ(t), Ẏ (t), Ż(t)]' ∈ R<sup>3</sup>.*

The rotation of the local frame fixed to the mobile robot J concerning the global frame can be represented by the roll, pitch and yaw angles; nevertheless, for the Kiwibot the plane of motion is considered only, which means, the Z-Coordinate of COM is constant, with this assumption the Roll and Pitch angles will be zero.

Under this assumption the state vector of independent generalized coordinates which describes the Robot's pose is *q(t) = [X(t), Y(t), θ(t)]* (With X and Y, and the orientation θ of the local coordinate frame concerning the inertial frame), similarly, its derivative which denotes the generalized velocities is *q̇(t) = [ Ẋ(t), Ẏ(t), dθ/d(t)]*. Hence the state vector which describes the Robot's linear and angular velocity can be represented as follows:


![image](https://user-images.githubusercontent.com/49252525/105549164-95bad580-5cce-11eb-9f4f-d365298b4f1d.png)


Where *V(t) = [V<sub>x</sub> (t), V<sub>y</sub>(t)]* (derivatives) represents the speed of the mobile robot in the local frame J and *ω(t) = dθ/d(t)* denotes the robot’s angular speed.

It is clear that no restriction for the planar movement of the robot has been introduced yet since it describes the free body planar kinematics; however, the robot cannot move in the direction Y freely, so the non-holonomic constraint (V<sub>y</sub> = 0) is included, i.e. under normal circumstances the robot won't slip laterally (Neither wheel can contribute to the sideways motion in the reference frame). Following this way the model assumes *V<sub>y</sub>(t)* through all the motion.

### Odometry Calculation
Odometry calculation is performed by blending the velocity of the wheel in RPM (Used to calculate the linear and angular velocity of the Kiwibot with direct kinematics) and the IMU data to get the Yaw (θ).

As an example of the linear velocity calculation for each wheel, see the expression below used for the frontal right wheel:

* **V<sub>FR</sub>** = (2 * PI * W<sub>radius</sub> * RPM) / 60.0

Hence after the calculation of the wheels speed, the SSMR is approximated to a DDMR with the following expression:

* **V<sub>right</sub>** = (V<sub>FR</sub> + V<sub>RR</sub>) / 2.0

* **V<sub>left</sub>** = (V<sub>FL</sub> + V<sub>RL</sub>) / 2.0

And so the calculation of the linear and angular velocity:

* **V<sub>X</sub>** = (V<sub>right</sub> + V<sub>left</sub>) / 2.0

* **ω** = (V<sub>right</sub> - V<sub>left</sub> ) / track

Finally, we can estimate the value of Ẋ and Ẏ and similarly X and Y by integrating the following terms:

* **Ẋ** = V<sub>x</sub> * Cos(θ)

* **Ẏ** = V<sub>x</sub> * Sin(θ)


And integrating this value we could get the actual `delta_X` and `delta_Y` positions.

* **delta_X** = (Ẋ * dt)

* **X** = delta_X + offset # What is the value of this offset?

In conclusion with the methodology explained above, we have a complete estimation of `q(t) = [X(t), Y(t), θ(t)]` and its derivative `q̇(t) = [ Ẋ(t), Ẏ(t), dθ/d(t)]`.
