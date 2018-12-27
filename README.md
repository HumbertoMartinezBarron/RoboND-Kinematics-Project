# Robotic Arm: Pick and Place.
By: Humberto MartinezBarron

## Parts of the Project :)

1. Writeup (duh!)
2. Kinematic Analysis
3. Project implementation

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/hi.jpg)

## Part 1.
### Writeup!

### Right here!
Hey there! In this writeup, I'll go through the steps of the project and the way I solved it! :)

## Part 2: Kinematic Analysis!

### Deriving DH's parameters.

- So it turns out I'm a terrible artist. However, I managed to draw the Kuka arm and, using the URDF file, the Forward Kinematics class, and the ```forward_kinematics.launch``` file, I was able to derive the parameters needed to build the DH table.

- That's my notebook right there! Sorry about the handwriting, messiness, etc... XD

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG_20170928_004445.jpg)

## Creating transformation matrices

In order to obtain each of the T's, we need to multiply the rotation matrices and the translation matrices. Thus, we obtain a general T for each i. Now we just need to substitute the values for each joint! :)

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG_20170928_011537.jpg)

So, at the end, the product of all the individual homogenous transforms gives us the total homogenous transform between the ```base_link``` and the ```gripper_link```.

If we wish to obtain its matrix using only the end-effector poses, we would get the total transformation matrix as shown here!

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG_20170928_102448.jpg)

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG_20170928_102454.jpg)

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG_20170928_102506.jpg)

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG_20170928_102526.jpg)

## Decoupling IK!

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/wal_e.jpg)

So, since all robotic arms are different and we need to consider our Kuka arm's configuration in order to figure out the angles for a given EE position and orientation, we are solving two separate problems:

- First three angles (__thetas__ 0, 1, and 2 in the code!).
- Last three angles (__thetas__ 3, 4, and 5 in the code!).

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/IMG-20170925-WA0009.jpg)

In the drawing, we get to see the Cosine Laws in action! After assigning sides a, b, and c, all that's left is to find the angles of the triangle. However, we must be careful! These are NOT the thetas we are looking for (believe me, it will make your errors skyrocket and you will never reach the goal!).

So now, we need to subtract these angles to __pi__/2 in order to get thetas 1 and 2.

Theta0 is the easiest to calculate, since a simple ```atan2``` will do the trick!

## Explanation on how I got thetas 1 and 2.

So if you are very curious and wanna go more into detail about how I obtained these thetas, I'll tell you here!

For theta1, we needed the WC position in x and y. The session goes into how to obtain these, but in short, we can use the total rotation matrix between the base link and the gripper link to determine where it is. The WC is always at a total distance of 0.303 meters from the gripper. Therefore, all we need to do now is subtract that distance (which is affected by the orientation) from the EE position.

The equations for the WC are given as follows:

- Wx = px - 0.303 * nx
- Wy = py - 0.303 * ny
- Wz = pz - 0.303 * nz

Where px, py, and pz are the EE coordinates and nx, ny, and nz are the values corresponding to the translation of the EE's orientation.

Now that we have the WC, we can calculate the following:

- Distance between the WC and joint 2 (our side c).
- Angles __xi__ 1, 2, and 3 (the angles in the triangle formed by the WC, joint 2, and joint 3) using the Cosine Laws.

Thus, the formulas for all three __xi__ angles are:

- xi_1 = arccos((a^2 + c^2 - b^2) / (2ac))
- xi_2 = arccos((a^2 + b^2 - c^2) / (2ab))
- xi_3 = arccos((b^2 + c^2 - a^2) / (2bc))

Now, the thetas 1 and 2 are given by subtracting these angles from __pi__/2 (in case of theta1, we also need to subtract the angle given by the arctangent of the difference in z between joint 2 and the WC divided by the distance between the WC and joint 2 measured along the xy plane).

Now that we finally have the first three thetas, we need - you guessed it - the last three thetas!

Noticing we already have five thetas and that the last three control the orientation of the EE, rather than the location of the WC (which is determined by the first three thetas), we now can have equations for each of these angles!

By multiplying the final transformation matrix that gives us the position of the WC, we are left with only 3 variables!

So, in a single line, our equation to extract the values of thetas 3, 4, and 5 is given by:

- S = T0_3.evalf(subs={theta0: new_theta0, theta1: new_theta1, theta2: new_theta2}).inv("LU") * Rrpy

Where S is the matrix that represents the rotation matrix from joint 4 to joint 6.

Thus, the third column will give us the thetas we need by performing the following operations:

- theta3 = arctan(S[2,2]/-S[0,2])
- theta4 = arctan(sqrt(S[0,2]^2+S[2,2]^2)/S[1,2])
- theta5 = arctan(-S[1,1]/S[1,0])

## Part 3: Project Implementation!

Now, all we have left is to fill in the code of ```IK_server.py```!

First, I filled in the FK code, making a matrix for each of the joints, which was probably a mistake because I could easily have created a single matrix and substitute different values depending on the joints it was transforming. Oh well...

I filled in the DH, and then performed multiplication to find the final transformation matrix (```T_final```)!

For IK, I used roll, pitch, and yaw to figure out the difference in orientation between the EE and the ```base_link```, used that result to find the wrist center, and then used that position to find thetas 0, 1, and 2. Finally, I used the total transformation matrix to derive the equations of the last thetas, using the values of the first three!

So in the implementation explained, I got terrible results! My error curves ranged approximately from 3.67e-5 to 0.5 units!

As it turns out, after trying several horrible things that never worked (mostly trying to minimize the error using previous poses, considering the fifth joint's angle to determine the sign of the fourth and sixth, etc) I found in the slack community that someone had used the transpose of the matrix instead of the inverse!

I had read in the presentation for the class "Mathematics for Inverse Kinematics" by Ming Yao, that in this IK case, the transpose is equivalent to the inverse. I decided to try it and... BOOM! All the errors are < 3.67e-5 units!

## Improvements and mess-ups.

Still, there are some particular poses that will make the server return imaginary numbers! I think this is due to one of two things:

- The poses are unreachable
- The arm __believes__ they are unreachable, given they are on the very edge of the navigable space, making the slightest error (even one as small as 3/67e-5) to result in a pose that is farther away than the arm can reach.

Either way, I have been trying to figure out how to handle this for a while now, but can't come up with anything. Some ideas include:

- Determining whether the pose makes the code return imaginary numbers, and it it does, use the previous angle configuration for the last three angles.
- Decreasing the error further (but HOW?! :( ).

## Finally...

This project was awesome! I think it is the best application of math I have used to date! The simulator was SO cool and I felt like Tony Stark every time I launched it and a bunch of windows opened performing awesome-looking code to show me a robot that I could program to make it move! How amazing is that!

It was not easy, but this project was eye-opening for me to the world of robotics!

# Thanks! :)

!(https://github.com/HumbertoMartinezBarron/RoboND-Kinematics-Project/blob/master/imgs/tony_stark.jpg)