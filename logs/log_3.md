## **In this session, I worked during 15 hours**
<br> <br>

## I created a supervisor that: -place the e-puck randomly on the field,  -put a random number (3 to 8) boxes on the field without collisions (with other boxes or e-puck) with random positions & orientations, -place a camera behind the robot to see where it is going (for now, it works only when the robots start at position (0,0) with an angle=0 because I did not use yet *Quaternions*)
<br><br>

## I added a transmitter for the Controller (E-puck) & receiver for the Supervisor so that they can communicate with one another. For now when the Controller is finished the Supervisor stop itself.
<br><br>

## I created a file "requirements.md" in the 'docs' folder so that there are information about how to create the "requirements.txt" file that contains all the needed Python libraries and their version used in this Project.

## I learned about Hypercomplex numbers & Quaternions to make the Camera work perfectly as I wanted. I had a hard time understanding how to convert the "SFRotation x y z theta" in a quaternion and understand how to make the Rotations in the Robot's perspective. The Camera is located in front of the Robot in the air (dx, dz offset values), looking at the Robot (PI rad rotation around "z" axis, -arctan(dz/dx) rad around "y" axis) in order to look at the ground in front of the Robot (to see if a collision is hapenning)