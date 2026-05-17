## **In this session, I worked during 5 hours**
<br> <br>

## I relocated the "eval" file outside the "controllers" file. It now located in "robots/e-puck/eval/".
<br> <br>

## I modified "basic.py" to have better performances. Firstly I deleted the phases, I wanted a simple strategy for my controller (I kept solely the Braitenberg part). Moreover, the robot could reach the "goal" if the tolerance was big enough, but couldn't do it for a small tolerance. As a consequence, I added a final brake to the robot, so that it would brake when coming close to the "goal". It wasn't so easy as the tolerance needed to be adaptated to the current linear speed (if not, it was hard to know when to start to brake solely depending on the goal tolerance). I modified the loop logic to work with "forward_speed" & "roation_speed" instead of "desired_left_motor_speed" & "desired_right_motor_speed" as it was more adapted to my brake strategy.
<br>