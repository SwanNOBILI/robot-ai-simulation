## **In this session, I worked during 4 hours**
<br> <br>

## I created in the 'docs' folder, a file named 'evaluation_criterias.md' in which I wrote the different criterias that I will use to evaluate the performance/quality of a controller. So that I will be able to compare several controllers (basic vs AI-based).
<br> <br>

## I added those criterias in an 'evaluate_controller.py' using a class that I can use in my controller (for now 'basic.py'). I used some basic maths and used the sensors value to compute the needed criterias. Some of those criterias are not so precise insofar as we don't have the material needed (for the "collisions_counter" for example, we don't have contact sensors that permit us to directly do that, as a consequence we use the proximity sensor values to estimate the number of collisions but it might be imprecise). A json file si saved in the 'eval' folder for each current configuration (the name of this file is made of the "path_efficiency").
<br>