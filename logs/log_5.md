## **In this session, I worked during 12 hours**
<br> <br>

## I added a GPS & Compass to my E-puck so that the controller knows the position & north orientation (in the Robot's frame)
<br> <br>

## I started to do the basic algorithm logic to reach the goal without collisions. I noticed that the sensors were not detecting obstacles from far away, so there are two solutions: -make the Robot speed slower; -extend the sensors range. I chose to reduce the Robot speed in order to work with realistic sensors.
<br> <br>

## I inspired my self from the "controller.c" code used for the e-puck in the original version. I added some features to make it more realistic (like smooth velocity change for example) and it works well most of the time. It uses Braitenberg coefficients, that I modified a bit to make it work better in my world. Braitenberg vehicles tend to shake left & right side when moving so I needed to modify a little bit my camera (in "supervisor.py"). Indeed, my camera was following the exact orientation of my Robot, so it was shaking left & right indefinitely (I added a threshold to modify the camera orientation).
<br> <br>

## I added my own WoddenBox (myWoodenBox) in the proto and slightly modified the original one (that I downloaded) to make the Boxes transparent (so that we can see through them) and modify the color (to black for now). I added some features in 'supervisor.py' as for example a goal that spawns "far" from the initial position or random box sizes (for each axis, so we do not necessarily have cubes anymore). I also modified a bit the spawn position of the several boxes, to have a world with obstacles better distributed around the path (segment between start & goal position).
<br>