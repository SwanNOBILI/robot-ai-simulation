## **In this session, I worked during 3 hours**
<br> <br>

## I downloaded the "Webots" application and tried a simple Robot named "e-puck" from "gctronic". The default simulation is working well. The world is a simple square with 4 boxes (with fixed positions). The Robot is moving forward using 2 actuators that are "left wheel" and "right wheel" and dodges the boxes or the square borders with 8 visual Sensors.
<br> <br>

## To continue, I modified the project architecture. Indeed, I didn't know anything about "Webots" and I took the architecture from this application. I updated the project architecture in the "readme.md".
<br> <br>

## Moreover, I added the correct files in the folder. I asked help from an AI Assistant because I had some errors when importing manually. Indeed, the simulation works by loading textures or important parts (as Sensors for example) from the github repository or the "Webots" local file. I needed to modify some of the files (as ".proto") to change this dependencies and add locally the needed dependencies & textures.
<br> <br>

## Finally, I made a new "basic_controller" in 'Python'. In fact, the default one is located in the local "Webots" file and is a compiled 'C' file (so a ".exe"). But I needed to make my own controller in 'Python' and change the dependency of my project simulation so that "basic_controller" is used instead of the default 'C' controller.<br>
### Before going into the controller programming, I created an environment for my project. Indeed, it ensures that nothing will break in the future (due to libraries update).<br>
<br>