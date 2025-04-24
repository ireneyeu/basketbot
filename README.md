Works just like the homeworks.

Need to create bin and build directories:
mkdir bin build

Then inside build need to compile:
cd build

cmake .. && make -j4

Then you can run the files from bin folder:
cd bin/basketbot

 ./basketbot-simviz

 Added an initial script for the robot to move up and down like in the tutorial. First run ./basketbot-simviz and in another terminal the robot_arm_up_down.py file


Notes:
Added a sphere and floor to the world.urdf and the simviz_config.xml files. "collisionRestitutionCoefficient" lets the ball bounce.
