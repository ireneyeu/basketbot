Works just like the homeworks.

Need to create bin and build directories:
mkdir bin build

Then inside build need to compile:
cd build

cmake .. && make -j4

Then you can run the files from bin folder:
cd basketbot/bin/basketbot

 ./basketbot-simviz
 ./basketbot

To run vision:
Go to cd basketbot/optitrack/drivers/PythonClient and run
Python StreamData.py
Then, go to cd basketbot/computer_vision and run
Python ball_tracker_optitrack.py

Notes:
Added a sphere and floor to the world.urdf and the simviz_config.xml files. "collisionRestitutionCoefficient" lets the ball bounce.
