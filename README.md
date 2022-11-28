
MEAM 517 Project Readme!

:link: [Google Drive](https://drive.google.com/drive/folders/12vvI-4S0ICZvCfdP6TQElmgLZ1OQ7fW2?usp=sharing)






So, to run the Gazebo Simulator, one path will need to be manually added.
After pulling the Repo, open the file: MEAM-517-Project/src/robot_spawner_pkg/src/robot_spawner_pkg/spawn_turtlebot.py

Change the below line :
```
sdf_file_path = "/home/aadith/Desktop/MEAM-517-Project/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model-1_4.sdf"
```

to reflect the path to whereever you have the REPO
```
sdf_file_path = "--ADD PATH TO REPO--/MEAM-517-Project/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model-1_4.sdf"
```

After pulling the Repo, open the file: MEAM-517-Project/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model-1_4.sdf

There are four instances of hardcoded path, change them to reflect your path!




Node: Guys,I have set up a git ignore file to ignore all the build/ , log/ and install/ folders as we would want to build them in our own system and do not want other build executables pushed to the git!



build instructions:
1. Navigate to root of the worksopace
2. Run 

     `$ colcon build`


To launch simulator with 2 turtlebots, Run:

```sh sim_launch.sh```

To Control robot with teleop:

``` sh teleop.sh <robot number> ```


     
     
     
