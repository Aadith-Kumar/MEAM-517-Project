
MEAM 517 Project Readme!

:link: [Google Drive](https://drive.google.com/drive/folders/12vvI-4S0ICZvCfdP6TQElmgLZ1OQ7fW2?usp=sharing)


Node: Guys,I have set up a git ignore file to ignore all the build/ , log/ and install/ folders as we would want to build them in our own system and do not want other build executables pushed to the git!

Just printing stuff to terminal for test

build instructions:
1. Navigate to root of the worksopace
2. Run 

     `$ colcon build`

3. Test the two sample files, one in CPP and one in python with the ros2 run command

     `$ ros 2 run path_planning cpp_publisher`
      
     `$ ros 2 run path_planning python_subscriber.py`
