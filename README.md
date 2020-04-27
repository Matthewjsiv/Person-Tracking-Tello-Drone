# Person-Tracking-Tello-Drone
Python script for using a Tello drone to track and follow a person. The Tello sends video to your computer, which detects key points of your body. It then sends motion commands, determined through basic PID control, back to the drone in order to keep you well in the frame.

**The code currently works but the PID definitely can be tuned better still. I am also working on adding more tracking behaviors.**

## video

## Requirements/Setup
### [TelloPy](https://github.com/hanyazou/TelloPy):
Allows you to easily send commands and receive video from the drone. Can be easily installed running `pip install tellopy`.

### [OpenCV-Python](https://pypi.org/project/opencv-python/):
If you receive an error related to Qt binaries that prevents the script from running, try installing the headless version instead with `pip install opencv-python-headless`.

### [Simple-PID](https://github.com/m-lundberg/simple-pid):
Basic PID controller - `pip install simple-pid`.

### [Tensorflow](https://www.tensorflow.org/install/pip):
This code works with Tensorflow 1.15. It might work with Tensorflow 2 but you would need to make some modifications both in my script and the posenet python code.

### [Posenet-Python](https://github.com/rwightman/posenet-python):
For pose detection, I use this python port (made by [rwightman](https://github.com/rwightman) of the Tensorflow.js Posenet models created by Google. The models and code are already included here so it should work as long as you have Tensorflow set up properly.

### Other:
There are a few other packages you might need to install if you don't have them already, such as keyboard, pygame, and av.

## Usage
To start the script, run `track_person.py` with admin privileges. The automatic control is on from the start, but you can toggle it as well as control the drone manually using the keyboard controls below:
* **T** - toggle automatic control
* **Space** - take off (it won't take off automatically)
* **L** - land
* **Q** - rotate counter clockwise
* **E** - rotate clockwise
* **D** - move right
* **A** - move left
* **W** - move forward
* **S** - move backward
* **R** - reset commands (hover/attempt to stay still)



