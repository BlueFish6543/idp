# IDP

> Code and relevant data files for IDP.

Source code lives in `src` directory.

## Navigation

* Line navigation is relatively simple, current plan is to use two IR emitters/receivers, one on either side of the robot. When robot is following line (line is white) both sides should record low intensity. When left side records high robot needs to turn left, and vice versa.

* After robot has crossed the tunnel, we need to keep track of the robot's position. Current plan is to use a fixed value for wheel rpm (we can measure this physically). Measure both left and right wheels separately in the program. From that we can compute `x`, `y` and `theta` and update at every timestep.

## Robot finding

* Use IR receiver to figure out which direction signal is coming from. Best to find a way to make the receiver as directional as possible. Rotate receiver to sweep areas. Once a signal is found, turn robot to face the signal and drive towards it.

* May have to program robot to sweep out locations in a predetermined pattern.
