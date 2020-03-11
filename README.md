# IDP

> Code and relevant data files for IDP.

Source code lives in `src` directory.

## Running

The code has been tested on Ubuntu but it is not known if it will work on other operating systems (probably not for non-Linux systems).

Both the Arduino and the remote computer need to be connected to a mobile hotspot (whose credentials are keyed in into `src.ino`). The IP addresses need to be found and set.

Run

```
python3 src/imaging.py
```

in the root folder, and power up the Arduino. Everything should work automatically after that.

Note: `imaging.py` will not exit itself so you need to kill it with `CTRL + C` after the robot has finished running.

## Notes

White lead is left motor, red lead is right motor.
