# Cheesy Vision 2.0

A program to track the angle of the scale using a webcam.

## Basic usage

1.  Opening `run_dashboard.bat` launches `CheesyVision2.py` and `firstperson.html` (see [First Person View](#first-person-view)).
2.  Press <kbd>D</kbd> to cycle through cameras until the webcam is the active one.
3.  Click and drag to select the rectangular region-of-interest.
    - The tighter the rectangle, the more accurate it will be; however, make sure that the entire range of the scale is included.
    - If you'd like to change the RoI later, simply press <kbd>R</kbd> and drag a new rectangle.
4.  Click on the scale arm to select the purple color.
    - You can click multiple times or drag the mouse to select multiple regions.
    - Press <kbd>F</kbd> to freeze the video feed and make selection easier.
    - If you misclick or make a mistake, press <kbd>C</kbd> to reset the color selection. You can also right-click to reset and then add the clicked color.
5.  Zero the scale by pressing <kbd>Z</kbd>.
    - Make sure you wait for a moment when the scale is horizontal and the program sees the scale well. Press <kbd>Z</kbd> again at any time to re-zero.

Then the program's ready.

## Full controls

#### In the raw camera feed view:

 - left-click and drag: select the region-of-interest (RoI)
 - <kbd>D</kbd>: switch cameras

#### In the region-of-interest view:

 - **left-click** (and optionally drag): add color(s)
 - **right-click**: reset and add color
 - <kbd>C</kbd>: reset color selection
 - <kbd>F</kbd>: freeze the video
 - <kbd>R</kbd>: go back to re-select the RoI
 - <kbd>Z</kbd>: zero the scale measurement
 - <kbd>M</kbd>: switch to manual color mode (default)
 - <kbd>A</kbd>: switch to auto-color mode (the color range is automatically detected each frame)
 
 ## Command-line arguments
 
 You can see the list of command-line arguments by running:
 
 ```$ python CheesyVision2.py --help```

## First Person View
- `firstperson.html` displays a MJPEG feed from the First Person View Camera on the robot
- Feed is streamed over Network Tables using the WPILib `MjpegServer` class