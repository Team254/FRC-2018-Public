# FRC 2018

Team 254's 2018 FRC robot code for Lockdown. Lockdown's code is written in Java and is based off of WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.

## Setup Instructions

### General
- Clone this repo
- Run `./gradlew` to download gradle and needed FRC libraries
- Run `./gradlew tasks` to see available build options
- Enjoy!

#### Eclipse
- Run `./gradlew eclipse`
- Open Eclipse and go to "File > Open Projects" from "File System..."
- Set the import source to the `FRC-2018-Public` folder then click finish

#### IntelliJ
- Run `./gradlew idea`
- Open the `FRC-2018-Public.ipr` file with IntelliJ

### Building/Deploying to the Robot
- Run `./gradlew build` to build the code. Use the `--info` flag for more details
- Run `./gradlew deploy` to deploy to the robot in Terminal (Mac) or Powershell (Windows)

## Code Highlights
- Building with Gradle

	Instead of working with Ant, we used GradleRIO, which is a powerful Gradle plugin that allows us to build and deploy our code for FRC. It automatically fetches WPILib, CTRE Toolsuite, and other libraries, and is easier to use across different IDEs. 

- Path following with a nonlinear feedback controller and splines

	To control autonomous driving, the robot utilizes a [nonlinear feedback controller](src/main/java/com/team254/frc2018/planners/DriveMotionPlanner.java#L263) and drives paths constructed of [quintic Hermite splines](src/main/java/com/team254/lib/spline/QuinticHermiteSpline.java).

- Path generation and visualization via Java app

	Cheesy Path, a Java webapp, allows a user to quickly and easily create and visualize autonomous paths. It is located in the [`src/main/webapp`](src/main/webapp) directory and the [com.team254.path](src/main/java/com/team254/path) package.  Run with `./gradlew tomcatRunWar` and open [`http://localhost:8080`](http://localhost:8080). To stop the server, run `./gradlew tomcatStop`.

- Self-test modes for each subsystem

	Each subsystem contains a [`checkSystem()`](src/main/java/com/team254/frc2018/subsystems/Drive.java#L464) method that tests motor output currents and RPMs. These self tests allow us to quickly verify that all motors are working properly.

- Scale detection

	[Cheesy Vision 2.0](dash/CheesyVision2.py) is a Python app that uses OpenCV to track the angle of the scale. The app is meant to be run on the driver station computer and uses an external USB webcam pointed through the driver station glass at the scale. This allows us to set our elevator to the right height during autonomous and prevent wasting time by raising it higher than necessary, which we found was needed to complete a 4 cube auto within the time limit.

- Lidar Processing

	Even though this was not used on the final iteration of our robot code, we are still releasing our lidar processing code. This consisted of ICP algorithms to detect the scale within the points detected and sent by the [Slamtec RPLIDAR A2](http://www.slamtec.com/en/support#rplidar-a2) and can be found in the [`com.team254.frc2018.lidar`](src/main/java/com/team254/frc2018/lidar) package. Our RPLIDAR driver can be found [here](https://github.com/Team254/rplidar_sdk).

## Package Functions
- com.team254.frc2018

	Contains the robot's central functions and holds a file with all numerical constants used throughout the code. For example, the `Robot` class controls all routines depending on the robot state.

- com.team254.frc2018.auto

	Handles the execution of autonomous routines and contains the `actions`, `creators`, and `modes` packages.
	
- com.team254.frc2018.auto.actions

	Contains all actions used during the autonomous period, which all share a common interface, [`Action`](src/main/java/com/team254/frc2018/auto/actions/Action.java) (also in this package). Examples include shooting cubes, driving a trajectory, or moving the elevator. Routines interact with the Subsystems, which interact with the hardware.

- com.team254.frc2018.auto.creators

	Contains all the auto mode creators, which select the correct auto mode to run based on user input and FMS data.
	
- com.team254.frc2018.auto.modes
	
	Contains all autonomous modes. Autonomous modes consist of a list of autonomous actions executed in a certain order.

- com.team254.frc2018.controlboard
	
	Contains all the code for the different control boards. This allows any combination of driver station joysticks, button board, and Xbox Controllers to be used for both driving and operating. These are controlled by booleans in `Constants.java`.

- com.team254.frc2018.lidar

	Contains classes that are used to communicate with the Slamtec RPLIDAR A2 and to store and process points sent by the lidar.

- com.team254.frc2018.lidar.icp

	Contains the algorithms for processing points sent by the lidar.
	
- com.team254.frc2018.loops

	Loops are routines that run periodically on the robot, such as calculating robot pose, processing vision feedback, or updating subsystems. All loops implement the `Loop` interface and are handled (started, stopped, added) by the `Looper` class, which runs at 200 Hz.
    The `Robot` class has one main looper, `mEnabledLooper`, that runs all loops when the robot is enabled.
	
- com.team254.frc2018.paths

    Contains the generator for all of the trajectories that the robot drives during autonomous period.

- com.team254.frc2018.planners

	Loops are routines that run periodically on the robot, such as calculating robot pose, processing vision feedback, or updating subsystems. All loops implement the `Loop` interface and are handled (started, stopped, added) by the `Looper` class, which runs at 200 Hz.
	The `Robot` class has one main looper, `mEnabledLooper`, that runs all loops when the robot is enabled.

- com.team254.frc2018.statemachines

    Contains the state machines for the intake and overall superstructure.

- com.team254.frc2018.states

    Contains states and other classes used in the subsystem and state machine classes.

- com.team254.frc2018.subsystems
	
	Subsystems are consolidated into one central class per subsystem, all of which extend the Subsystem abstract class. Each subsystem uses state machines for control.
	Each subsystem is also a singleton, meaning that there is only one instance of each. To modify a subsystem, one would get the instance of the subsystem and change its state. The `Subsystem` class will work on setting the desired state.
	
- com.team254.lib.drivers

    Contains a set of custom classes for TalonSRXs.
	
- com.team254.lib.geometry

    Contains a set of classes that represent various geometric entities.
	
- com.team254.lib.physics

    Contains classes that model DC motor transmissions and differential drive characterization.

- com.team254.lib.spline

    Contains the code for generating and optimizing splines.

- com.team254.lib.trajectory

    Contains classes for following and storing trajectories.

- com.team254.lib.trajectory.timing

	Contains classes for fitting trajectories with time profiles.

- com.team254.lib.util

    A collection of assorted utilities classes used in the robot code. Check each file for more information.
	
## Variable Naming Conventions
- k*** (i.e. `kDriveWheelTrackWidthInches`): Final constants, especially those found in the Constants.java file
- m*** (i.e. `mIsHighGear`): Private instance variables
