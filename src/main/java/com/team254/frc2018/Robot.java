package com.team254.frc2018;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeExecutor;
import com.team254.frc2018.lidar.LidarProcessor;
import com.team254.frc2018.lidar.LidarServer;
import com.team254.frc2018.loops.Looper;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.statemachines.SuperstructureStateMachine;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.*;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends IterativeRobot {
    private Looper mEnabledLooper = new Looper();
    private Looper mDisabledLooper = new Looper();
    private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
    private IControlBoard mControlBoard = ControlBoard.getInstance();
    private AutoFieldState mAutoFieldState = AutoFieldState.getInstance();
    private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

    private final SubsystemManager mSubsystemManager = new SubsystemManager(
            Arrays.asList(
                    RobotStateEstimator.getInstance(),
                    Drive.getInstance(),
                    Superstructure.getInstance(),
                    Intake.getInstance(),
                    Wrist.getInstance(),
                    Elevator.getInstance(),
                    CarriageCanifier.getInstance(),
                    Infrastructure.getInstance(),
                    CheesyVision2.getInstance()
            )
    );

    private Drive mDrive = Drive.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Forklift mForklift = Forklift.getInstance();
    private LED mLED = LED.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Infrastructure mInfrastructure = Infrastructure.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Elevator mElevator = Elevator.getInstance();
    private CheesyVision2 mCheesyVision2 = CheesyVision2.getInstance();

    private LatchedBoolean mRunIntakeReleased = new LatchedBoolean();
    private LatchedBoolean mShootReleased = new LatchedBoolean();
    private LatchedBoolean mRunIntakePressed = new LatchedBoolean();

    private LatchedBoolean mHangModeEnablePressed = new LatchedBoolean();
    private LatchedBoolean mLowShiftPressed = new LatchedBoolean();
    private LatchedBoolean mHighShiftPressed = new LatchedBoolean();

    private LatchedBoolean mKickStandReleased = new LatchedBoolean();
    private MinTimeBoolean mKickStandRumble = new MinTimeBoolean(Constants.kKickstandToggleRumbleTime);

    private MinTimeBoolean mShootDelayed = new MinTimeBoolean(Constants.kMinShootTimeSec);
    private MinTimeBoolean mPoopyShootDelayed = new MinTimeBoolean(Constants.kMinShootTimeSec);

    private DelayedBoolean mIntakeLightsDelayed = new DelayedBoolean(Timer.getFPGATimestamp(), 0.1);

    private boolean mInHangMode;
    private boolean mKickStandEngaged;

    private double mLastHangModeTimestamp = 0.0;

    private AutoModeExecutor mAutoModeExecutor;

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            //init camera stream
            UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
            camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
            MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
            cameraServer.setSource(camera);

            CrashTracker.logRobotInit();

            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            mSubsystemManager.registerDisabledLoops(mDisabledLooper);

            mEnabledLooper.register(LidarProcessor.getInstance());

            try {
                SmartDashboard.putString("LIDAR status", "starting");
                boolean started = LidarServer.getInstance().start();
                SmartDashboard.putString("LIDAR status", started ? "started" : "failed to start");
            } catch (Throwable t) {
                SmartDashboard.putString("LIDAR status", "crashed: " + t);
                t.printStackTrace();
                throw t;
            }

            mLED.registerEnabledLoops(mEnabledLooper);
            mLED.registerEnabledLoops(mDisabledLooper);
            mCheesyVision2.registerEnabledLoops(mDisabledLooper);

            mTrajectoryGenerator.generateTrajectories();

            mRunIntakeReleased.update(true);
            mShootReleased.update(true);

            mAutoModeSelector.updateModeCreator();

            // Set the auto field state at least once.
            mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
            mKickStandEngaged = true;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mInfrastructure.setIsDuringAuto(true);
            Drive.getInstance().zeroSensors();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            // Reset all auto mode state.
            mAutoModeSelector.reset();
            mAutoModeSelector.updateModeCreator();
            mAutoModeExecutor = new AutoModeExecutor();

            mDisabledLooper.start();

            mLED.setEnableFaults(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        try {
            CrashTracker.logAutoInit();
            mDisabledLooper.stop();

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

            Drive.getInstance().zeroSensors();
            mInfrastructure.setIsDuringAuto(true);

            mWrist.setRampRate(Constants.kAutoWristRampRate);

            mAutoModeExecutor.start();

            mLED.setEnableFaults(false);
            mEnabledLooper.start();

            mSuperstructure.setKickstand(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        SmartDashboard.putString("Match Cycle", "TELEOP");

        try {
            CrashTracker.logTeleopInit();
            mDisabledLooper.stop();
            if (mAutoModeExecutor != null) {
                mAutoModeExecutor.stop();
            }

            mAutoFieldState.disableOverride();

            mInfrastructure.setIsDuringAuto(false);
            mWrist.setRampRate(Constants.kWristRampRate);

            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mEnabledLooper.start();
            mLED.setEnableFaults(false);
            mInHangMode = false;
            mForklift.retract();

            mShootDelayed.update(false, Double.POSITIVE_INFINITY);
            mPoopyShootDelayed.update(false, Double.POSITIVE_INFINITY);
            mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
            mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));

            mKickStandEngaged = true;
            mKickStandReleased.update(true);
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        SmartDashboard.putString("Match Cycle", "TEST");

        try {
            System.out.println("Starting check systems.");

            mDisabledLooper.stop();
            mEnabledLooper.stop();

            //mDrive.checkSystem();
            //mIntake.checkSystem();
            //mWrist.checkSystem();
            mElevator.checkSystem();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledPeriodic() {
        SmartDashboard.putString("Match Cycle", "DISABLED");

        try {
            outputToSmartDashboard();
            mWrist.resetIfAtLimit();
            mElevator.resetIfAtLimit();

            // Poll FMS auto mode info and update mode creator cache
            mAutoFieldState.setSides(DriverStation.getInstance().getGameSpecificMessage());
            mAutoModeSelector.updateModeCreator();

            if (mAutoFieldState.isValid()) {
                Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode(mAutoFieldState);
                if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
                    System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
                    mAutoModeExecutor.setAutoMode(autoMode.get());
                }
                System.gc();
            }
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
        SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

        outputToSmartDashboard();
        try {

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopPeriodic() {
        SmartDashboard.putString("Match Cycle", "TELEOP");
        double timestamp = Timer.getFPGATimestamp();

        double throttle = mControlBoard.getThrottle();
        double turn = mControlBoard.getTurn();

        try {
            // When elevator is up, tune sensitivity on turn a little.
            if (mElevator.getInchesOffGround() > Constants.kElevatorLowSensitivityThreshold) {
                turn *= Constants.kLowSensitivityFactor;
            }
            mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(),
                    mDrive.isHighGear()));

            if (mHangModeEnablePressed.update(mControlBoard.getEnableHangMode())) {
                if (mInHangMode) {
                    mInHangMode = false;
                } else {
                    mInHangMode = true;
                }
                mSuperstructure.setHangMode(mInHangMode);
            }

            if (mInHangMode) {
                mLED.setWantedAction(LED.WantedAction.DISPLAY_HANG);
                if (mControlBoard.getDeployForks()) {
                    // This is fire once only!
                    mForklift.deploy();
                }

                double elevatorThrottle = mControlBoard.getElevatorThrottle();
                if (Math.abs(elevatorThrottle) < Constants.kElevatorThrottleDeadband) {
                    elevatorThrottle = 0.0;
                } else {
                    elevatorThrottle =
                            (elevatorThrottle - Math.signum(elevatorThrottle) *
                                    Constants.kElevatorThrottleDeadband) /
                                    (1.0 - Constants.kElevatorThrottleDeadband);
                }
                mSuperstructure.setHangThrottle(elevatorThrottle);

                if (mLowShiftPressed.update(mControlBoard.getElevatorLowShift())) {
                    mSuperstructure.setElevatorLowGear();
                } else if (mHighShiftPressed.update(mControlBoard.getElevatorHighShift())) {
                    mSuperstructure.setElevatorHighGear();
                }
                mSuperstructure.setUnlockHookSolenoid(true);
                mKickStandEngaged = true;
                mLastHangModeTimestamp = timestamp;
                mKickStandReleased.update(true);
            } else {
                mSuperstructure.setUnlockHookSolenoid(false);
                mForklift.retract();

                // LEDs

                if (mControlBoard.getWantsCubeLEDBlink()) {
                    mLED.setWantedAction(LED.WantedAction.DISPLAY_WANTS_CUBE);
                } else {
                    mLED.setWantedAction(LED.WantedAction.DISPLAY_INTAKE);
                }

                // Intake/Shoot
                boolean runIntakePosition = mControlBoard.getIntakePosition() &&
                        (mWrist.getAngle() > SuperstructureConstants.kMinIntakePositionAngle);
                boolean runIntake = mControlBoard.getRunIntake() || runIntakePosition;
                boolean forceSuperClamp = mControlBoard.getRunIntake() && runIntakePosition;
                boolean runIntakeReleased = mRunIntakeReleased.update(!runIntake);
                boolean intakeAction = false;

                boolean normalShoot = mControlBoard.getShoot();
                boolean poopyShoot = mControlBoard.getPoopyShoot();

                boolean shootReleased = mShootReleased.update(!(normalShoot || poopyShoot));

                if (forceSuperClamp) {
                    mIntake.forceClampJaw(true);
                } else {
                    mIntake.forceClampJaw(false);
                }

                if (runIntake) {
                    mIntake.getOrKeepCube();
                    intakeAction = true;
                } else if (normalShoot) {
                    intakeAction = true;
                    if (mElevator.getInchesOffGround() < SuperstructureConstants.kSwitchHeight + 5.0) {
                        if (mWrist.getAngle() > SuperstructureConstants.kWeakShootAngle) {
                            mIntake.shoot(IntakeStateMachine.kSwitchShootSetpoint);
                        } else {
                            mIntake.shoot(IntakeStateMachine.kExchangeShootSetpoint);
                        }
                    } else {
                        if (mWrist.getAngle() > SuperstructureConstants.kWeakShootAngle) {
                            mIntake.shoot(IntakeStateMachine.kWeakShootSetpoint);
                        } else {
                            mIntake.shoot(IntakeStateMachine.kStrongShootSetpoint);
                        }
                    }
                } else if (poopyShoot) {
                    intakeAction = true;
                    if (mElevator.getInchesOffGround() < SuperstructureConstants.kSwitchHeight + 5.0 && mWrist.getAngle() < SuperstructureConstants.kWeakShootAngle) {
                        mIntake.shoot(IntakeStateMachine.kExchangeShootSetpoint);
                    } else {
                        mIntake.shoot(IntakeStateMachine.kPoopyShootSetpoint);
                    }
                } else if (runIntakeReleased) {
                    if (mIntake.hasCube()) {
                        mIntake.getOrKeepCube();
                    } else {
                        mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
                        mIntake.setPower(0.0);
                    }
                    intakeAction = true;
                } else if (shootReleased) {
                    mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
                    mIntake.setPower(0.0);
                    intakeAction = true;
                }

                // Manual jaw inputs.
                if (mControlBoard.getOpenJaw()) {
                    mIntake.tryOpenJaw();
                    if (!intakeAction) {
                        mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
                        mIntake.setPower(0.0);
                    }
                } else if (normalShoot) {
                    mIntake.clampJaw();
                } else {
                    mIntake.closeJaw();
                }

                // Rumble
                boolean should_rumble = runIntake &&
                        mIntakeLightsDelayed.update(Timer.getFPGATimestamp(),
                                mIntake.definitelyHasCube());

                boolean kick_stand_released =
                        mKickStandReleased.update(!mControlBoard.getToggleKickstand())
                                && (timestamp - mLastHangModeTimestamp > Constants.kKickstandDelay);
                // Only toggle if below.
                if (kick_stand_released &&
                        (mElevator.getInchesOffGround() <
                                SuperstructureConstants.kClearFirstStageMaxHeight)) {
                    mKickStandEngaged = !mKickStandEngaged;
                    // Force a rising edge.
                    mKickStandRumble.update(false, timestamp);
                    mKickStandRumble.update(true, timestamp);
                }

                should_rumble = should_rumble | mKickStandRumble.update(false, timestamp);
                if (should_rumble) {
                    mControlBoard.setRumble(true);
                } else {
                    mControlBoard.setRumble(false);
                }

                double back_high_height = mKickStandEngaged ?
                        SuperstructureConstants.kScaleHighHeightBackwards :
                        SuperstructureConstants.kScaleHighHeightBackwardsNoKick;
                double back_neutral_height = mKickStandEngaged ?
                        SuperstructureConstants.kScaleNeutralHeightBackwards :
                        SuperstructureConstants.kScaleNeutralHeightBackwardsNoKick;
                double back_low_height = mKickStandEngaged ?
                        SuperstructureConstants.kScaleLowHeightBackwards :
                        SuperstructureConstants.kScaleLowHeightBackwardsNoKick;

                double high_height = mKickStandEngaged ?
                        SuperstructureConstants.kScaleHighHeight :
                        SuperstructureConstants.kScaleHighHeightNoKick;
                double neutral_height = mKickStandEngaged ?
                        SuperstructureConstants.kScaleNeutralHeight :
                        SuperstructureConstants.kScaleNeutralHeightNoKick;
                double low_height = mKickStandEngaged ?
                        SuperstructureConstants.kScaleLowHeight :
                        SuperstructureConstants.kScaleLowHeightNoKick;

                double backwards_angle = mKickStandEngaged ?
                        SuperstructureConstants.kScoreBackwardsAngle :
                        SuperstructureConstants.kScoreBackwardsAngleNoKick;

                // Presets.
                double desired_height = Double.NaN;
                double desired_angle = Double.NaN;

                if (mControlBoard.getGoToStowHeight()) {
                    desired_height = SuperstructureConstants.kStowedPositionHeight;
                    desired_angle = SuperstructureConstants.kStowedPositionAngle;
                }

                if (mRunIntakePressed.update(mControlBoard.getIntakePosition())) {
                    desired_height = SuperstructureConstants.kIntakePositionHeight;
                    desired_angle = SuperstructureConstants.kIntakePositionAngle;
                }

                // Intake Preset locations
                boolean go_high_scale = mControlBoard.getGoToHighScaleHeight();
                boolean go_neutral_scale = mControlBoard.getGoToNeutralScaleHeight();
                boolean go_low_scale = mControlBoard.getGoToLowScaleHeight();
                boolean go_switch = mControlBoard.getGoToSwitchHeight();
                if (mControlBoard.getIntakePosition()) {
                    desired_angle = SuperstructureConstants.kIntakePositionAngle;
                    if (go_high_scale) {
                        desired_height = SuperstructureConstants.kIntakeThirdLevelHeight;
                    } else if (go_neutral_scale) {
                        desired_height = SuperstructureConstants.kIntakeSecondLevelHeight;
                    } else if (go_low_scale) {
                        desired_height = SuperstructureConstants.kIntakeFloorLevelHeight;
                    }
                } else if (mControlBoard.getBackwardsModifier()) {
                    // These are score backwards
                    if (go_high_scale) {
                        desired_height = back_high_height;
                        desired_angle = backwards_angle;
                    } else if (go_neutral_scale) {
                        desired_height = back_neutral_height;
                        desired_angle = backwards_angle;
                    } else if (go_low_scale) {
                        desired_height = back_low_height;
                        desired_angle = backwards_angle;
                    } else if (go_switch) {
                        desired_height = SuperstructureConstants.kSwitchHeightBackwards;
                        desired_angle = SuperstructureConstants.kScoreSwitchBackwardsAngle;
                    }
                } else {
                    // These are score forward
                    if (go_high_scale) {
                        desired_height = high_height;
                        desired_angle = SuperstructureConstants.kScoreForwardAngledAngle;
                    } else if (go_neutral_scale) {
                        desired_height = neutral_height;
                        desired_angle = SuperstructureConstants.kScoreForwardAngledAngle;
                    } else if (go_low_scale) {
                        desired_height = low_height;
                        desired_angle = SuperstructureConstants.kScoreForwardAngledAngle;
                    } else if (go_switch) {
                        desired_height = SuperstructureConstants.kSwitchHeight;
                        desired_angle = SuperstructureConstants.kPlacingLowAngle;
                    }
                }

                // Wrist.
                if (mControlBoard.goToStowWrist()) {
                    desired_angle = SuperstructureConstants.kStowedPositionAngle;
                } else if (mControlBoard.goToIntakingWrist()) {
                    if (mSuperstructure.getScoringHeight() > SuperstructureConstants.kPlacingHighThreshold) {
                        desired_angle = SuperstructureConstants.kPlacingHighAngle;
                    } else {
                        desired_angle = SuperstructureConstants.kPlacingLowAngle;
                    }
                } else if (mControlBoard.goToVerticalWrist()) {
                    desired_angle = SuperstructureConstants.kVerticalAngle;
                } else if (mControlBoard.goToScoringWrist()) {
                    desired_angle = SuperstructureConstants.kScoreBackwardsAngle;
                } else if (mControlBoard.goToScoringAngledWrist()) {
                    desired_angle = SuperstructureConstants.kScoreForwardAngledAngle;
                }

                // Attempt to fix wrist angle if we will be in an invalid state.
                if (!Double.isNaN(desired_height) && Double.isNaN(desired_angle) &&
                        desired_height > SuperstructureConstants.kClearFirstStageMaxHeight) {
                    if (mSuperstructure.getScoringAngle() <
                            SuperstructureConstants.kClearFirstStageMinWristAngle) {
                        desired_angle = SuperstructureConstants.kClearFirstStageMinWristAngle;
                    }
                }

                if (Double.isNaN(desired_angle) && Double.isNaN(desired_height)) {
                    mSuperstructure.setWantedAction(SuperstructureStateMachine.WantedAction.IDLE);
                } else if (Double.isNaN(desired_angle)) {
                    mSuperstructure.setDesiredHeight(desired_height);
                } else if (Double.isNaN(desired_height)) {
                    mSuperstructure.setDesiredAngle(desired_angle);
                } else if (!Double.isNaN(desired_angle) && !Double.isNaN(desired_height)) {
                    mSuperstructure.setDesiredAngle(desired_angle);
                    mSuperstructure.setDesiredHeight(desired_height);
                }

                double elevator_jog = mControlBoard.getJogElevatorThrottle();
                if (Math.abs(elevator_jog) > Constants.kJoystickJogThreshold) {
                    elevator_jog =
                            (elevator_jog - Math.signum(elevator_jog) *
                                    Constants.kJoystickJogThreshold) /
                                    (1.0 - Constants.kJoystickJogThreshold);
                    mSuperstructure.setElevatorJog(
                            elevator_jog * SuperstructureConstants.kElevatorJogThrottle);
                }

                double wrist_jog = mControlBoard.getJogWristThrottle();
                if (Math.abs(wrist_jog) > Constants.kJoystickJogThreshold) {
                    wrist_jog =
                            (wrist_jog - Math.signum(wrist_jog) *
                                    Constants.kJoystickJogThreshold) /
                                    (1.0 - Constants.kJoystickJogThreshold);
                    mSuperstructure.setWristJog(
                            wrist_jog * SuperstructureConstants.kWristJogThrottle);
                }

            }
            mSuperstructure.setKickstand(mKickStandEngaged);

            outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putString("Match Cycle", "TEST");
    }

    public void outputToSmartDashboard() {
        RobotState.getInstance().outputToSmartDashboard();
        Drive.getInstance().outputTelemetry();
        Wrist.getInstance().outputTelemetry();
        Intake.getInstance().outputTelemetry();
        Elevator.getInstance().outputTelemetry();
        Infrastructure.getInstance().outputTelemetry();
        mAutoFieldState.outputToSmartDashboard();
        mEnabledLooper.outputToSmartDashboard();
        mAutoModeSelector.outputToSmartDashboard();
        mCheesyVision2.outputTelemetry();
        // SmartDashboard.updateValues();
    }
}
