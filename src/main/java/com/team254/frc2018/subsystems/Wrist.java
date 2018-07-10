package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Wrist extends Subsystem {
    private static final int kMagicMotionSlot = 0;
    private static final int kPositionControlSlot = 1;
    private static final int kForwardSoftLimit = 2100;  // Encoder ticks.
    private static final int kReverseSoftLimit = -500;  // Encoder ticks.  TODO make ~0 once skipping is fixed.

    private static final double kHomingOutput = -0.25;
    private boolean mHasBeenZeroed = false;

    private static Wrist mInstance;
    private final Intake mIntake = Intake.getInstance();
    private final CarriageCanifier mCanifier = CarriageCanifier.getInstance();
    private final Elevator mElevator = Elevator.getInstance();
    private final TalonSRX mMaster;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private double mZeroPosition = Double.NaN;
    private SystemState mSystemState = SystemState.HOMING;
    private SystemState mDesiredState = SystemState.MOTION_PROFILING;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;

    private Wrist() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.KWristMasterId);
        ErrorCode errorCode;

        //configure talon
        errorCode = mMaster.configRemoteFeedbackFilter(Constants.kCanifierId, RemoteSensorSource.CANifier_Quadrature,
                0, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist encoder!!!: " + errorCode, false);


        errorCode = mMaster.configSelectedFeedbackSensor(RemoteFeedbackDevice.RemoteSensor0, 0, Constants
                .kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not detect wrist encoder: " + errorCode, false);

        errorCode = mMaster.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal
                .NormallyOpen, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set forward limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal
                .NormallyOpen, mCanifier.getDeviceId(), Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse limit switch wrist: " + errorCode, false);

        errorCode = mMaster.configForwardSoftLimitThreshold(kForwardSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set forward soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable forward soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitThreshold(kReverseSoftLimit, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set reverse soft limit switch wrist: " + errorCode, false);
        errorCode = mMaster.configReverseSoftLimitEnable(true, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not enable reverse soft limit switch wrist: " + errorCode, false);

        errorCode = mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist voltage compensation: " + errorCode, false);

        //configure magic motion
        errorCode = mMaster.config_kP(kMagicMotionSlot, Constants.kWristKp, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kp: " + errorCode, false);

        errorCode = mMaster.config_kI(kMagicMotionSlot, Constants.kWristKi, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist ki: " + errorCode, false);

        errorCode = mMaster.config_kD(kMagicMotionSlot, Constants.kWristKd, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kd: " + errorCode, false);

        errorCode = mMaster.config_kF(kMagicMotionSlot, Constants.kWristKf, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kf: " + errorCode, false);

        errorCode = mMaster.configMaxIntegralAccumulator(kMagicMotionSlot, Constants.kWristMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist max integral: " + errorCode, false);

        errorCode = mMaster.config_IntegralZone(kMagicMotionSlot, Constants.kWristIZone, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist i zone: " + errorCode, false);

        errorCode = mMaster.configAllowableClosedloopError(kMagicMotionSlot, Constants.kWristDeadband, Constants
                .kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist deadband: " + errorCode, false);

        errorCode = mMaster.configMotionAcceleration(Constants.kWristAcceleration, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist acceleration: " + errorCode, false);

        errorCode = mMaster.configMotionCruiseVelocity(Constants.kWristCruiseVelocity, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist cruise velocity: " + errorCode, false);


        // Configure position PID
        errorCode = mMaster.config_kP(kPositionControlSlot, Constants.kWristJogKp, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kp: " + errorCode, false);

        errorCode = mMaster.config_kI(kPositionControlSlot, Constants.kWristKi, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist ki: " + errorCode, false);

        errorCode = mMaster.config_kD(kPositionControlSlot, Constants.kWristJogKd, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist kd: " + errorCode, false);

        errorCode = mMaster.configMaxIntegralAccumulator(kPositionControlSlot, Constants.kWristMaxIntegralAccumulator,
                Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist max integral: " + errorCode, false);

        errorCode = mMaster.config_IntegralZone(kPositionControlSlot, Constants.kWristIZone, Constants.kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist i zone: " + errorCode, false);

        errorCode = mMaster.configAllowableClosedloopError(kPositionControlSlot, Constants.kWristDeadband, Constants
                .kLongCANTimeoutMs);
        if (errorCode != ErrorCode.OK)
            DriverStation.reportError("Could not set wrist deadband: " + errorCode, false);


        TalonSRXUtil.checkError(
                mMaster.configContinuousCurrentLimit(20, Constants.kLongCANTimeoutMs),
                "Could not set wrist continuous current limit.");

        TalonSRXUtil.checkError(
                mMaster.configPeakCurrentLimit(40, Constants.kLongCANTimeoutMs),
                "Could not set wrist peak current limit.");

        TalonSRXUtil.checkError(
                mMaster.configPeakCurrentDuration(200, Constants.kLongCANTimeoutMs),
                "Could not set wrist peak current duration.");

        TalonSRXUtil.checkError(
                mMaster.configClosedloopRamp(
                        Constants.kWristRampRate, Constants.kLongCANTimeoutMs),
                "Could not set wrist voltage ramp rate: ");

        mMaster.enableCurrentLimit(true);

        mMaster.selectProfileSlot(0, 0);

        mMaster.setInverted(false);
        mMaster.setSensorPhase(false);
        mMaster.setNeutralMode(NeutralMode.Brake);
        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(true);

        mMaster.enableVoltageCompensation(true);
        mMaster.set(ControlMode.PercentOutput, 0);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

        // DO NOT reset encoder positions on limit switch
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
    }

    public synchronized static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist();
        }
        return mInstance;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putNumber("Wrist Position", getPosition());
        SmartDashboard.putNumber("Wrist Ticks", mPeriodicIO.position_ticks);
        SmartDashboard.putNumber("Wrist periodic demand", mPeriodicIO.demand);
        SmartDashboard.putBoolean("LIMR", mPeriodicIO.limit_switch);

        SmartDashboard.putNumber("Wrist RPM", getRPM());
        SmartDashboard.putNumber("Wrist Power %", mPeriodicIO.output_percent);
        SmartDashboard.putBoolean("Wrist Limit Switch", mPeriodicIO.limit_switch);
        SmartDashboard.putNumber("Wrist Last Expected Trajectory", getSetpoint());
        SmartDashboard.putNumber("Wrist Current Trajectory Point", mPeriodicIO.active_trajectory_position);
        SmartDashboard.putNumber("Wrist Traj Vel", mPeriodicIO.active_trajectory_velocity);
        SmartDashboard.putNumber("Wrist Traj Accel", mPeriodicIO.active_trajectory_acceleration_rad_per_s2);
        SmartDashboard.putBoolean("Wrist Has Sent Trajectory", hasFinishedTrajectory());
        SmartDashboard.putNumber("Wrist feedforward", mPeriodicIO.feedforward);

        if (mCSVWriter != null) {
            mCSVWriter.write();
        }

    }

    public synchronized void setRampRate(double rampRate) {
        mMaster.configClosedloopRamp(rampRate, 0);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 0);
        mCanifier.resetWristEncoder();
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double mHomingStartTime;
            boolean mFoundHome;

            @Override
            public void onStart(double timestamp) {
                mHomingStartTime = timestamp;
                // startLogging();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Wrist.this) {
                    if (!Double.isNaN(mZeroPosition) && mDesiredState != mSystemState) {
                        System.out.println(timestamp + ": Wrist changed states: " + mSystemState + " -> " +
                                mDesiredState);
                        mSystemState = mDesiredState;
                    }

                    switch (mSystemState) {
                        case OPEN_LOOP:
                            // Handled in writePeriodicOutputs
                            break;
                        case MOTION_PROFILING:
                            // Handled in writePeriodicOutputs
                            break;
                        case HOMING:
                            // TODO get this working again
//                            if (Double.isNaN(mZeroPosition)) {
//                                mPeriodicOutputs.demand = resetIfAtLimit() ? 0.0 : kHomingOutput;
//                            } else {
                            mSystemState = SystemState.OPEN_LOOP;

                            break;
                        default:
                            System.out.println("Fell through on Wrist states!");
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stopLogging();
            }
        });
    }

    public synchronized void setOpenLoop(double percentage) {
        mPeriodicIO.demand = percentage;
        mDesiredState = SystemState.OPEN_LOOP;
    }

    public synchronized boolean resetIfAtLimit() {
        if (mCanifier.getLimR()) {
            zeroSensors();
            return true;
        }
        return false;
    }

    /**
     * @param position the target position of the wrist in sensor units
     */
    public void setClosedLoop(int position) {
        mPeriodicIO.demand = (position);
        mDesiredState = SystemState.MOTION_PROFILING;
    }

    /**
     * @param angle the target position of the wrist in degrees.  0 is full back, 180 is facing forwards
     */
    public synchronized void setMotionProfileAngle(double angle) {
        mPeriodicIO.demand = (degreesToSensorUnits(angle));
        if (mDesiredState != SystemState.MOTION_PROFILING) {
            mDesiredState = SystemState.MOTION_PROFILING;
            mMaster.selectProfileSlot(kMagicMotionSlot, 0);
        }
    }

    /**
     * @param angle the target position of the wrist in degrees.  0 is full back, 180 is facing forwards
     */
    public synchronized void setPositionPIDAngle(double angle) {
        mPeriodicIO.demand = (degreesToSensorUnits(angle));
        if (mDesiredState != SystemState.POSITION_PID) {
            mDesiredState = SystemState.POSITION_PID;
            mMaster.selectProfileSlot(kPositionControlSlot, 0);
        }
    }

    /**
     * @return current position of the wrist in sensor units
     */
    public synchronized double getPosition() { //returns angle of wrist in degrees
        return (mPeriodicIO.position_ticks);
    }

    /**
     * @return current angle of the wrist in degrees
     */
    public synchronized double getAngle() { //returns angle of wrist in degrees
        return sensorUnitsToDegrees((mPeriodicIO.position_ticks));
    }

    /**
     * @return current velocity in rpm
     */
    public double getRPM() {
        return sensorUnitsToDegrees(mPeriodicIO.velocity_ticks_per_100ms) * 600.0 / 360.0;
    }

    /**
     * @return current velocity in degrees per second
     */
    public double getDegreesPerSecond() {
        return sensorUnitsToDegrees(mPeriodicIO.velocity_ticks_per_100ms) * 10.0;
    }

    public synchronized boolean hasFinishedTrajectory() {
        if (Util.epsilonEquals(mPeriodicIO.active_trajectory_position,
                degreesToSensorUnits(getSetpoint()), 2)) {
            return true;
        }
        return false;
    }

    public synchronized double getSetpoint() {
        return mDesiredState == SystemState.MOTION_PROFILING || mDesiredState == SystemState.POSITION_PID
                ? sensorUnitsToDegrees((mPeriodicIO.demand)) : Double.NaN;
    }

    private double sensorUnitsToDegrees(double units) {
        return units / 4096.0 * 360.0;
    }

    private double degreesToSensorUnits(double degrees) {
        return degrees * 4096.0 / 360.0;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        if (mMaster.hasResetOccurred()) {
            DriverStation.reportError("Wrist Talon Reset! ", false);
        }
        StickyFaults faults = new StickyFaults();
        mMaster.getStickyFaults(faults);
        if (faults.hasAnyFault()) {
            DriverStation.reportError("Wrist Talon Fault! " + faults.toString(), false);
            mMaster.clearStickyFaults(0);
        }
        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicIO.active_trajectory_position = mMaster.getActiveTrajectoryPosition();

            if (mPeriodicIO.active_trajectory_position < kReverseSoftLimit) {
                DriverStation.reportError("Active trajectory past reverse soft limit!", false);
            } else if (mPeriodicIO.active_trajectory_position > kForwardSoftLimit) {
                DriverStation.reportError("Active trajectory past forward soft limit!", false);
            }
            final int newVel = mMaster.getActiveTrajectoryVelocity();
            // TODO check sign of accel
            if (Util.epsilonEquals(newVel, Constants.kWristCruiseVelocity, 5) ||
                    Util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, 5)) {
                // Wrist is ~constant velocity.
                mPeriodicIO.active_trajectory_acceleration_rad_per_s2 = 0.0;
            } else {
                // Wrist is accelerating.
                mPeriodicIO.active_trajectory_acceleration_rad_per_s2 = Math.signum(newVel - mPeriodicIO
                        .active_trajectory_velocity) * Constants.kWristAcceleration * 20.0 * Math.PI /
                        4096;
            }
            mPeriodicIO.active_trajectory_velocity = newVel;
        } else {
            mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
            mPeriodicIO.active_trajectory_velocity = 0;
            mPeriodicIO.active_trajectory_acceleration_rad_per_s2 = 0.0;
        }
        mPeriodicIO.limit_switch = mCanifier.getLimR();
        mPeriodicIO.output_voltage = mMaster.getMotorOutputVoltage();
        mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
        mPeriodicIO.position_ticks = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity_ticks_per_100ms = mMaster.getSelectedSensorVelocity(0);

        if (getAngle() > Constants.kWristEpsilon ||
                sensorUnitsToDegrees(mPeriodicIO.active_trajectory_position) > Constants.kWristEpsilon) {
            double wristGravityComponent = Math.cos(Math.toRadians(getAngle())) * (mIntake.hasCube() ? Constants
                    .kWristKfMultiplierWithCube : Constants.kWristKfMultiplierWithoutCube);
            double elevatorAccelerationComponent = mElevator.getActiveTrajectoryAccelG() * Constants
                    .kWristElevatorAccelerationMultiplier;
            double wristAccelerationComponent = mPeriodicIO.active_trajectory_acceleration_rad_per_s2 *
                    (mIntake.hasCube() ? Constants.kWristKaWithCube : Constants.kWristKaWithoutCube);
            mPeriodicIO.feedforward = (elevatorAccelerationComponent) * wristGravityComponent + wristAccelerationComponent;
        } else {
            if (getSetpoint() < Util.kEpsilon) {
                mPeriodicIO.feedforward = -0.1;
            } else {
                mPeriodicIO.feedforward = 0.0;
            }
        }
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mDesiredState == SystemState.MOTION_PROFILING) {
            mMaster.set(ControlMode.MotionMagic,
                    mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        } else if (mDesiredState == SystemState.POSITION_PID) {
            mMaster.set(ControlMode.Position,
                    mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        } else {
            mMaster.set(ControlMode.PercentOutput,
                    mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        }
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                    {
                        add(new TalonSRXChecker.TalonSRXConfig("wrist_master", mMaster));
                    }
                }, new TalonSRXChecker.CheckerConfig() {
                    {
                        mRunTimeSec = 1.0;
                        mRunOutputPercentage = 0.20;

                        mRPMFloor = 50.0;
                        mCurrentFloor = 2.0;

                        mRPMSupplier = () -> mMaster.getSelectedSensorVelocity(0);
                    }
                });
    }

    public synchronized void startLogging() {
        if (mCSVWriter == null) {
            mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/WRIST-LOGS.csv", PeriodicIO.class);
        }
    }

    public synchronized void stopLogging() {
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    public enum SystemState {
        HOMING,
        MOTION_PROFILING,
        POSITION_PID,
        OPEN_LOOP,
    }

    public static class PeriodicIO {
        // INPUTS
        public int position_ticks;
        public int velocity_ticks_per_100ms;
        public int active_trajectory_position;
        public int active_trajectory_velocity;
        public double active_trajectory_acceleration_rad_per_s2;
        public double output_percent;
        public double output_voltage;
        public double feedforward;
        public boolean limit_switch;

        // OUTPUTS
        public double demand;
    }
}

