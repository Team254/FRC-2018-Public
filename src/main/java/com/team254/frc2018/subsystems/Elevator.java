package com.team254.frc2018.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

// Top Soft limit : -151288
public class Elevator extends Subsystem {
    public static final double kHomePositionInches = 5.0;
    private static final int kHighGearSlot = 0;
    private static final int kLowGearSlot = 1;
    private static final int kPositionControlSlot = 2;
    private static final int kReverseSoftLimit = -100000; // Encoder ticks (used to be -151000)
    private static final int kForwardSoftLimit = 500; // Encoder ticks.  TODO set to ~0 once skipping is fixed.
    private static final double kEncoderTicksPerInch = -1271.0;
    private static Elevator mInstance = null;
    private Intake mIntake = Intake.getInstance();
    private final TalonSRX mMaster, mRightSlave, mLeftSlaveA, mLeftSlaveB;
    private final Solenoid mShifter;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private ElevatorControlState mElevatorControlState = ElevatorControlState.OPEN_LOOP;

    private boolean mHasBeenZeroed = false;

    private Elevator() {
        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kElevatorMasterId);

        TalonSRXUtil.checkError(
                mMaster.configSelectedFeedbackSensor(
                        FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100),
                "Could not detect elevator encoder: ");

        TalonSRXUtil.checkError(
                mMaster.configForwardLimitSwitchSource(
                        LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
                        Constants.kLongCANTimeoutMs),
                "Could not set forward (down) limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configForwardSoftLimitThreshold(
                        kForwardSoftLimit, Constants.kLongCANTimeoutMs),
                "Could not set forward (down) soft limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configForwardSoftLimitEnable(true, Constants.kLongCANTimeoutMs),
                "Could not enable forward (down) soft limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs),
                "Could not set voltage compensation saturation elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configReverseSoftLimitThreshold(
                        kReverseSoftLimit, Constants.kLongCANTimeoutMs),
                "Could not set reverse (up) soft limit switch elevator: ");

        TalonSRXUtil.checkError(
                mMaster.configReverseSoftLimitEnable(
                        true, Constants.kLongCANTimeoutMs),
                "Could not enable reverse (up) soft limit switch elevator: ");

        //configure magic motion
        TalonSRXUtil.checkError(
                mMaster.config_kP(
                        kHighGearSlot, Constants.kElevatorHighGearKp, Constants.kLongCANTimeoutMs),
                "Could not set elevator kp: ");

        TalonSRXUtil.checkError(
                mMaster.config_kI(
                        kHighGearSlot, Constants.kElevatorHighGearKi, Constants.kLongCANTimeoutMs),
                "Could not set elevator ki: ");

        TalonSRXUtil.checkError(
                mMaster.config_kD(
                        kHighGearSlot, Constants.kElevatorHighGearKd + Constants.kElevatorHighGearKd / 100.0, Constants.kLongCANTimeoutMs),
                "Could not set elevator kd: ");

        TalonSRXUtil.checkError(
                mMaster.config_kF(
                        kHighGearSlot, Constants.kElevatorHighGearKf, Constants.kLongCANTimeoutMs),
                "Could not set elevator kf: ");

        TalonSRXUtil.checkError(
                mMaster.configMaxIntegralAccumulator(
                        kHighGearSlot, Constants.kElevatorHighGearMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
                "Could not set elevator max integral: ");

        TalonSRXUtil.checkError(
                mMaster.config_IntegralZone(
                        kHighGearSlot, Constants.kElevatorHighGearIZone, Constants.kLongCANTimeoutMs),
                "Could not set elevator i zone: ");

        TalonSRXUtil.checkError(
                mMaster.configAllowableClosedloopError(
                        kHighGearSlot, Constants.kElevatorHighGearDeadband, Constants.kLongCANTimeoutMs),
                "Could not set elevator deadband: ");

        TalonSRXUtil.checkError(
                mMaster.configMotionAcceleration(
                        Constants.kElevatorHighGearAcceleration, Constants.kLongCANTimeoutMs),
                "Could not set elevator acceleration: ");

        TalonSRXUtil.checkError(
                mMaster.configMotionCruiseVelocity(
                        Constants.kElevatorHighGearCruiseVelocity, Constants.kLongCANTimeoutMs),
                "Could not set elevator cruise velocity: ");

        //configure position PID
        TalonSRXUtil.checkError(
                mMaster.config_kP(
                        kPositionControlSlot, Constants.kElevatorJogKp, Constants.kLongCANTimeoutMs),
                "Could not set elevator kp: ");

        TalonSRXUtil.checkError(
                mMaster.config_kI(
                        kPositionControlSlot, Constants.kElevatorHighGearKi, Constants.kLongCANTimeoutMs),
                "Could not set elevator ki: ");

        TalonSRXUtil.checkError(
                mMaster.config_kD(
                        kPositionControlSlot, Constants.kElevatorJogKd, Constants.kLongCANTimeoutMs),
                "Could not set elevator kd: ");

        TalonSRXUtil.checkError(
                mMaster.configMaxIntegralAccumulator(
                        kPositionControlSlot, Constants.kElevatorHighGearMaxIntegralAccumulator, Constants.kLongCANTimeoutMs),
                "Could not set elevator max integral: ");

        TalonSRXUtil.checkError(
                mMaster.config_IntegralZone(
                        kPositionControlSlot, Constants.kElevatorHighGearIZone, Constants.kLongCANTimeoutMs),
                "Could not set elevator i zone: ");

        TalonSRXUtil.checkError(
                mMaster.configAllowableClosedloopError(
                        kPositionControlSlot, Constants.kElevatorHighGearDeadband, Constants.kLongCANTimeoutMs),
                "Could not set elevator deadband: ");


        TalonSRXUtil.checkError(
                mMaster.configClosedloopRamp(
                        Constants.kElevatorRampRate, Constants.kLongCANTimeoutMs),
                "Could not set elevator voltage ramp rate: ");

        TalonSRXUtil.checkError(
                mMaster.configOpenloopRamp(
                        Constants.kElevatorRampRate, Constants.kLongCANTimeoutMs),
                "Could not set elevator voltage ramp rate: ");

        TalonSRXUtil.checkError(
                mMaster.configContinuousCurrentLimit(20, Constants.kLongCANTimeoutMs),
                "Could not set wrist continuous current limit.");

        TalonSRXUtil.checkError(
                mMaster.configPeakCurrentLimit(35, Constants.kLongCANTimeoutMs),
                "Could not set wrist peak current limit.");

        TalonSRXUtil.checkError(
                mMaster.configPeakCurrentDuration(200, Constants.kLongCANTimeoutMs),
                "Could not set wrist peak current duration.");
        mMaster.enableCurrentLimit(true);


        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        mMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

        // TODO add low gear gains

        mMaster.selectProfileSlot(0, 0);

        mMaster.overrideLimitSwitchesEnable(true);
        mMaster.overrideSoftLimitsEnable(false);

        mMaster.enableVoltageCompensation(true);

        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        mMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

        mMaster.setInverted(true);
        mMaster.setSensorPhase(true);

        mRightSlave = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorRightSlaveId,
                Constants.kElevatorMasterId);
        mRightSlave.setInverted(true);

        mLeftSlaveA = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveAId,
                Constants.kElevatorMasterId);
        mLeftSlaveA.setInverted(false);

        mLeftSlaveB = TalonSRXFactory.createPermanentSlaveTalon(Constants.kElevatorLeftSlaveBId,
                Constants.kElevatorMasterId);
        mLeftSlaveB.setInverted(false);

        mShifter = Constants.makeSolenoidForId(Constants.kElevatorShifterSolenoidId);
        mShifter.set(true);

        // Start with zero power.
        mMaster.set(ControlMode.PercentOutput, 0);
        setNeutralMode(NeutralMode.Brake);
    }

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    public synchronized void setOpenLoop(double percentage) {
        mElevatorControlState = ElevatorControlState.OPEN_LOOP;
        mPeriodicIO.demand = percentage;
    }

    public synchronized void setMotionMagicPosition(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
        double encoderPosition = positionInchesFromHome * kEncoderTicksPerInch;
        setClosedLoopRawPosition(encoderPosition);
    }

    public synchronized void setPositionPID(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - kHomePositionInches;
        double encoderPosition = positionInchesFromHome * kEncoderTicksPerInch;
        if (mElevatorControlState != ElevatorControlState.POSITION_PID) {
            mElevatorControlState = ElevatorControlState.POSITION_PID;
            mMaster.selectProfileSlot(kPositionControlSlot, 0);
        }
        mPeriodicIO.demand = encoderPosition;
    }

    private synchronized void setClosedLoopRawPosition(double encoderPosition) {
        if (mElevatorControlState != ElevatorControlState.MOTION_MAGIC) {
            mElevatorControlState = ElevatorControlState.MOTION_MAGIC;
            mMaster.selectProfileSlot(kHighGearSlot, 0);
        }
        mPeriodicIO.demand = encoderPosition;
    }

    public synchronized boolean hasFinishedTrajectory() {
        return mElevatorControlState == ElevatorControlState.MOTION_MAGIC &&
                Util.epsilonEquals(mPeriodicIO.active_trajectory_position, mPeriodicIO.demand, 5);
    }

    public synchronized void setHangMode(boolean hang_mode) {
        mShifter.set(!hang_mode);
    }

    public synchronized double getRPM() {
        // We are using a CTRE mag encoder which is 4096 native units per revolution.
        return mPeriodicIO.velocity_ticks_per_100ms * 10.0 / 4096.0 * 60.0;
    }

    public synchronized double getInchesOffGround() {
        return (mPeriodicIO.position_ticks / kEncoderTicksPerInch) + kHomePositionInches;
    }

    public synchronized double getSetpoint() {
        return mElevatorControlState == ElevatorControlState.MOTION_MAGIC ?
                mPeriodicIO.demand / kEncoderTicksPerInch + kHomePositionInches : Double.NaN;
    }

    public synchronized double getActiveTrajectoryAccelG() {
        return mPeriodicIO.active_trajectory_accel_g;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Elevator Output %", mPeriodicIO.output_percent);
        SmartDashboard.putNumber("Elevator RPM", getRPM());
        SmartDashboard.putNumber("Elevator Current", mMaster.getOutputCurrent());
        // SmartDashboard.putNumber("Elevator Error", mMaster.getClosedLoopError(0) / kEncoderTicksPerInch);
        SmartDashboard.putNumber("Elevator Height", getInchesOffGround());
        SmartDashboard.putBoolean("Elevator Limit", mPeriodicIO.limit_switch);
        SmartDashboard.putNumber("Elevator Sensor Height", mPeriodicIO.position_ticks);

        SmartDashboard.putNumber("Elevator Last Expected Trajectory", mPeriodicIO.demand);
        SmartDashboard.putNumber("Elevator Current Trajectory Point", mPeriodicIO.active_trajectory_position);
        SmartDashboard.putNumber("Elevator Traj Vel", mPeriodicIO.active_trajectory_velocity);
        SmartDashboard.putNumber("Elevator Traj Accel", mPeriodicIO.active_trajectory_accel_g);
        SmartDashboard.putBoolean("Elevator Has Sent Trajectory", hasFinishedTrajectory());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }

    public synchronized void resetIfAtLimit() {
        if (mPeriodicIO.limit_switch) {
            zeroSensors();
        }
    }

    private void setNeutralMode(NeutralMode neutralMode) {
        mLeftSlaveA.setNeutralMode(neutralMode);
        mLeftSlaveB.setNeutralMode(neutralMode);
        mMaster.setNeutralMode(neutralMode);
        mRightSlave.setNeutralMode(neutralMode);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        final double t = Timer.getFPGATimestamp();
        mPeriodicIO.position_ticks = mMaster.getSelectedSensorPosition(0);
        mPeriodicIO.velocity_ticks_per_100ms = mMaster.getSelectedSensorVelocity(0);
        if (mMaster.getControlMode() == ControlMode.MotionMagic) {
            mPeriodicIO.active_trajectory_position = mMaster.getActiveTrajectoryPosition();
            final int newVel = mMaster.getActiveTrajectoryVelocity();
            // TODO check sign of elevator accel
            if (Util.epsilonEquals(newVel, Constants.kElevatorHighGearCruiseVelocity, 5) ||
                    Util.epsilonEquals(newVel, mPeriodicIO.active_trajectory_velocity, 5)) {
                // Elevator is ~constant velocity.
                mPeriodicIO.active_trajectory_accel_g = 0.0;
            } else if (newVel > mPeriodicIO.active_trajectory_velocity) {
                // Elevator is accelerating downwards.
                mPeriodicIO.active_trajectory_accel_g = -Constants.kElevatorHighGearAcceleration * 10.0 /
                        (kEncoderTicksPerInch * 386.09);
            } else {
                // Elevator is accelerating upwards.
                mPeriodicIO.active_trajectory_accel_g = Constants.kElevatorHighGearAcceleration * 10.0 /
                        (kEncoderTicksPerInch * 386.09);
            }
            mPeriodicIO.active_trajectory_velocity = newVel;
        } else {
            mPeriodicIO.active_trajectory_position = Integer.MIN_VALUE;
            mPeriodicIO.active_trajectory_velocity = 0;
            mPeriodicIO.active_trajectory_accel_g = 0.0;
        }
        mPeriodicIO.output_percent = mMaster.getMotorOutputPercent();
        mPeriodicIO.limit_switch = mMaster.getSensorCollection().isFwdLimitSwitchClosed();
        mPeriodicIO.t = t;

        if (getInchesOffGround() > Constants.kElevatorEpsilon && mShifter.get()) {
            mPeriodicIO.feedforward = mIntake.hasCube() ? Constants.kElevatorFeedforwardWithCube : Constants
                    .kElevatorFeedforwardNoCube;
        } else {
            mPeriodicIO.feedforward = 0.0;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mElevatorControlState == ElevatorControlState.MOTION_MAGIC) {
            mMaster.set(ControlMode.MotionMagic,
                    mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        } else if (mElevatorControlState == ElevatorControlState.POSITION_PID) {
            mMaster.set(ControlMode.Position,
                    mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        } else {
            mMaster.set(ControlMode.PercentOutput,
                    mPeriodicIO.demand, DemandType.ArbitraryFeedForward, mPeriodicIO.feedforward);
        }
    }

    @Override
    public boolean checkSystem() {
        setNeutralMode(NeutralMode.Coast);
        setHangMode(true);

        boolean leftSide =
                TalonSRXChecker.CheckTalons(this,
                        new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                            {
                                add(new TalonSRXChecker.TalonSRXConfig("left_slave_a",
                                        mLeftSlaveA));
                                add(new TalonSRXChecker.TalonSRXConfig("left_slave_b",
                                        mLeftSlaveB));
                            }
                        }, new TalonSRXChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 200;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 2.0;
                                mRunOutputPercentage = -0.4;
                                mRPMSupplier = () -> -mMaster.getSelectedSensorVelocity(0);
                            }
                        });
        boolean rightSide =
                TalonSRXChecker.CheckTalons(this,
                        new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                            {
                                add(new TalonSRXChecker.TalonSRXConfig("master", mMaster));
                                add(new TalonSRXChecker.TalonSRXConfig("right_slave", mRightSlave));
                            }
                        }, new TalonSRXChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 200;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 2.0;
                                mRunOutputPercentage = -0.4;
                                mRPMSupplier = () -> -mMaster.getSelectedSensorVelocity(0);
                            }
                        });

        return leftSide && rightSide;
    }

    private enum ElevatorControlState {
        OPEN_LOOP,
        MOTION_MAGIC,
        POSITION_PID
    }

    public static class PeriodicIO {
        // INPUTS
        public int position_ticks;
        public int velocity_ticks_per_100ms;
        public double active_trajectory_accel_g;
        public int active_trajectory_velocity;
        public int active_trajectory_position;
        public double output_percent;
        public boolean limit_switch;
        public double feedforward;
        public double t;

        // OUTPUTS
        public double demand;
    }
}
