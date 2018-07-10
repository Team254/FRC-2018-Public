package com.team254.frc2018.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.states.IntakeState;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;

public class Intake extends Subsystem {
    private final static boolean kClosed = false;
    private final static boolean kClamped = false;

    private static Intake mInstance;
    private final Solenoid mCloseSolenoid, mClampSolenoid; //open->false, false; close->true, false; clamp->true, true;
    private final TalonSRX mLeftMaster, mRightMaster;
    private final CarriageCanifier mCanifier = CarriageCanifier.getInstance();
    private final LED mLED = LED.getInstance();
    private IntakeStateMachine.WantedAction mWantedAction = IntakeStateMachine.WantedAction.WANT_MANUAL;
    private IntakeState.JawState mJawState;
    private IntakeState mCurrentState = new IntakeState();
    private IntakeStateMachine mStateMachine = new IntakeStateMachine();
    private boolean mKickStandEngaged = true;

    private Intake() {
        mCloseSolenoid = Constants.makeSolenoidForId(Constants.kIntakeCloseSolenoid);
        mClampSolenoid = Constants.makeSolenoidForId(Constants.kIntakeClampSolenoid);

        mLeftMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeLeftMasterId);
        mLeftMaster.set(ControlMode.PercentOutput, 0);
        mLeftMaster.setInverted(true);
        mLeftMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mLeftMaster.enableVoltageCompensation(true);

        mRightMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeRightMasterId);
        mRightMaster.set(ControlMode.PercentOutput, 0);
        mRightMaster.setInverted(false);
        mRightMaster.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
        mRightMaster.enableVoltageCompensation(true);
    }

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("LeftBanner", getLeftBannerSensor());
        SmartDashboard.putBoolean("RightBanner", getRightBannerSensor());
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    private IntakeState getCurrentState() {
        mCurrentState.leftCubeSensorTriggered = getLeftBannerSensor();
        mCurrentState.rightCubeSensorTriggered = getRightBannerSensor();
        mCurrentState.wristAngle = Wrist.getInstance().getAngle();
        mCurrentState.wristSetpoint = Wrist.getInstance().getSetpoint();
        mCurrentState.kickStandEngaged = mKickStandEngaged;
        return mCurrentState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        Loop loop = new Loop() {


            @Override
            public void onStart(double timestamp) {
                if (hasCube()) {
                    mWantedAction = IntakeStateMachine.WantedAction.WANT_CUBE;
                } else {
                    mWantedAction = IntakeStateMachine.WantedAction.WANT_MANUAL;
                }

            }

            @Override
            public void onLoop(double timestamp) {

                synchronized (Intake.this) {
                    IntakeState newState = mStateMachine.update(Timer.getFPGATimestamp(), mWantedAction,
                            getCurrentState());
                    updateActuatorFromState(newState);
                }

            }

            @Override
            public void onStop(double timestamp) {
                mWantedAction = IntakeStateMachine.WantedAction.WANT_MANUAL;
                // Set the states to what the robot falls into when disabled.
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    private void setJaw(IntakeState.JawState state) {
        if (mJawState == state) {
            return;
        }
        mJawState = state;
        switch (mJawState) {
            case OPEN:
                mCloseSolenoid.set(!kClosed);
                mClampSolenoid.set(!kClamped);
                break;
            case CLOSED:
                mCloseSolenoid.set(kClosed);
                mClampSolenoid.set(!kClamped);
                break;
            case CLAMPED:
                mCloseSolenoid.set(kClosed);
                mClampSolenoid.set(kClamped);
                break;
        }
    }

    private synchronized void updateActuatorFromState(IntakeState state) {
        mLeftMaster.set(ControlMode.PercentOutput, state.leftMotor);
        mRightMaster.set(ControlMode.PercentOutput, state.rightMotor);
        setJaw(state.jawState);

        mLED.setIntakeLEDState(state.ledState);
    }

    public synchronized boolean hasCube() {
        return getLeftBannerSensor() || getRightBannerSensor();
    }

    public synchronized boolean definitelyHasCube() {
        return getLeftBannerSensor() && getRightBannerSensor();
    }

    private boolean getLeftBannerSensor() {
        return mCanifier.getLeftBannerSensor();
    }

    private boolean getRightBannerSensor() {
        return mCanifier.getRightBannerSensor();
    }

    public IntakeState.JawState getJawState() {
        return mJawState;
    }

    public synchronized void setState(IntakeStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    public synchronized void setPower(double power) {
        mStateMachine.setWantedPower(power);
    }

    public synchronized void shoot(double power) {
        setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
        setPower(power);
    }

    public IntakeStateMachine.WantedAction getWantedAction() {
        return mWantedAction;
    }

    public synchronized void getOrKeepCube() {
        setState(IntakeStateMachine.WantedAction.WANT_CUBE);
    }

    public synchronized void tryOpenJaw() {
        mStateMachine.setWantedJawState(IntakeState.JawState.OPEN);
    }

    public synchronized void clampJaw() {
        mStateMachine.setWantedJawState(IntakeState.JawState.CLAMPED);
    }

    public synchronized void forceClampJaw(boolean clamp) {
        mStateMachine.forceClampJaw(clamp);
    }

    public synchronized void closeJaw() {
        mStateMachine.setWantedJawState(IntakeState.JawState.CLOSED);
    }

    public synchronized void setKickStand(boolean engaged) {
        mKickStandEngaged = engaged;
    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public void writePeriodicOutputs() {
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.CheckTalons(this,
                new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
                    {

                        add(new TalonSRXChecker.TalonSRXConfig("intake right master", mRightMaster));
                        add(new TalonSRXChecker.TalonSRXConfig("intake left master", mLeftMaster));
                    }
                }, new TalonSRXChecker.CheckerConfig() {
                    {
                        mCurrentFloor = 2;
                        mCurrentEpsilon = 2.0;
                        mRPMSupplier = null;
                    }
                });
    }
}

