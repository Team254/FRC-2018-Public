package com.team254.frc2018.subsystems;

import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.states.LEDState;
import com.team254.frc2018.states.TimedLEDState;

public class LED extends Subsystem {
    private static final double kHangingBlinkDuration = 0.5; // In sec
    private static final double kWantsCubeBlinkDuration = 0.075; // In sec
    private static final double kFaultBlinkDuration = 0.25; // In sec

    private static LED mInstance;

    private CarriageCanifier mCarriageCanifier;
    private SystemState mSystemState = SystemState.DISPLAYING_INTAKE;
    private WantedAction mWantedAction = WantedAction.DISPLAY_INTAKE;

    private Wrist mWrist;
    private Elevator mElevator;

    private boolean mFaultsEnabled = false;

    private LEDState mDesiredLEDState = new LEDState(0.0, 0.0, 0.0);
    private TimedLEDState mIntakeLEDState = TimedLEDState.StaticLEDState.kStaticOff;

    public synchronized static LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    private LED() {
        mCarriageCanifier = CarriageCanifier.getInstance();
    }

    public synchronized void setIntakeLEDState(TimedLEDState intakeLEDState) {
        mIntakeLEDState = intakeLEDState;
    }

    public synchronized void setWantedAction(WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            double stateStartTime;

            @Override
            public void onStart(double timestamp) {
                stateStartTime = timestamp;
                mWrist = Wrist.getInstance();
                mElevator = Elevator.getInstance();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LED.this) {
                    SystemState newState = getStateTransition();

                    if (mSystemState != newState) {
                        System.out.println(timestamp + ": LED changed state: " + mSystemState + " -> " + newState);
                        mSystemState = newState;
                        stateStartTime = timestamp;
                    }

                    double timeInState = timestamp - stateStartTime;

                    switch (mSystemState) {
                        case DISPLAYING_INTAKE:
                            setIntakeLEDCommand(timeInState);
                            break;
                        case DISPLAYING_FAULT:
                            setFaultLEDCommand(timeInState);
                            break;
                        case DISPLAYING_HANG:
                            setHangLEDCommand(timeInState);
                            break;
                        case DISPLAYING_WANTS_CUBE:
                            setBlinkLEDCommand(timeInState);
                            break;
                        default:
                            System.out.println("Fell through on LED commands: " + mSystemState);
                            break;
                    }
                    mCarriageCanifier.setLEDColor(mDesiredLEDState.red, mDesiredLEDState.green,
                            mDesiredLEDState.blue);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    private void setIntakeLEDCommand(double timeInState) {
        mIntakeLEDState.getCurrentLEDState(mDesiredLEDState, timeInState);
    }

    private void setFaultLEDCommand(double timeInState) {
        // Blink red.
        if ((int) (timeInState / kFaultBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kFault);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private void setHangLEDCommand(double timeInState) {
        // Blink orange.
        if ((int) (timeInState / kHangingBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kHanging);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private void setBlinkLEDCommand(double timeInState) {
        // Blink white.
        if ((int) (timeInState / kWantsCubeBlinkDuration) % 2 == 0) {
            mDesiredLEDState.copyFrom(LEDState.kWantsCube);
        } else {
            mDesiredLEDState.copyFrom(LEDState.kOff);
        }
    }

    private SystemState getStateTransition() {
        if (mFaultsEnabled && (!mWrist.hasBeenZeroed() || !mElevator.hasBeenZeroed())) {
            return SystemState.DISPLAYING_FAULT;
        }
        switch (mWantedAction) {
            case DISPLAY_HANG:
                return SystemState.DISPLAYING_HANG;
            case DISPLAY_INTAKE:
                return SystemState.DISPLAYING_INTAKE;
            case DISPLAY_WANTS_CUBE:
                return SystemState.DISPLAYING_WANTS_CUBE;
            default:
                System.out.println("Fell through on LED wanted action check: " + mWantedAction);
                return SystemState.DISPLAYING_INTAKE;
        }
    }

    public synchronized void setEnableFaults(boolean enable) {
        mFaultsEnabled = enable;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    public enum WantedAction {
        DISPLAY_HANG,
        DISPLAY_INTAKE,
        DISPLAY_WANTS_CUBE,
    }

    private enum SystemState {
        DISPLAYING_FAULT,
        DISPLAYING_INTAKE,
        DISPLAYING_HANG,
        DISPLAYING_WANTS_CUBE,
    }
}
