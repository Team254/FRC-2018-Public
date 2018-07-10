package com.team254.frc2018.statemachines;

import com.team254.frc2018.states.IntakeState;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.TimedLEDState;
import com.team254.lib.util.TimeDelayedBoolean;

public class IntakeStateMachine {
    public final static double kActuationTime = 0.0;
    public final static double kExchangeShootSetpoint = 1.0;
    public final static double kSwitchShootSetpoint = 0.65;
    public final static double kStrongShootSetpoint = 0.85;
    public final static double kWeakShootSetpoint = .5;
    public final static double kPoopyShootSetpoint = .40;
    public final static double kIntakeCubeSetpoint = -1.0;
    public final static double kHoldSetpoint = 0.0;
    public final static double kLostCubeTime = 0.25;
    public final static double kUnclampWaitingTime = 1.0;

    public enum WantedAction {
        WANT_MANUAL,
        WANT_CUBE,
    }

    private enum SystemState {
        OPEN_LOOP,
        KEEPING_CUBE,
    }

    private SystemState mSystemState = SystemState.OPEN_LOOP;
    private IntakeState mCommandedState = new IntakeState();
    private double mCurrentStateStartTime = 0;

    private TimeDelayedBoolean mLastSeenCube = new TimeDelayedBoolean();
    private double mLastSeenCubeTime = Double.NaN;

    private IntakeState.JawState mWantedJawState = IntakeState.JawState.CLAMPED;
    private double mWantedPower = 0.0;
    private boolean mForceClamp = false;

    public synchronized void setWantedJawState(final IntakeState.JawState jaw_state) {
        mWantedJawState = jaw_state;
    }

    public synchronized void forceClampJaw(boolean clamp) {
        mForceClamp = clamp;
    }

    public synchronized void setWantedPower(double power) {
        mWantedPower = power;
    }

    public IntakeState update(double timestamp, WantedAction wantedAction, IntakeState currentState) {
        synchronized (IntakeStateMachine.this) {
            SystemState newState;
            double timeInState = timestamp - mCurrentStateStartTime;

            // Handle state transitions
            switch (mSystemState) {
                case OPEN_LOOP:
                    newState = handleOpenLoopTransitions(wantedAction, currentState);
                    break;
                case KEEPING_CUBE:
                    newState = handleKeepingCubeTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected intake system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                System.out.println(timestamp + ": Intake changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = timestamp;
                mLastSeenCube.update(false, kLostCubeTime);
            }

            // Handle State outputs
            switch (mSystemState) {
                case OPEN_LOOP:
                    getOpenLoopCommandedState(currentState, mCommandedState);
                    break;
                case KEEPING_CUBE:
                    getKeepingCubeCommandedState(currentState, mCommandedState, timestamp);
                    break;
                default:
                    getOpenLoopCommandedState(currentState, mCommandedState);
                    break;
            }
        }
        return mCommandedState;
    }

    // OPEN_LOOP
    private synchronized SystemState handleOpenLoopTransitions(WantedAction wantedAction, IntakeState currentState) {
        switch (wantedAction) {
            case WANT_CUBE:
                mLastSeenCubeTime = Double.NaN;
                return SystemState.KEEPING_CUBE;
            default:
                return SystemState.OPEN_LOOP;
        }
    }

    private synchronized void getOpenLoopCommandedState(IntakeState currentState, IntakeState commandedState) {
        commandedState.setPower(mWantedPower);
        if (mustStayClosed(currentState)) {
            commandedState.jawState = (mWantedJawState == IntakeState.JawState.CLAMPED) ?
                    IntakeState.JawState.CLAMPED : IntakeState.JawState.CLOSED;
        } else {
            commandedState.jawState = mWantedJawState;
        }
        commandedState.ledState = TimedLEDState.StaticLEDState.kStaticOff;
    }

    // KEEP_CUBE
    private synchronized SystemState handleKeepingCubeTransitions(WantedAction wantedAction, IntakeState currentState) {
        switch (wantedAction) {
            case WANT_MANUAL:
                return SystemState.OPEN_LOOP;
            default:
                return SystemState.KEEPING_CUBE;
        }
    }

    private synchronized void getKeepingCubeCommandedState(IntakeState currentState, IntakeState commandedState, double timestamp) {
        commandedState.setPower(kIntakeCubeSetpoint);
        boolean clamp = (currentState.seesCube() && mWantedJawState != IntakeState.JawState.OPEN) || mustStayClosed(currentState);

        boolean currentlySeeCube = currentState.seesCube();
        boolean resetSeenCubeTime = true;
        if (!currentlySeeCube && !Double.isNaN(mLastSeenCubeTime) &&
                (timestamp - mLastSeenCubeTime < kLostCubeTime)) {
            currentlySeeCube = true;
            clamp = (mWantedJawState != IntakeState.JawState.OPEN) || mustStayClosed(currentState);
            resetSeenCubeTime = false;
        }

        boolean seenCube = mLastSeenCube.update(currentlySeeCube, kLostCubeTime);

        if (currentlySeeCube) {
            if (!seenCube) {
                commandedState.setPower(kIntakeCubeSetpoint);
            } else {
                commandedState.setPower(kHoldSetpoint);
            }
            commandedState.jawState = clamp ? IntakeState.JawState.CLAMPED : IntakeState.JawState.OPEN;
            commandedState.ledState = currentState.kickStandEngaged ?
                    TimedLEDState.StaticLEDState.kHasCube :
                    TimedLEDState.BlinkingLEDState.kHasCube;
            if (resetSeenCubeTime) {
                mLastSeenCubeTime = timestamp;
            }
        } else {
            commandedState.setPower(kIntakeCubeSetpoint);

            if (mForceClamp) {
                commandedState.jawState = IntakeState.JawState.CLAMPED;
            } else if (!Double.isNaN(mLastSeenCubeTime) &&
                    (timestamp - mLastSeenCubeTime < kUnclampWaitingTime)) {
                commandedState.jawState = IntakeState.JawState.CLAMPED;
            } else {
                commandedState.jawState = mustStayClosed(currentState) ? IntakeState.JawState.CLOSED : (mWantedJawState == IntakeState.JawState.OPEN ? IntakeState.JawState.OPEN : IntakeState.JawState.CLOSED);
            }

            commandedState.ledState = TimedLEDState.StaticLEDState.kIntaking;
        }
    }

    private boolean mustStayClosed(IntakeState state) {
        return state.wristSetpoint < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle ||
                state.wristAngle < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle;
    }
}
