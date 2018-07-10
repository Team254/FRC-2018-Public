package com.team254.frc2018.statemachines;

import com.team254.frc2018.planners.SuperstructureMotionPlanner;
import com.team254.frc2018.states.SuperstructureCommand;
import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.Elevator;
import com.team254.lib.util.Util;

public class SuperstructureStateMachine {
    public enum WantedAction {
        IDLE,
        GO_TO_POSITION,
        WANT_MANUAL,
    }

    public enum SystemState {
        HOLDING_POSITION,
        MOVING_TO_POSITION,
        MANUAL
    }

    private SystemState mSystemState = SystemState.HOLDING_POSITION;

    private SuperstructureCommand mCommand = new SuperstructureCommand();
    private SuperstructureState mCommandedState = new SuperstructureState();
    private SuperstructureState mDesiredEndState = new SuperstructureState();

    private SuperstructureMotionPlanner mPlanner = new SuperstructureMotionPlanner();

    private double mScoringHeight = Elevator.kHomePositionInches;
    private double mScoringAngle = SuperstructureConstants.kStowedPositionAngle;

    private double mOpenLoopPower = 0.0;
    private boolean mManualWantsLowGear = false;
    private double mMaxHeight = SuperstructureConstants.kElevatorMaxHeight;

    public synchronized void resetManual() {
        mOpenLoopPower = 0.0;
        mManualWantsLowGear = false;
    }

    public synchronized void setMaxHeight(double height) {
        mMaxHeight = height;
    }

    public synchronized void setManualWantsLowGear(boolean wantsLowGear) {
        mManualWantsLowGear = wantsLowGear;
    }

    public synchronized void setOpenLoopPower(double power) {
        mOpenLoopPower = power;
    }

    public synchronized void setScoringHeight(double inches) {
        mScoringHeight = inches;
    }

    public synchronized double getScoringHeight() {
        return mScoringHeight;
    }

    public synchronized void setScoringAngle(double angle) {
        mScoringAngle = angle;
    }

    public synchronized double getScoringAngle() {
        return mScoringAngle;
    }

    public synchronized void jogElevator(double relative_inches) {
        mScoringHeight += relative_inches;
        mScoringHeight = Math.min(mScoringHeight, mMaxHeight);
        mScoringHeight = Math.max(mScoringHeight, SuperstructureConstants.kElevatorMinHeight);
    }

    public synchronized void jogWrist(double relative_degrees) {
        mScoringAngle += relative_degrees;
        mScoringAngle = Math.min(mScoringAngle, SuperstructureConstants.kWristMaxAngle);
        mScoringAngle = Math.max(mScoringAngle, SuperstructureConstants.kWristMinAngle);
    }

    public synchronized boolean scoringPositionChanged() {
        return !Util.epsilonEquals(mDesiredEndState.angle, mScoringAngle) ||
                !Util.epsilonEquals(mDesiredEndState.height, mScoringHeight);
    }

    public synchronized SystemState getSystemState() {
        return mSystemState;
    }

    public synchronized void setUpwardsSubcommandEnable(boolean enabled) {
        mPlanner.setUpwardsSubcommandEnable(enabled);
    }

    public synchronized SuperstructureCommand update(double timestamp, WantedAction wantedAction,
                                                     SuperstructureState currentState) {
        synchronized (SuperstructureStateMachine.this) {
            SystemState newState;

            // Handle state transitions
            switch (mSystemState) {
                case HOLDING_POSITION:
                    newState = handleHoldingPositionTransitions(wantedAction, currentState);
                    break;
                case MOVING_TO_POSITION:
                    newState = handleMovingToPositionTransitions(wantedAction, currentState);
                    break;
                case MANUAL:
                    newState = handleManualTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexpected superstructure system state: " + mSystemState);
                    newState = mSystemState;
                    break;
            }

            if (newState != mSystemState) {
                System.out.println(timestamp + ": Superstructure changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
            }

            // Pump elevator planner only if not jogging.
            if (!mCommand.openLoopElevator) {
                mCommandedState = mPlanner.update(currentState);
                mCommand.height = Math.min(mCommandedState.height, mMaxHeight);
                mCommand.wristAngle = mCommandedState.angle;
            }

            // Handle state outputs
            switch (mSystemState) {
                case HOLDING_POSITION:
                    getHoldingPositionCommandedState();
                    break;
                case MOVING_TO_POSITION:
                    getMovingToPositionCommandedState();
                    break;
                case MANUAL:
                    getManualCommandedState();
                    break;
                default:
                    System.out.println("Unexpected superstructure state output state: " + mSystemState);
                    break;
            }

            return mCommand;
        }
    }

    private void updateMotionPlannerDesired(SuperstructureState currentState) {
        mDesiredEndState.angle = mScoringAngle;
        mDesiredEndState.height = mScoringHeight;

        System.out.println("Setting motion planner to height: " + mDesiredEndState.height
                + " angle: " + mDesiredEndState.angle);

        // Push into elevator planner.
        if (!mPlanner.setDesiredState(mDesiredEndState, currentState)) {
            System.out.println("Unable to set elevator planner!");
        }

        mScoringAngle = mDesiredEndState.angle;
        mScoringHeight = mDesiredEndState.height;
    }

    private SystemState handleDefaultTransitions(WantedAction wantedAction, SuperstructureState currentState) {
        if (wantedAction == WantedAction.GO_TO_POSITION) {
            if (scoringPositionChanged()) {
                updateMotionPlannerDesired(currentState);
            } else if (mPlanner.isFinished(currentState)) {
                return SystemState.HOLDING_POSITION;
            }
            return SystemState.MOVING_TO_POSITION;
        } else if (wantedAction == WantedAction.WANT_MANUAL) {
            return SystemState.MANUAL;
        } else {
            if (mSystemState == SystemState.MOVING_TO_POSITION && !mPlanner.isFinished(currentState)) {
                return SystemState.MOVING_TO_POSITION;
            } else {
                return SystemState.HOLDING_POSITION;
            }
        }
    }

    // HOLDING_POSITION
    private SystemState handleHoldingPositionTransitions(WantedAction wantedAction,
                                                         SuperstructureState currentState) {
        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void getHoldingPositionCommandedState() {
        mCommand.elevatorLowGear = false;
        mCommand.openLoopElevator = false;
    }

    // MOVING_TO_POSITION
    private SystemState handleMovingToPositionTransitions(WantedAction wantedAction,
                                                          SuperstructureState currentState) {

        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void getMovingToPositionCommandedState() {
        mCommand.elevatorLowGear = false;
        mCommand.openLoopElevator = false;
    }

    // MANUAL
    private SystemState handleManualTransitions(WantedAction wantedAction,
                                                SuperstructureState currentState) {
        if (wantedAction != WantedAction.WANT_MANUAL) {
            // Freeze height.
            mScoringAngle = currentState.angle;
            mScoringHeight = currentState.height;
            return handleDefaultTransitions(WantedAction.GO_TO_POSITION, currentState);
        }
        return handleDefaultTransitions(wantedAction, currentState);
    }

    private void getManualCommandedState() {
        mCommand.elevatorLowGear = mManualWantsLowGear;
        mCommand.wristAngle = SuperstructureConstants.kWristMinAngle;
        mCommand.openLoopElevator = true;
        mCommand.openLoopElevatorPercent = mOpenLoopPower;
    }
}
