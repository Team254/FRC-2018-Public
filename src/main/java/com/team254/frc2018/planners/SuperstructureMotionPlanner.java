package com.team254.frc2018.planners;

import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.lib.util.Util;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {
    private boolean mUpwardsSubcommandEnabled = true;

    class SubCommand {
        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;
        public double mHeightThreshold = 1.0;
        public double mWristThreshold = 5.0;

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, mWristThreshold);
        }
    }

    class WaitForWristSafeSubcommand extends SubCommand {
        public WaitForWristSafeSubcommand(SuperstructureState endState) {
            super(endState);
            mWristThreshold = mWristThreshold + Math.max(0.0, mEndState.angle - SuperstructureConstants
                    .kClearFirstStageMinWristAngle);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, Double.POSITIVE_INFINITY, mWristThreshold);
        }
    }

    class WaitForElevatorSafeSubcommand extends SubCommand {
        public WaitForElevatorSafeSubcommand(SuperstructureState endState, SuperstructureState currentState) {
            super(endState);
            if (endState.height >= currentState.height) {
                mHeightThreshold = mHeightThreshold + Math.max(0.0, mEndState.height - SuperstructureConstants
                        .kClearFirstStageMaxHeight);
            } else {
                mHeightThreshold = mHeightThreshold + Math.max(0.0, SuperstructureConstants.kClearFirstStageMaxHeight
                        - mEndState.height);
            }
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForElevatorApproachingSubcommand extends SubCommand {
        public WaitForElevatorApproachingSubcommand(SuperstructureState endState) {
            super(endState);
            mHeightThreshold = SuperstructureConstants.kElevatorApproachingThreshold;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForFinalSetpointSubcommand extends SubCommand {
        public WaitForFinalSetpointSubcommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return currentState.elevatorSentLastTrajectory && currentState.wristSentLastTrajectory;
        }
    }

    protected SuperstructureState mCommandedState = new SuperstructureState();
    protected SuperstructureState mIntermediateCommandState = new SuperstructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();

    public synchronized boolean setDesiredState(SuperstructureState desiredStateIn, SuperstructureState currentState) {
        SuperstructureState desiredState = new SuperstructureState(desiredStateIn);

        // Limit illegal inputs.
        desiredState.angle = Util.limit(desiredState.angle, SuperstructureConstants.kWristMinAngle,
                SuperstructureConstants.kWristMaxAngle);
        desiredState.height = Util.limit(desiredState.height, SuperstructureConstants.kElevatorMinHeight,
                SuperstructureConstants.kElevatorMaxHeight);

        SuperstructureState swapJaw = new SuperstructureState(currentState);
        swapJaw.jawClamped = desiredState.jawClamped;

        // Immediate return, totally illegal commands.
        if (desiredState.inIllegalJawZone() || swapJaw.inIllegalJawZone()) {
            // Desired state is not legal.  Return false, let the caller deal with it.
            return false;
        }

        // Everything beyond this is probably do-able; clear queue
        mCommandQueue.clear();

        final boolean longUpwardsMove = desiredState.height - currentState.height > SuperstructureConstants
                .kElevatorLongRaiseDistance;
        final double firstWristAngle = longUpwardsMove ? Math.min(desiredState.angle, SuperstructureConstants
                .kStowedAngle) : desiredState.angle;

        if (currentState.angle < SuperstructureConstants.kClearFirstStageMinWristAngle && desiredState.height >
                SuperstructureConstants.kClearFirstStageMaxHeight) {
            // PRECONDITION: wrist is unsafe, want to go high
            // mCommandQueue.add(new WaitForWristSafeSubcommand(new SuperstructureState(SuperstructureConstants
            // .kClearFirstStageMaxHeight, Math.max(SuperstructureConstants
            //        .kClearFirstStageMinWristAngle, firstWristAngle), true)));
            // POSTCONDITION: wrist is safe (either at desired angle, or the cruise angle), elevator is as close as
            // possible to goal.
        } else if (desiredState.angle < SuperstructureConstants.kClearFirstStageMinWristAngle && currentState.height
                > SuperstructureConstants.kClearFirstStageMaxHeight) {
            // PRECONDITION: wrist is safe, want to go low.
//            mCommandQueue.add(new WaitForElevatorSafeSubcommand(new SuperstructureState(desiredState.height,
//                    SuperstructureConstants
//                    .kClearFirstStageMinWristAngle, true), currentState));
            // POSTCONDITION: elevator is safe, wrist is as close as possible to goal.
        }

        if (longUpwardsMove) {
            // PRECONDITION: wrist is safe, we are moving upwards.
            if (mUpwardsSubcommandEnabled) {
                mCommandQueue.add(new WaitForElevatorApproachingSubcommand(new SuperstructureState(desiredState.height,
                        firstWristAngle, true)));
            }
            // POSTCONDITION: elevator is approaching final goal.
        }

        // Go to the goal.
        mCommandQueue.add(new WaitForFinalSetpointSubcommand(desiredState));

        // Reset current command to start executing on next iteration
        mCurrentCommand = Optional.empty();

        return true; // this is a legal move
    }

    void reset(SuperstructureState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    public boolean isFinished(SuperstructureState currentState) {
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty() && currentState.wristSentLastTrajectory &&
                currentState.elevatorSentLastTrajectory;
    }

    public synchronized void setUpwardsSubcommandEnable(boolean enabled) {
        mUpwardsSubcommandEnabled = enabled;
    }

    public SuperstructureState update(SuperstructureState currentState) {
        if (!mCurrentCommand.isPresent() && !mCommandQueue.isEmpty()) {
            mCurrentCommand = Optional.of(mCommandQueue.remove());
        }

        if (mCurrentCommand.isPresent()) {
            SubCommand subCommand = mCurrentCommand.get();
            mIntermediateCommandState = subCommand.mEndState;
            if (subCommand.isFinished(currentState) && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome
                // unclear.
                mCurrentCommand = Optional.empty();
            }
        } else {
            mIntermediateCommandState = currentState;
        }

        mCommandedState.angle = Util.limit(mIntermediateCommandState.angle, SuperstructureConstants.kWristMinAngle,
                SuperstructureConstants.kWristMaxAngle);
        mCommandedState.height = Util.limit(mIntermediateCommandState.height, SuperstructureConstants
                .kElevatorMinHeight, SuperstructureConstants.kElevatorMaxHeight);

        return mCommandedState;
    }
}
