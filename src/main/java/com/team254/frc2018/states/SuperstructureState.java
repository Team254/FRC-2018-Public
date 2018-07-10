package com.team254.frc2018.states;

import com.team254.lib.util.Util;

public class SuperstructureState {
    public double height = SuperstructureConstants.kElevatorMinHeight;
    public double angle = SuperstructureConstants.kWristMinAngle;
    public boolean jawClamped = true;

    // This isnt touched by planner
    public boolean hasCube = false;
    public boolean elevatorSentLastTrajectory = false;
    public boolean wristSentLastTrajectory = false;

    public SuperstructureState(double height, double angle, boolean jawClamped) {
        this.height = height;
        this.angle = angle;
        this.jawClamped = jawClamped;
    }

    public SuperstructureState(double height, double angle) {
        this(height, angle, true);
    }

    public SuperstructureState(SuperstructureState other) {
        this.height = other.height;
        this.angle = other.angle;
        this.jawClamped = other.jawClamped;
    }

    public SuperstructureState() {
        this(SuperstructureConstants.kElevatorMinHeight, SuperstructureConstants.kWristMinAngle, true);
    }

    public boolean inIllegalZone(boolean allowSmallErrors) {
        double kAllowableWristAngleError = allowSmallErrors ? 5.5 : 0;
        double kAllowableElevatorHeightError = allowSmallErrors ? 1 : 0;

        if (height >= SuperstructureConstants.kClearFirstStageMaxHeight + kAllowableElevatorHeightError &&
                angle < SuperstructureConstants.kClearFirstStageMinWristAngle - kAllowableWristAngleError) {
            return true;
        }

        return false;
    }

    public boolean inIllegalZone() {
        return inIllegalZone(false);
    }

    public boolean inIllegalJawZone() {
        return angle < SuperstructureConstants.kAlwaysNeedsJawClampMinAngle && !jawClamped;
    }

    public boolean isInRange(SuperstructureState otherState, double heightThreshold, double wristThreshold) {
        return Util.epsilonEquals(otherState.height, height, heightThreshold) &&
                Util.epsilonEquals(otherState.angle, angle, wristThreshold);

    }

    @Override
    public String toString() {
        return "" + height + " / " + angle + " / " + jawClamped + " / " + hasCube;
    }

}
