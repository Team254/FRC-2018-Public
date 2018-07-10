package com.team254.util.test;

public class ControlledActuatorLinearSim {
    double mMinPosition, mMaxPosition, mVelocity;
    double mCommandedPosition = 0;
    double mCurrentPosition = 0;

    public ControlledActuatorLinearSim(double minPosition, double maxPosition, double velocity) {
        mMaxPosition = maxPosition;
        mMinPosition = minPosition;
        mVelocity = velocity;
        mCommandedPosition = mMinPosition;
        mCurrentPosition = mCommandedPosition;
    }

    public void setCommandedPosition(double commandedPosition) {
        mCommandedPosition = commandedPosition;
    }

    public double update(double timeStep) {
        boolean positiveError = (mCommandedPosition - mCurrentPosition) > 0;
        double amountToMove = (mVelocity * timeStep) * (positiveError ? 1 : -1);
        mCurrentPosition += amountToMove;
        boolean newPositiveError = (mCommandedPosition - mCurrentPosition) > 0;
        if (newPositiveError != positiveError) {
            mCurrentPosition = mCommandedPosition;
        }
        if (mCurrentPosition > mMaxPosition) {
            mCurrentPosition = mMaxPosition;
        }
        if (mCurrentPosition < mMinPosition) {
            mCurrentPosition = mMinPosition;
        }
        return mCurrentPosition;
    }

    public void reset() {
        reset(mMinPosition);
    }

    public void reset(double position) {
        mCurrentPosition = mCommandedPosition = position;
    }

}
