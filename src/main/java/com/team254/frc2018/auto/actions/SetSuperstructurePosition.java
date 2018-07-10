package com.team254.frc2018.auto.actions;

import com.team254.frc2018.states.SuperstructureState;
import com.team254.frc2018.subsystems.Superstructure;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class SetSuperstructurePosition implements Action {
    private static final Superstructure mSuperstructure = Superstructure.getInstance();
    private static final double kHeightEpsilon = 2.0;
    private static final double kAngleEpsilon = 5.0;
    private static final double kTimeout = 4.0;

    private final double mHeight;
    private final double mAngle;
    private final boolean mWaitForCompletion;
    private double mStartTime;

    public SetSuperstructurePosition(double height, double angle, boolean waitForCompletion) {
        mHeight = height;
        // mHeight = SuperstructureConstants.kElevatorMinHeight;
        mAngle = angle;
        mWaitForCompletion = waitForCompletion;
    }

    @Override
    public void start() {
        mSuperstructure.setDesiredHeight(mHeight);
        mSuperstructure.setDesiredAngle(mAngle);
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        if (Timer.getFPGATimestamp() - mStartTime > kTimeout) {
            System.out.println("Set Superstructure Position timed out!!!");
            return true;
        }
        if (mWaitForCompletion) {
            SuperstructureState state = mSuperstructure.getObservedState();
            return Util.epsilonEquals(state.height, mHeight, kHeightEpsilon) &&
                    Util.epsilonEquals(state.angle, mAngle, kAngleEpsilon);
        } else {
            return true;
        }
    }

    @Override
    public void done() {
        System.out.println("Set Superstructure Position action finished");
    }
}
