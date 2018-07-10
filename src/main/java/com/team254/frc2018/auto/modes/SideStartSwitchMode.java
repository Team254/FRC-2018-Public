package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.*;
import com.team254.frc2018.paths.TrajectoryGenerator;
import com.team254.frc2018.states.SuperstructureConstants;

import java.util.Arrays;

public class SideStartSwitchMode extends AutoModeBase {

    final boolean mGoLeft;
    final boolean mStartedLeft;
    private DriveTrajectory mTrajectory;

    public SideStartSwitchMode(boolean robotStartedOnLeft, boolean switchIsLeft) {
        mStartedLeft = robotStartedOnLeft;
        mGoLeft = switchIsLeft;

        if (mGoLeft == mStartedLeft) {
            mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToNearSwitch.get(mStartedLeft), true);
        } else {
            mTrajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().sideStartToFarSwitch.get(mStartedLeft), true);
        }
    }


    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Simple switch");
        runAction(new SetIntaking(false, false));

        runAction(new ParallelAction(
                Arrays.asList(
                        mTrajectory,
                        (new SetSuperstructurePosition(SuperstructureConstants.kSwitchHeight, SuperstructureConstants.kStowedPositionAngle, true))
                )
        ));

        runAction(new ShootCube(AutoConstants.kMediumShootPower));
    }
}
