package com.team254.frc2018.auto.actions;

import com.team254.frc2018.RobotState;
import com.team254.lib.geometry.Translation2d;

public class WaitUntilInsideRegion implements Action {
    private final static RobotState mRobotState = RobotState.getInstance();

    private final Translation2d mBottomLeft;
    private final Translation2d mTopRight;

    //(100, 100) (200, 200)
    public WaitUntilInsideRegion(Translation2d bottomLeft, Translation2d topRight, boolean isOnLeft) {
        if (isOnLeft) {
            mBottomLeft = new Translation2d(bottomLeft.x(), -topRight.y());
            mTopRight = new Translation2d(topRight.x(), -bottomLeft.y());
        } else {
            mBottomLeft = bottomLeft;
            mTopRight = topRight;
        }
    }

    @Override
    public boolean isFinished() {
        Translation2d position = mRobotState.getLatestFieldToVehicle().getValue().getTranslation();
        return position.x() > mBottomLeft.x() && position.x() < mTopRight.x()
                && position.y() > mBottomLeft.y() && position.y() < mTopRight.y();
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {

    }
}
