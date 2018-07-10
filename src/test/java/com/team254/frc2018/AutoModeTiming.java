package com.team254.frc2018;

import com.team254.frc2018.auto.AutoConstants;
import com.team254.frc2018.paths.TrajectoryGenerator;
import org.junit.jupiter.api.Test;

public class AutoModeTiming {
    TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
    boolean mStartedLeft = true;

    @Test
    void checkTiming() {
        mTrajectoryGenerator.generateTrajectories();

        double nearScaleDuration = mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().nearFenceToNearScale.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence2.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().nearFence2ToNearScale.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().nearScaleToNearFence3.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().nearFence3ToNearScale.get(mStartedLeft).getLastState().t();
        double nearScaleBestCaseWait = 3 * AutoConstants.kWaitForCubeTime;
        double nearScaleWorstCaseWait = nearScaleBestCaseWait + 0.5 * 4;


        double farScaleDuration = mTrajectoryGenerator.getTrajectorySet().sideStartToFarScale.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().farFenceToFarScale.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence2.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().farFence2ToFarScale.get(mStartedLeft).getLastState().t() +
                mTrajectoryGenerator.getTrajectorySet().farScaleToFarFence3.get(mStartedLeft).getLastState().t();
        double farScaleBestCaseWait = 2 * AutoConstants.kWaitForCubeTime;
        double farScaleWorstCaseWait = farScaleBestCaseWait + 0.5 * 3;

        System.out.println("Near Scale Only:");
        System.out.println("\tTrajectory Duration: " + nearScaleDuration);
        System.out.println("\tBest Case Wait Duration: " + nearScaleBestCaseWait);
        System.out.println("\tWorst Case Wait Duration: " + nearScaleWorstCaseWait);
        System.out.println("Far Scale Only:");
        System.out.println("\tTrajectory Duration: " + farScaleDuration);
        System.out.println("\tBest Case Wait Duration: " + farScaleBestCaseWait);
        System.out.println("\tWorst Case Wait Duration: " + farScaleWorstCaseWait);
    }
}
