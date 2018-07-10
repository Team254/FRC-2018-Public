package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Drive;
import com.team254.lib.util.DriveSignal;
import edu.wpi.first.wpilibj.Timer;

public class ShiftHighGearAction implements Action {
    private static final double kTime = 2.0;
    private static final double kPower = 0.5;
    private static final Drive mDrive = Drive.getInstance();

    private final boolean mReverse;

    private double mStartTime = 0.0;

    public ShiftHighGearAction(boolean reverse) {
        mReverse = reverse;
    }

    @Override
    public void start() {
        mDrive.setHighGear(true);
        mDrive.setOpenLoop(new DriveSignal((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * kPower));
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kTime;
    }

    @Override
    public void done() {
        mDrive.setOpenLoop(DriveSignal.BRAKE);
    }
}
