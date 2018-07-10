package com.team254.frc2018.controlboard;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    @Override
    public double getThrottle() {
        return mThrottleStick.getRawAxis(1);
    }

    @Override
    public double getTurn() {
        return -mTurnStick.getRawAxis(0);
    }

    @Override
    public boolean getPoopyShoot() {
        return mThrottleStick.getRawButton(1);
    }

    @Override
    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(1);
    }

    @Override
    public boolean getOpenJaw() {
        return mTurnStick.getRawButton(2);
    }

    @Override
    public boolean getShoot() {
        return mThrottleStick.getRawButton(2);
    }
}