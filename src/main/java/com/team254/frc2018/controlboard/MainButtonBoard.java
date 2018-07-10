package com.team254.frc2018.controlboard;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainButtonBoard implements IButtonControlBoard {
    private static MainButtonBoard mInstance = null;

    public static MainButtonBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainButtonBoard();
        }
        return mInstance;
    }

    private final Joystick mButtonBoard;

    private MainButtonBoard() {
        mButtonBoard = new Joystick(2);
    }

    // Wrist
    @Override
    public boolean goToIntakingWrist() {
        return mButtonBoard.getRawAxis(0) < Constants.kJoystickThreshold;
    }

    @Override
    public boolean goToScoringWrist() {
        return mButtonBoard.getRawButton(7);
    }

    @Override
    public boolean goToVerticalWrist() {
        return mButtonBoard.getRawAxis(1) < Constants.kJoystickThreshold;
    }

    @Override
    public boolean goToStowWrist() {
        return mButtonBoard.getRawAxis(2) < Constants.kJoystickThreshold;
    }

    @Override
    public boolean goToScoringAngledWrist() {
        return false;
    }

    // Elevator
    @Override
    public boolean getGoToHighScaleHeight() {
        return mButtonBoard.getRawButton(6);
    }

    @Override
    public boolean getGoToNeutralScaleHeight() {
        return mButtonBoard.getRawButton(5);
    }

    @Override
    public boolean getGoToLowScaleHeight() {
        return mButtonBoard.getRawButton(4);
    }

    @Override
    public boolean getGoToSwitchHeight() {
        return mButtonBoard.getRawButton(3);
    }

    @Override
    public boolean getGoToStowHeight() {
        return mButtonBoard.getRawButton(10);
    }

    @Override
    public boolean getBackwardsModifier() {
        return false;
    }

    @Override
    public boolean getAutoHeightModifier() {
        return mButtonBoard.getRawButton(8);
    }

    // Jog Elevator
    @Override
    public double getJogElevatorThrottle() {
        return mButtonBoard.getRawButton(1) ? 0.5 : (mButtonBoard.getRawButton(2) ? -0.5 : 0.0);
    }

    // Jog Wrist
    @Override
    public double getJogWristThrottle() {
        return mButtonBoard.getRawButton(11) ? 0.5 : (mButtonBoard.getRawButton(12) ? -0.5 : 0.0);
    }

    // Intake
    @Override
    public boolean getRunIntake() {
        return mButtonBoard.getRawButton(9);
    }

    @Override
    public boolean getIntakePosition() {
        return false;
    }

    @Override
    public void setRumble(boolean on) {

    }

    // Climbing
    @Override
    public boolean getEnableHangMode() {
        return false;
    }

    @Override
    public double getElevatorThrottle() {
        return 0.0;
    }

    @Override
    public boolean getDeployForks() {
        return false;
    }

    @Override
    public boolean getElevatorLowShift() {
        return false;
    }

    @Override
    public boolean getElevatorHighShift() {
        return false;
    }

    @Override
    public boolean getWantsCubeLEDBlink() {
        return false;
    }

    @Override
    public boolean getToggleKickstand() {
        return false;
    }
}
