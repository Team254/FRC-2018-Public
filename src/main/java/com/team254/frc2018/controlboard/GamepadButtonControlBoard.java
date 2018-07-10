package com.team254.frc2018.controlboard;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private Joystick mJoystick;

    private GamepadButtonControlBoard() {
        mJoystick = new Joystick(Constants.kButtonGamepadPort);
    }

    //Wrist
    @Override
    public boolean goToIntakingWrist() {
        return mJoystick.getPOV() == 90;
    }

    @Override
    public boolean goToScoringWrist() {
        return mJoystick.getPOV() == 180;
    }

    @Override
    public boolean goToVerticalWrist() {
        return mJoystick.getPOV() == 0;
    }

    @Override
    public boolean goToStowWrist() {
        return mJoystick.getPOV() == 270;
    }

    @Override
    public boolean goToScoringAngledWrist() {
        return mJoystick.getPOV() == 45;
    }

    //Elevator
    @Override
    public boolean getGoToHighScaleHeight() {
        return mJoystick.getRawButton(4);
    }

    @Override
    public boolean getGoToNeutralScaleHeight() {
        return mJoystick.getRawButton(2);
    }

    @Override
    public boolean getGoToLowScaleHeight() {
        return mJoystick.getRawButton(1);
    }

    @Override
    public boolean getGoToSwitchHeight() {
        return mJoystick.getRawButton(3);
    }

    @Override
    public boolean getGoToStowHeight() {
        return mJoystick.getRawButton(6);
    }

    @Override
    public boolean getBackwardsModifier() {
        return mJoystick.getRawAxis(3) > Constants.kJoystickThreshold;
    }

    @Override
    public boolean getAutoHeightModifier() {
        return false;
    }

    //Jog Elevator
    @Override
    public double getJogElevatorThrottle() {
        return -mJoystick.getRawAxis(5);
    }

    //Jog Wrist
    @Override
    public double getJogWristThrottle() {
        return mJoystick.getRawAxis(0);
    }

    //Intake
    @Override
    public boolean getIntakePosition() {
        return mJoystick.getRawAxis(2) > Constants.kJoystickThreshold;
    }

    @Override
    public boolean getRunIntake() {
        return mJoystick.getRawButton(5);
    }

    @Override
    public void setRumble(boolean on) {
        mJoystick.setRumble(GenericHID.RumbleType.kRightRumble, on ? 1.0 : 0);
    }

    //Climbing
    @Override
    public boolean getEnableHangMode() {
        return mJoystick.getRawButton(8) && mJoystick.getRawButton(7);
    }

    @Override
    public double getElevatorThrottle() {
        return mJoystick.getRawAxis(5);
    }

    @Override
    public boolean getDeployForks() {
        return mJoystick.getRawButton(5) && mJoystick.getRawButton(6);
    }

    @Override
    public boolean getElevatorLowShift() {
        return mJoystick.getRawButton(3);
    }

    @Override
    public boolean getElevatorHighShift() {
        return mJoystick.getRawButton(4);
    }

    @Override
    public boolean getWantsCubeLEDBlink() {
        return mJoystick.getRawButton(8);
    }

    @Override
    public boolean getToggleKickstand() {
        return mJoystick.getRawButton(7);
    }
}