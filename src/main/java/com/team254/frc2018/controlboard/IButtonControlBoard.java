package com.team254.frc2018.controlboard;

public interface IButtonControlBoard {
    // Wrist
    boolean goToIntakingWrist();

    boolean goToScoringWrist();

    boolean goToVerticalWrist();

    boolean goToStowWrist();

    boolean goToScoringAngledWrist();

    // Elevator
    boolean getGoToHighScaleHeight();

    boolean getGoToNeutralScaleHeight();

    boolean getGoToLowScaleHeight();

    boolean getGoToSwitchHeight();

    boolean getGoToStowHeight();

    boolean getBackwardsModifier();

    boolean getAutoHeightModifier();

    // Jog Elevator
    double getJogElevatorThrottle();

    // Jog Wrist
    double getJogWristThrottle();

    // Intake
    boolean getRunIntake();

    boolean getIntakePosition();

    void setRumble(boolean on);

    // Climbing
    boolean getEnableHangMode();

    double getElevatorThrottle();

    boolean getDeployForks();

    boolean getElevatorLowShift();

    boolean getElevatorHighShift();

    // LED
    boolean getWantsCubeLEDBlink();

    boolean getToggleKickstand();
}