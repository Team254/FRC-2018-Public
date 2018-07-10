package com.team254.frc2018.states;

public class IntakeState {
    public enum JawState {
        OPEN,
        CLOSED,
        CLAMPED
    }

    public JawState jawState = JawState.CLOSED;
    public double leftMotor = 0;
    public double rightMotor = 0;
    public double wristAngle = 0; // needed to enforce clamping limits
    public double wristSetpoint = 0;

    public boolean leftCubeSensorTriggered = false;
    public boolean rightCubeSensorTriggered = false;

    // Kinda doesn't belong, but avoid cyclic dependencies...
    public boolean kickStandEngaged = true;

    public TimedLEDState ledState = TimedLEDState.StaticLEDState.kStaticOff;

    public void setPower(double power) {
        leftMotor = rightMotor = power;
    }

    public boolean seesCube() {
        return leftCubeSensorTriggered && rightCubeSensorTriggered;
    }
}
