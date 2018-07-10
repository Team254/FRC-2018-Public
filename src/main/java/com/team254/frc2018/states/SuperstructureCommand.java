package com.team254.frc2018.states;

public class SuperstructureCommand {
    public double height = SuperstructureConstants.kElevatorMinHeight;
    public double wristAngle = SuperstructureConstants.kWristMinAngle;

    public boolean openLoopElevator = false;
    public double openLoopElevatorPercent = 0.0;

    public boolean elevatorLowGear = false;
    public boolean deployForklift = false;
}
