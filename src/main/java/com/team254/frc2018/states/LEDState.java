package com.team254.frc2018.states;

public class LEDState {
    public static final LEDState kOff = new LEDState(0.0, 0.0, 0.0);

    public static final LEDState kIntakeOpenLoop = new LEDState(0.0, 0.0, 0.0);
    public static final LEDState kIntakeHasCube = new LEDState(1.0, 0.0, 0.0);
    public static final LEDState kIntakeIntaking = new LEDState(0.0, 1.0, 0.0);

    public static final LEDState kFault = new LEDState(0.0, 0.0, 1.0);

    public static final LEDState kHanging = new LEDState(0.0, 0.3, 1.0);

    public static final LEDState kWantsCube = new LEDState(0.0, 0.25, 1.0);

    public LEDState() {
    }

    public LEDState(double b, double g, double r) {
        blue = b;
        green = g;
        red = r;
    }

    public void copyFrom(LEDState other) {
        this.blue = other.blue;
        this.green = other.green;
        this.red = other.red;
    }

    public double blue;
    public double green;
    public double red;
}
