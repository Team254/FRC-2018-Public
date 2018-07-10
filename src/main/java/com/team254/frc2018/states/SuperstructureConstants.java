package com.team254.frc2018.states;

public class SuperstructureConstants {
    public static final double kWristMinAngle = 0.0;
    public static final double kWristMaxAngle = 180.0;
    public static final double kElevatorMaxHeight = 85.0;
    public static final double kElevatorMaxHeightKickEngaged = 78.0;
    public static final double kElevatorMinHeight = 4.0;

    public static final double kClearFirstStageMaxHeight = 33.0;
    public static final double kClearFirstStageMinWristAngle = 45.0;

    public static final double kAlwaysNeedsJawClampMinAngle = kClearFirstStageMinWristAngle;

    public static final double kElevatorLongRaiseDistance = 28.0;
    public static final double kElevatorApproachingThreshold = 12.0;

    public final static double kStowedAngle = 90.0;

    // This is in inches / ~20ms
    public final static double kElevatorJogThrottle = 60.0 / 50.0;

    // This is in degrees / ~20ms
    public final static double kWristJogThrottle = 200.0 / 25.0;

    // In inches, the height to use the kPlacingHighAngle.
    public final static double kPlacingHighThreshold = 33.0;

    // Presets.

    // Combinations.
    public final static double kStowedPositionHeight = 0.0;
    public final static double kStowedPositionAngle = 0.0;

    public final static double kIntakePositionHeight = 5.75;
    public final static double kIntakePositionAngle = 180.0;

    // Elevator Heights.
    public final static double kScaleHighHeight = 78.0;
    public final static double kScaleNeutralHeight = 78.0;
    public final static double kScaleLowHeight = 66.0;

    public final static double kScaleHighHeightNoKick = 85.0;
    public final static double kScaleNeutralHeightNoKick = 75.0;
    public final static double kScaleLowHeightNoKick = 63.0;

    public final static double kScaleHighHeightBackwards = 78.0; //75.0 without kickstand
    public final static double kScaleNeutralHeightBackwards = 70.0; //65.0 without kickstand
    public final static double kScaleLowHeightBackwards = 60.0; //55.0 without kickstand

    public final static double kScaleHighHeightBackwardsNoKick = 65.0;
    public final static double kScaleNeutralHeightBackwardsNoKick = 53.0;
    public final static double kScaleLowHeightBackwardsNoKick = 45.0;

    public final static double kCubeOffset = 7.0;

    public final static double kIntakeThirdLevelHeight = 25.5;
    public final static double kIntakeSecondLevelHeight = 14.5;
    public final static double kIntakeFloorLevelHeight = 0.0;

    public final static double kSwitchHeight = 30.0;
    public final static double kSwitchHeightBackwards = 27.0;

    // Wrist Angles.
    public final static double kVerticalAngle = 90.0;
    public final static double kScoreBackwardsAngle = 0.0; //45.0 without kickstand
    public final static double kScoreBackwardsAngleNoKick = 45.0;
    public final static double kScoreForwardAngledAngle = 135.0;
    public final static double kScoreSwitchBackwardsAngle = 0.0;

    public final static double kPlacingLowAngle = 175.0;
    public final static double kPlacingHighAngle = 175.0;
    public final static double kWeakShootAngle = 130.0;

    public final static double kMinIntakePositionAngle = 135.0;
}
