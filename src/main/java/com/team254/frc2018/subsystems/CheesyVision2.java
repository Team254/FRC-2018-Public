package com.team254.frc2018.subsystems;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.AutoFieldState.Side;
import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.ILooper;
import com.team254.frc2018.loops.Loop;
import com.team254.frc2018.states.SuperstructureConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles things related to scale angle detection.
 */
public class CheesyVision2 extends Subsystem {
    public static final double kSmallThreshold = 2.7;
    public static final double kLargeThreshold = 4.0;

    private static CheesyVision2 mInstance;

    public synchronized static CheesyVision2 getInstance() {
        if (mInstance == null) {
            mInstance = new CheesyVision2();
        }
        return mInstance;
    }

    public enum ScaleHeight {
        LOW,
        NEUTRAL,
        HIGH
    }

    private CheesyVision2() {
    }

    private double mAngle = 0.0;
    private double mTip = 0.0;
    private boolean mError = true;
    private double mLastHeartbeatValue = -1;
    private double mLastHeartbeatTime = Double.NEGATIVE_INFINITY;
    private ScaleHeight mFilteredHeight = ScaleHeight.HIGH;


    /**
     * @return true if the robot is receiving data from the scale tracker
     */
    public synchronized boolean isConnected() {
        return Timer.getFPGATimestamp() < mLastHeartbeatTime + Constants.kScaleTrackerTimeout &&
                !Double.isNaN(mAngle);
    }

    /**
     * @return true if the system doesn't have a good reading of the scale
     */
    public boolean getError() {
        return mError;
    }

    /**
     * @return the current angle of the scale, in degrees, as seen
     * from the driver station (positive = right side raised;
     * negative = left side raised)
     */
    public double getAngle() {
        return mAngle;
    }

    /**
     * @return the current thresholded "height" of the <em>right</em> scale
     * plate, as seen from the driver station (-1, 0, or +1)
     */
    public double getTip() {
        return mTip;
    }

    public synchronized double getDesiredHeight(boolean backwards, int cubeNum, boolean useKickstand) {
        double baseHeight;

        if (useKickstand) {
            if (mFilteredHeight == ScaleHeight.LOW) {
                baseHeight = backwards ? SuperstructureConstants.kScaleLowHeightBackwards : SuperstructureConstants.kScaleLowHeight;
            } else if (mFilteredHeight == ScaleHeight.NEUTRAL) {
                baseHeight = backwards ? SuperstructureConstants.kScaleNeutralHeightBackwards : SuperstructureConstants.kScaleNeutralHeight;
            } else {
                baseHeight = backwards ? SuperstructureConstants.kScaleHighHeightBackwards : SuperstructureConstants.kScaleHighHeight;
            }
        } else {
            if (mFilteredHeight == ScaleHeight.LOW) {
                baseHeight = backwards ? SuperstructureConstants.kScaleLowHeightBackwardsNoKick : SuperstructureConstants.kScaleLowHeightNoKick;
            } else if (mFilteredHeight == ScaleHeight.NEUTRAL) {
                baseHeight = backwards ? SuperstructureConstants.kScaleNeutralHeightBackwardsNoKick : SuperstructureConstants.kScaleNeutralHeightNoKick;
            } else {
                baseHeight = backwards ? SuperstructureConstants.kScaleHighHeightBackwardsNoKick : SuperstructureConstants.kScaleHighHeightNoKick;
            }
        }

        //assume we are losing the scale if we don't have a reading
        if (!isConnected()) { // check for error as well?
            if (useKickstand) {
                baseHeight = backwards ? SuperstructureConstants.kScaleHighHeightBackwards : SuperstructureConstants.kScaleHighHeight;
            } else {
                baseHeight = backwards ? SuperstructureConstants.kScaleHighHeightBackwardsNoKick : SuperstructureConstants.kScaleHighHeightNoKick;
            }
        }

        baseHeight += cubeNum * SuperstructureConstants.kCubeOffset;

        if (useKickstand) {
            baseHeight = Math.min(baseHeight, SuperstructureConstants.kElevatorMaxHeightKickEngaged);
        } else {
            baseHeight = Math.min(baseHeight, SuperstructureConstants.kElevatorMaxHeight);
        }

        return baseHeight;
    }

    private synchronized ScaleHeight getNewFilteredScaleHeight() {
        // If we are not connected, we are just HIGH
        if (!isConnected() || Double.isNaN(mAngle)) {
            return ScaleHeight.HIGH;
        }

        double corrected_angle =
                mAngle * (AutoFieldState.getInstance().getScaleSide() == Side.RIGHT ? 1.0 : -1.0);

        // Always latch upwards.
        switch (mFilteredHeight) {
            case HIGH:
                if (corrected_angle < -kSmallThreshold) {
                    return ScaleHeight.LOW;
                } else if (corrected_angle < kSmallThreshold) {
                    return ScaleHeight.NEUTRAL;
                } else {
                    return ScaleHeight.HIGH;
                }
            case NEUTRAL:
                if (corrected_angle < -kLargeThreshold) {
                    return ScaleHeight.LOW;
                } else if (corrected_angle > kSmallThreshold) {
                    return ScaleHeight.HIGH;
                } else {
                    return ScaleHeight.NEUTRAL;
                }
            case LOW:
                if (corrected_angle > kSmallThreshold) {
                    return ScaleHeight.HIGH;
                } else if (corrected_angle > -kLargeThreshold) {
                    return ScaleHeight.NEUTRAL;
                } else {
                    return ScaleHeight.LOW;
                }
            default:
                System.out.println("Invalid previous filtered height");
                return ScaleHeight.HIGH;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Connected to CheesyVision2", isConnected());
        SmartDashboard.putNumber("Desired Height (0 cubes)", getDesiredHeight(false, 0, true));
        SmartDashboard.putNumber("Desired Height (1 cube)", getDesiredHeight(false, 1, true));
        SmartDashboard.putNumber("Desired Height (2 cubes)", getDesiredHeight(false, 2, true));
    }

    @Override
    public void stop() {
    }

    @Override
    public void registerEnabledLoops(ILooper looper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (CheesyVision2.this) {
                    mAngle = SmartDashboard.getNumber("scaleAngle", Double.NaN);
                    mTip = SmartDashboard.getNumber("scaleTip", Double.NaN);
                    mError = SmartDashboard.getBoolean("scaleError", true);
                    double heartbeat = SmartDashboard.getNumber("scaleHeartbeat", -2);
                    if (heartbeat > mLastHeartbeatValue) {
                        mLastHeartbeatValue = heartbeat;
                        mLastHeartbeatTime = timestamp;
                    }
                    ScaleHeight newHeight = getNewFilteredScaleHeight();

                    if (newHeight != mFilteredHeight) {
                        System.out.println("Detected scale height change: " + mFilteredHeight + " -> " + newHeight);
                        mFilteredHeight = newHeight;
                    }

                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };

        looper.register(loop);
    }
}