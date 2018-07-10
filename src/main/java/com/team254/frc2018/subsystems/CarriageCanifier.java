package com.team254.frc2018.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.team254.frc2018.Constants;

public class CarriageCanifier extends Subsystem {
    private static CarriageCanifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;
    private PeriodicOutputs mPeriodicOutputs;
    private boolean mOutputsChanged = true;

    private CarriageCanifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicInputs = new PeriodicInputs();
        mPeriodicOutputs = new PeriodicOutputs();

        // Force a first update.
        mOutputsChanged = true;
    }

    public synchronized static CarriageCanifier getInstance() {
        if (mInstance == null) {
            mInstance = new CarriageCanifier();
        }
        return mInstance;
    }

    public int getWristTicks() {
        return mCanifier.getQuadraturePosition();
    }

    public synchronized boolean getLeftBannerSensor() {
        return mPeriodicInputs.left_sensor_state_;
    }

    public synchronized boolean getRightBannerSensor() {
        return mPeriodicInputs.right_sensor_state_;
    }

    public synchronized boolean getLimR() {
        return mPeriodicInputs.limr_;
    }

    public synchronized void resetWristEncoder() {
        mCanifier.setQuadraturePosition(0, 0);
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    public synchronized void setLEDColor(double red, double green, double blue) {
        if (red != mPeriodicOutputs.r_ || green != mPeriodicOutputs.g_ || blue != mPeriodicOutputs.b_) {
            mPeriodicOutputs.r_ = red;
            mPeriodicOutputs.g_ = green;
            mPeriodicOutputs.b_ = blue;
            mOutputsChanged = true;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);
        mPeriodicInputs.left_sensor_state_ = pins.SCL;
        mPeriodicInputs.right_sensor_state_ = pins.SDA;
        mPeriodicInputs.limr_ = !pins.LIMR;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        // A: Blue
        // B: Green
        // C: Red
        if (mOutputsChanged) {
            mCanifier.setLEDOutput(mPeriodicOutputs.b_, CANifier.LEDChannel.LEDChannelA);
            mCanifier.setLEDOutput(mPeriodicOutputs.g_, CANifier.LEDChannel.LEDChannelB);
            mCanifier.setLEDOutput(mPeriodicOutputs.r_, CANifier.LEDChannel.LEDChannelC);
            mOutputsChanged = false;
        }
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {
        mPeriodicOutputs = new PeriodicOutputs();
        mOutputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public void zeroSensors() {
        mPeriodicInputs = new PeriodicInputs();
    }

    private static class PeriodicInputs {
        public boolean left_sensor_state_;
        public boolean right_sensor_state_;
        public boolean limr_;
    }

    private static class PeriodicOutputs {
        public double r_;
        public double g_;
        public double b_;
    }
}
