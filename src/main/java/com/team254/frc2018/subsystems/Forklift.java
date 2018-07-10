package com.team254.frc2018.subsystems;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Solenoid;

public class Forklift extends Subsystem {
    private static final boolean kDeploySolenoidState = true;

    private static Forklift mInstance = null;

    private Solenoid mDeploySolenoid;
    private boolean mDeployed;

    private Forklift() {
        mDeploySolenoid = Constants.makeSolenoidForId(Constants.kForkliftDeploySolenoid);

        // Start the forklift in the retracted position.  Set true to force a state change.
        mDeployed = true;
        retract();
    }

    public synchronized static Forklift getInstance() {
        if (mInstance == null) {
            mInstance = new Forklift();
        }
        return mInstance;
    }

    public synchronized void deploy() {
        // Try to avoid hitting CAN/JNI wrapper.
        if (!mDeployed) {
            mDeploySolenoid.set(kDeploySolenoidState);
            mDeployed = true;
        }
    }

    public synchronized void retract() {
        // Try to avoid hitting CAN/JNI wrapper.
        if (mDeployed) {
            mDeploySolenoid.set(!kDeploySolenoidState);
            mDeployed = false;
        }
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }
}