package com.team254.frc2018.auto.actions;

import com.team254.frc2018.statemachines.IntakeStateMachine;
import com.team254.frc2018.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class PlaceCube implements Action {
    private static final Intake mIntake = Intake.getInstance();
    private static final double kPlaceTime = 0.75;

    private double mStartTime;

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
        mIntake.setState(IntakeStateMachine.WantedAction.WANT_MANUAL);
        mIntake.setPower(0.0);
        mIntake.tryOpenJaw();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - mStartTime > kPlaceTime;
    }

    @Override
    public void done() {
        mIntake.clampJaw();
    }
}
