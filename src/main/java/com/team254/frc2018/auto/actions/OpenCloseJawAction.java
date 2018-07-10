package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Intake;

public class OpenCloseJawAction implements Action {
    private boolean mOpen;

    public OpenCloseJawAction(boolean open) {
        mOpen = open;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        if (mOpen) {
            Intake.getInstance().tryOpenJaw();
        } else {
            Intake.getInstance().closeJaw();
        }
    }
}
