package com.team254.frc2018.auto.actions;

import com.team254.frc2018.subsystems.Superstructure;

public class EngageKickstand extends RunOnceAction {
    private boolean mEngage;

    public EngageKickstand(boolean engage) {
        mEngage = engage;
    }

    @Override
    public void runOnce() {
        Superstructure.getInstance().setKickstand(mEngage);
    }
}
