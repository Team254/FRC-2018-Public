package com.team254.frc2018.auto.creators;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.modes.SimpleSwitchMode;

public class SimpleSwitchModeCreator implements AutoModeCreator {

    // Pre build trajectories to go left and right
    private SimpleSwitchMode mGoLeftMode = new SimpleSwitchMode(true);
    private SimpleSwitchMode mGoRightMode = new SimpleSwitchMode(false);

    @Override
    public AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState) {
        if (fieldState.getOurSwitchSide() == AutoFieldState.Side.LEFT) {
            return mGoLeftMode;
        } else {
            return mGoRightMode;
        }
    }
}
