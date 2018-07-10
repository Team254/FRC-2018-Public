package com.team254.frc2018.auto.creators;

import com.team254.frc2018.AutoFieldState;
import com.team254.frc2018.auto.AutoModeBase;

public interface AutoModeCreator {
    AutoModeBase getStateDependentAutoMode(AutoFieldState fieldState);
}
