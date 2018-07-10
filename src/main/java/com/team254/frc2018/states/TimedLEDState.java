package com.team254.frc2018.states;

public interface TimedLEDState {
    void getCurrentLEDState(LEDState desiredState, double timestamp);

    class BlinkingLEDState implements TimedLEDState {
        public static BlinkingLEDState kHasCube = new BlinkingLEDState(
                LEDState.kIntakeHasCube,
                LEDState.kOff,
                0.5);

        LEDState mStateOne = new LEDState(0.0, 0.0, 0.0);
        LEDState mStateTwo = new LEDState(0.0, 0.0, 0.0);
        double mDuration;

        public BlinkingLEDState(LEDState stateOne, LEDState stateTwo, double duration) {
            mStateOne.copyFrom(stateOne);
            mStateTwo.copyFrom(stateTwo);
            mDuration = duration;
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            if ((int) (timestamp / mDuration) % 2 == 0) {
                desiredState.copyFrom(mStateOne);
            } else {
                desiredState.copyFrom(mStateTwo);
            }
        }
    }

    class StaticLEDState implements TimedLEDState {
        public static StaticLEDState kStaticOff = new StaticLEDState(LEDState.kOff);
        public static StaticLEDState kIntaking = new StaticLEDState(LEDState.kIntakeIntaking);
        public static StaticLEDState kHasCube = new StaticLEDState(LEDState.kIntakeHasCube);

        LEDState mStaticState = new LEDState(0.0, 0.0, 0.0);

        public StaticLEDState(LEDState staticState) {
            mStaticState.copyFrom(staticState);
        }

        @Override
        public void getCurrentLEDState(LEDState desiredState, double timestamp) {
            desiredState.copyFrom(mStaticState);
        }

    }
}
