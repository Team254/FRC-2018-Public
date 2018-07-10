package com.team254.frc2018;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents the state of the robot and field at the beginning
 * of the autonomous period: the starting position of the robot,
 * and the state of each of the switch/scale plates.
 */
public class AutoFieldState {
    private static AutoFieldState mInstance = null;

    public enum Side {LEFT, RIGHT}

    private Side ourSwitchSide, scaleSide, opponentSwitchSide;
    private Side overrideOurSwitchSide, overrideScaleSide, overrideOpponentSwitchSide;
    private boolean overrideGameData = false;

    private AutoFieldState() {
    }

    public synchronized static AutoFieldState getInstance() {
        if (mInstance == null) {
            mInstance = new AutoFieldState();
        }
        return mInstance;
    }

    /**
     * Sets the switch/scale sides based on the given GameSpecificMessage.
     * If the message is invalid or null, returns false and leaves this
     * object unchanged; otherwise, on success, returns true.
     */
    public synchronized boolean setSides(String gameData) {
        if (gameData == null) {
            return false;
        }
        gameData = gameData.trim();
        if (gameData.length() != 3) {
            return false;
        }
        Side s0 = getCharSide(gameData.charAt(0));
        Side s1 = getCharSide(gameData.charAt(1));
        Side s2 = getCharSide(gameData.charAt(2));
        if (s0 == null || s1 == null || s2 == null) {
            return false;
        }
        ourSwitchSide = s0;
        scaleSide = s1;
        opponentSwitchSide = s2;
        return true;
    }

    public synchronized boolean isValid() {
        if (overrideGameData) {
            return overrideScaleSide != null && overrideOurSwitchSide != null;
        } else {
            return scaleSide != null && ourSwitchSide != null;
        }
    }

    public synchronized boolean overrideSides(String gameData) {
        if (gameData == null) {
            return false;
        }
        gameData = gameData.trim();
        if (gameData.length() != 3) {
            return false;
        }
        Side s0 = getCharSide(gameData.charAt(0));
        Side s1 = getCharSide(gameData.charAt(1));
        Side s2 = getCharSide(gameData.charAt(2));
        if (s0 == null || s1 == null || s2 == null) {
            return false;
        }
        overrideOurSwitchSide = s0;
        overrideScaleSide = s1;
        overrideOpponentSwitchSide = s2;
        overrideGameData = true;
        return true;
    }

    public synchronized void disableOverride() {
        overrideGameData = false;
    }

    public synchronized boolean isOverridingGameData() {
        return overrideGameData;
    }

    /**
     * Helper method to convert 'L' or 'R' to their respective Side.
     */
    private Side getCharSide(char c) {
        return c == 'L' ? Side.LEFT : c == 'R' ? Side.RIGHT : null;
    }

    /**
     * Returns which Side of our switch is our alliance's.
     */
    public synchronized Side getOurSwitchSide() {
        if (overrideGameData) {
            return overrideOurSwitchSide;
        } else {
            return ourSwitchSide;
        }
    }

    /**
     * Returns which Side of the scale is our alliance's.
     */
    public synchronized Side getScaleSide() {
        if (overrideGameData) {
            return overrideScaleSide;
        } else {
            return scaleSide;
        }
    }

    /**
     * Returns which Side of our opponent's switch is our alliance's.
     */
    public synchronized Side getOpponentSwitchSide() {
        if (overrideGameData) {
            return overrideOpponentSwitchSide;
        } else {
            return opponentSwitchSide;
        }
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("FieldState OurSwitch", getOurSwitchSide() == null ? "NULL" : getOurSwitchSide().toString());
        SmartDashboard.putString("FieldState Scale", getScaleSide() == null ? "NULL" : getScaleSide().toString());
        SmartDashboard.putString("FieldState TheirSwitch", getOpponentSwitchSide() == null ? "NULL" : getOpponentSwitchSide().toString());
    }

    @Override
    public String toString() {
        return "AutoFieldState{" +
                "ourSwitchSide=" + getOurSwitchSide() +
                ", scaleSide=" + getScaleSide() +
                ", opponentSwitchSide=" + getOpponentSwitchSide() +
                '}';
    }
}