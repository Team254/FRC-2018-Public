package com.team254.frc2018;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;


public class AutoFieldStateTest {

    @Test
    public void test() {
        AutoFieldState state = AutoFieldState.getInstance();

        String[] badStates = new String[]{null, "dakjfhaksdjh", "", "foo", "LRQ"};
        for (String str : badStates) {
            assertFalse(state.setSides(str));
            assertNull(state.getOurSwitchSide());
            assertNull(state.getScaleSide());
            assertNull(state.getOpponentSwitchSide());
            assertFalse(state.isValid());
        }

        String[] goodStates = new String[]{"RLR", "    RLR", "RLR    ", "    RLR    "};
        for (String str : goodStates) {
            assertTrue(state.setSides(str));
            assertEquals(AutoFieldState.Side.RIGHT, state.getOurSwitchSide());
            assertEquals(AutoFieldState.Side.LEFT, state.getScaleSide());
            assertEquals(AutoFieldState.Side.RIGHT, state.getOpponentSwitchSide());
            assertTrue(state.isValid());
        }

        assertFalse(state.overrideSides("bad state"));
        assertFalse(state.isOverridingGameData());
        assertTrue(state.isValid());
        assertTrue(state.overrideSides("LRL"));
        assertTrue(state.isOverridingGameData());
        assertEquals(AutoFieldState.Side.LEFT, state.getOurSwitchSide());
        assertEquals(AutoFieldState.Side.RIGHT, state.getScaleSide());
        assertEquals(AutoFieldState.Side.LEFT, state.getOpponentSwitchSide());

        state.disableOverride();
        assertEquals(AutoFieldState.Side.RIGHT, state.getOurSwitchSide());
        assertEquals(AutoFieldState.Side.LEFT, state.getScaleSide());
        assertEquals(AutoFieldState.Side.RIGHT, state.getOpponentSwitchSide());
    }
}
