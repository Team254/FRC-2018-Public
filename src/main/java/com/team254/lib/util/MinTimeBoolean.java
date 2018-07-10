package com.team254.lib.util;

/**
 * This boolean enforces a minimum time for the value to be true.  It captures a rising edge and enforces
 * based on timestamp.
 */
public class MinTimeBoolean {
    private LatchedBoolean mLatchedBoolean;
    private double mMinTime;
    private double mRisingEdgeTime;

    public MinTimeBoolean(double minTime) {
        mLatchedBoolean = new LatchedBoolean();
        mMinTime = minTime;
        mRisingEdgeTime = Double.NaN;
    }

    public boolean update(boolean value, double timestamp) {
        if (mLatchedBoolean.update(value)) {
            mRisingEdgeTime = timestamp;
        }

        if (!value && !Double.isNaN(mRisingEdgeTime)
                && (timestamp - mRisingEdgeTime < mMinTime)) {
            return true;
        }
        return value;
    }
}
