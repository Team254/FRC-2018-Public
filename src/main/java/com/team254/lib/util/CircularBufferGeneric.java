package com.team254.lib.util;

import java.util.LinkedList;

/**
 * Implements a simple circular buffer.
 * Can be used for any class.
 */
public class CircularBufferGeneric<E> {
    /**
     *
     */
    int mWindowSize;
    LinkedList<E> mSamples;
    double mSum;

    public CircularBufferGeneric(int window_size) {
        mWindowSize = window_size;
        mSamples = new LinkedList<E>();
        mSum = 0.0;
    }


    public void clear() {
        mSamples.clear();
        mSum = 0.0;
    }

    public void addValue(E val) {
        mSamples.addLast(val);
        if (mSamples.size() > mWindowSize) {
            mSamples.removeFirst();
        }
    }

    public int getNumValues() {
        return mSamples.size();
    }

    public boolean isFull() {
        return mWindowSize == mSamples.size();
    }

    public LinkedList<E> getLinkedList() {
        /*
         * NOTE: To get an Array of the specific class type which the instance is using,
         * you have to use this specific code:
         * specificCircularBufferGeneric.getLinkedList().toArray(new ClassThatIWant[specificCircularBufferGeneric
         * .getLinkedList().size()]);
         * The reason is that for some reason an array of a generic class(i.e. E[]) cannot be created because
         * of some archaic data flow ambiguities
         */

        return mSamples;
    }
}
