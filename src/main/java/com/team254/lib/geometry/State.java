package com.team254.lib.geometry;

import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
