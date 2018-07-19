package com.team254.frc2018.lidar.icp;

import java.util.Collection;

public class ReferenceModel {

    public static final double TOWER_WIDTH = 17;
    public static final double TOWER_DEPTH = 21.5;
    private static final Point TOWER00 = new Point(0, -TOWER_WIDTH / 2);
    private static final Point TOWER01 = new Point(0, +TOWER_WIDTH / 2);
    private static final Point TOWER10 = new Point(TOWER_DEPTH, -TOWER_WIDTH / 2);
    private static final Point TOWER11 = new Point(TOWER_DEPTH, +TOWER_WIDTH / 2);

    public static final ReferenceModel TOWER = new ReferenceModel(
            // new Segment(TOWER00, TOWER10), // bottom (-Y) face
            new Segment(TOWER00, TOWER01) // front face
            // new Segment(TOWER01, TOWER11)  // top (+Y) face
    );


    public final Segment[] segments;

    public ReferenceModel(Segment... ss) {
        if (ss.length == 0) throw new IllegalArgumentException("zero Segments passed to ReferenceModel");
        segments = ss;
    }

    public ReferenceModel(Collection<Segment> ss) {
        this(ss.toArray(new Segment[ss.size()]));
    }

    public Point getClosestPoint(Point p) {
        double minDist = Double.MAX_VALUE;
        Segment minSeg = null;
        for (Segment s : segments) {
            double dist = s.getDistanceSq(p);
            if (dist < minDist) {
                minDist = dist;
                minSeg = s;
            }
        }
        return minSeg.getClosestPoint(p);
    }

    public Point getMidpoint() {
        if (segments.length > 1)
            throw new RuntimeException("getMidpoint() called on multi-segment ReferenceModel");
        return segments[0].getMidpoint();
    }

    public String toString() {
        String str = "[";
        for (Segment s : segments) {
            str += "\n  " + s;
        }
        return str + "\n]";
    }

}