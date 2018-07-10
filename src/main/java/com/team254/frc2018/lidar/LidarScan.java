package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import com.team254.lib.geometry.Translation2d;

import java.util.ArrayList;

/**
 * Holds a single 360 degree scan from the lidar
 */
public class LidarScan {
    private ArrayList<Translation2d> points = new ArrayList<>(Constants.kChezyLidarScanSize);
    private double timestamp;

    public String toJsonString() {
        String json = "{\"timestamp\": " + timestamp + ", \"scan\": [";
        for (Translation2d point : points) {
            json += "{\"x\":" + point.x() + ", \"y\":" + point.y() + "},";
        }
        json = json.substring(0, json.length() - 1);
        json += "]}";
        return json;
    }

    public String toString() {
        String s = "";
        for (Translation2d point : points) {
            s += "x: " + point.x() + ", y: " + point.y() + "\n";
        }
        return s;
    }

    public ArrayList<Translation2d> getPoints() {
        return points;
    }

    public double getTimestamp() {
        return timestamp;
    }

    public void addPoint(LidarPoint point) {
        if (timestamp == 0) {
            timestamp = point.timestamp;
        }
        points.add(point.toCartesian());
    }
}