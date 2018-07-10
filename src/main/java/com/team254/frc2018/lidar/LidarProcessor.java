package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import com.team254.frc2018.loops.Loop;
import edu.wpi.first.wpilibj.Timer;

import java.util.LinkedList;

/**
 * Stores a set amount of lidar scans. All interfacing with the lidar should be
 * done through this class.
 */
public class LidarProcessor implements Loop {
    private static LidarProcessor mInstance = null;
    private LidarServer mLidarServer = LidarServer.getInstance();
    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prev_timestamp = Double.MAX_VALUE;

    public static LidarProcessor getInstance() {
        if (mInstance == null) {
            mInstance = new LidarProcessor();
        }
        return mInstance;
    }

    public LidarProcessor() {
        mScans.add(new LidarScan());
    }

    int count = 0;

    public void addPoint(LidarPoint point, boolean newScan) {
        if (newScan) { // crosses the 360-0 threshold. start a new scan
            prev_timestamp = Timer.getFPGATimestamp();
            count++;
            if (count > 10) {
                count = 0;
                // SmartDashboard.putString("lidarScan", mScans.getLast().toJsonString());
                // //output to lidar visualizer
            }

            mScans.add(new LidarScan());
            if (mScans.size() > Constants.kChezyLidarNumScansToStore) {
                mScans.removeFirst();
            }
        }

        if (point.toCartesian() != null) {
            mScans.getLast().addPoint(point);
        }
    }

    public LinkedList<LidarScan> getAllScans() {
        return mScans;
    }

    public LidarScan getCurrentScan() {
        return mScans.getLast();
    }

    public LidarScan getLatestCompleteScan() {
        return mScans.get(mScans.size() > 1 ? mScans.size() - 2 : 0);
    }

    @Override
    public void onStart(double timestamp) {
        prev_timestamp = Double.MAX_VALUE;
    }

    @Override
    public void onLoop(double timestamp) {
        if (Timer.getFPGATimestamp() - prev_timestamp > Constants.kChezyLidarRestartTime) {
            if (mLidarServer.isRunning()) {
                System.out.println("Lidar timed out. Restarting");
                mLidarServer.stop();
            } else {
                if (!mLidarServer.isEnding()) {
                    if (mLidarServer.start()) {
                        prev_timestamp = Timer.getFPGATimestamp();
                    }
                }
            }
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
}