package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import com.team254.frc2018.RobotState;
import com.team254.frc2018.lidar.icp.ICP;
import com.team254.frc2018.lidar.icp.Point;
import com.team254.frc2018.lidar.icp.ReferenceModel;
import com.team254.frc2018.lidar.icp.Transform;
import com.team254.frc2018.loops.Loop;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.zip.GZIPOutputStream;

/**
 * Receives LIDAR points from the {@link LidarServer}, stores a set number
 * of scans/revolutions, and provides methods for processing the data.
 * <p>
 * All interfacing with the LIDAR should be done through this class.
 *
 * @see Constants.kChezyLidarNumScansToStore
 * @see doICP()
 * @see getTowerPosition()
 */
public class LidarProcessor implements Loop {
    private static LidarProcessor mInstance = null;

    public static LidarProcessor getInstance() {
        if (mInstance == null) {
            mInstance = new LidarProcessor();
        }
        return mInstance;
    }

    private RobotState mRobotState = RobotState.getInstance();

    private LinkedList<LidarScan> mScans = new LinkedList<>();
    private double prev_timestamp;

    private ICP icp = new ICP(ReferenceModel.TOWER, 100);

    private DataOutputStream dataLogFile;

    private final ReadWriteLock lock = new ReentrantReadWriteLock();

    private static FileOutputStream newLogFile() throws IOException {
        // delete old files if we're over the limit
        File logDir = new File(Constants.kLidarLogDir);
        File[] logFiles = logDir.listFiles();
        if (logFiles == null) throw new IOException("List files in " + Constants.kLidarLogDir);
        Arrays.sort(logFiles, (f1, f2) -> {
            return Long.compare(f1.lastModified(), f2.lastModified());
        });
        for (int i = 0; i < logFiles.length - Constants.kNumLidarLogsToKeep + 1; i++) {
            logFiles[i].delete();
        }

        // create the new file and return
        String dateStr = new SimpleDateFormat("MM-dd-HH_mm_ss").format(new Date());
        File newFile = new File(logDir, "lidarLog-" + dateStr + ".dat");
        newFile.createNewFile();
        return new FileOutputStream(newFile, false);
    }

    private LidarProcessor() {
        mScans.add(new LidarScan());
        try {
            dataLogFile = new DataOutputStream(new GZIPOutputStream(newLogFile()));
        } catch (IOException e) {
            System.err.println("Failed to open lidar log file:");
            e.printStackTrace();
        }
    }

    private void logPoint(double angle, double dist, double x, double y) {
        try {
            dataLogFile.writeInt((int) (angle * 100));
            dataLogFile.writeInt((int) (dist * 256));
            dataLogFile.writeInt((int) (x * 256));
            dataLogFile.writeInt((int) (y * 256));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addPoint(LidarPoint point, boolean newScan) {
        SmartDashboard.putNumber("LIDAR last_angle", point.angle);

        Translation2d cartesian = point.toCartesian();
        logPoint(point.angle, point.distance, cartesian.x(), cartesian.y());

        lock.writeLock().lock();
        try {
            if (newScan) { // crosses the 360-0 threshold. start a new scan
                prev_timestamp = Timer.getFPGATimestamp();

                // long start = System.nanoTime();
                // Translation2d towerPos = getTowerPosition();
                // long end = System.nanoTime();
                // SmartDashboard.putNumber("towerPos_ms", (end-start)/1000000);
                // SmartDashboard.putNumber("towerPosX", towerPos.x());
                // SmartDashboard.putNumber("towerPosY", towerPos.y());

                mScans.add(new LidarScan());
                if (mScans.size() > Constants.kChezyLidarNumScansToStore) {
                    mScans.removeFirst();
                }
            }

            if (!excludePoint(cartesian.x(), cartesian.y())) {
                getCurrentScan().addPoint(new Point(cartesian), point.timestamp);
            }
        } finally {
            lock.writeLock().unlock();
        }
    }

    private static final double FIELD_WIDTH = 27 * 12, FIELD_HEIGHT = 54 * 12;
    private static final double RECT_RX = FIELD_WIDTH / 5, RECT_RY = FIELD_HEIGHT / 2;
    private static final double FIELD_CX = FIELD_WIDTH / 2, FIELD_CY = FIELD_HEIGHT / 2;
    private static final double RECT_X_MIN = FIELD_CX - RECT_RX, RECT_X_MAX = FIELD_CX + RECT_RX,
            RECT_Y_MIN = FIELD_CY - RECT_RY, RECT_Y_MAX = FIELD_CY + RECT_RY;

    private static boolean excludePoint(double x, double y) {
        return x < RECT_X_MIN || x > RECT_X_MAX ||
                y < RECT_Y_MIN || y > RECT_Y_MAX;
    }

    private LidarScan getCurrentScan() {
        return mScans.getLast();
    }

    private ArrayList<Point> getAllPoints() {
        ArrayList<Point> list = new ArrayList<>();
        for (LidarScan scan : mScans) {
            list.addAll(scan.getPoints());
        }
        return list;
    }

    private Point getAveragePoint() {
        double sumX = 0, sumY = 0;
        int n = 0;
        for (Point p : getAllPoints()) {
            sumX += p.x;
            sumY += p.y;
            n++;
        }
        return new Point(sumX / n, sumY / n);
    }

    private static final double BUCKET_SIZE = 3.0; // inches

    /**
     * Cantor pairing function (to bucket & hash two doubles)
     */
    private int getBucket(double x, double y) {
        int ix = (int) (x / BUCKET_SIZE);
        int iy = (int) (y / BUCKET_SIZE);
        int a = ix >= 0 ? 2 * ix : -2 * ix - 1;
        int b = iy >= 0 ? 2 * iy : -2 * iy - 1;
        int sum = a + b;
        return sum * (sum + 1) / 2 + a;
    }

    /**
     * Returns a list of points that have been thinned roughly uniformly.
     */
    private ArrayList<Point> getCulledPoints() {
        ArrayList<Point> list = new ArrayList<>();
        HashSet<Integer> buckets = new HashSet<>();
        for (Point p : getAllPoints()) {
            if (buckets.add(getBucket(p.x, p.y)))
                list.add(p);
        }
        return list;
    }

    public Pose2d doICP() {
        lock.readLock().lock();
        try {
            Pose2d guess = mRobotState.getFieldToLidar(getCurrentScan().getTimestamp());
            return icp.doICP(getCulledPoints(), new Transform(guess).inverse()).inverse().toPose2d();
        } finally {
            lock.readLock().unlock();
        }
    }

    public Translation2d getTowerPosition() {
        lock.readLock().lock();
        try {
            Point avg = getAveragePoint();
            Transform trans = icp.doICP(getCulledPoints(), new Transform(0, avg.x, avg.y));
            return trans.apply(icp.reference).getMidpoint().toTranslation2d();
        } finally {
            lock.readLock().unlock();
        }
    }

    public void setPrevTimestamp(double time) {
        lock.writeLock().lock();
        try {
            prev_timestamp = time;
        } finally {
            lock.writeLock().unlock();
        }
    }

    public double getPrevTimestamp() {
        lock.readLock().lock();
        try {
            return prev_timestamp;
        } finally {
            lock.readLock().unlock();
        }
    }

    @Override
    public void onStart(double timestamp) {
        setPrevTimestamp(Double.NEGATIVE_INFINITY);
    }

    @Override
    public void onLoop(double timestamp) {
        LidarServer lidarServer = LidarServer.getInstance();
        if (timestamp - getPrevTimestamp() > Constants.kChezyLidarRestartTime) {
            if (lidarServer.isRunning()) {
                System.err.println("Lidar timed out. Restarting");
                lidarServer.stop();
            } else if (!lidarServer.isEnding() && lidarServer.start()) {
                setPrevTimestamp(timestamp);
            }
        }
    }

    @Override
    public void onStop(double timestamp) {
    }
}