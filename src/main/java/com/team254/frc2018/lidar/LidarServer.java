package com.team254.frc2018.lidar;

import com.team254.frc2018.Constants;
import edu.wpi.first.wpilibj.Timer;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

public class LidarServer {
    private static LidarServer mInstance = null;
    private LidarProcessor mLidarProcessor = LidarProcessor.getInstance();
    private static BufferedReader mBufferedReader;
    private boolean mRunning = false;
    private Thread mThread;
    private boolean thread_ending = false;

    public static LidarServer getInstance() {
        if (mInstance == null) {
            mInstance = new LidarServer();
        }
        return mInstance;
    }

    public LidarServer() {
    }

    public boolean isLidarConnected() {
        try {
            Runtime r = Runtime.getRuntime();
            Process p = r.exec("/bin/ls /dev/serial/by-id/");
            InputStreamReader reader = new InputStreamReader(p.getInputStream());
            BufferedReader response = new BufferedReader(reader);
            String s;
            while ((s = response.readLine()) != null) {
                if (s.equals("usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"))
                    return true;
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return false;
    }

    public boolean start() {
        synchronized (LidarServer.this) {
            if (!isLidarConnected() || mRunning) {
                return false;
            }
            mRunning = true;
        }

        System.out.println("Starting lidar");
        try {
            Process p = new ProcessBuilder().command(Constants.kChezyLidarPath).start();
            mThread = new Thread(new ReaderThread());
            mThread.start();
            InputStreamReader reader = new InputStreamReader(p.getInputStream());
            mBufferedReader = new BufferedReader(reader);
        } catch (Exception e) {
            e.printStackTrace();
        }

        return true;
    }

    public boolean stop() {
        synchronized (LidarServer.this) {
            if (!mRunning) {
                return false;
            }

            System.out.print("Stopping Lidar... ");

            mRunning = false;
            thread_ending = true;
        }

        try {
            Runtime r = Runtime.getRuntime();
            r.exec("/usr/bin/killall chezy_lidar");
        } catch (IOException e) {
            System.out.println("Error: couldn't kill process");
            e.printStackTrace();
            thread_ending = false;
            return false;
        }

        try {
            mThread.join();
        } catch (InterruptedException e) {
            System.out.println("Error: Couldn't join thread");
            e.printStackTrace();
            thread_ending = false;
            return false;
        }
        System.out.println("Stopped");
        thread_ending = false;
        return true;
    }

    private void handleLine(String line) {
        boolean isNewScan = line.substring(line.length() - 1).equals("s");
        if (isNewScan) {
            line = line.substring(0, line.length() - 1);
        }

        long curSystemTime = System.currentTimeMillis();
        double curFPGATime = Timer.getFPGATimestamp();

        String[] parts = line.split(",");
        if (parts.length == 3) {
            try {
                long ts = Long.parseLong(parts[0]);
                long ms_ago = curSystemTime - ts;
                double normalizedTs = curFPGATime - (ms_ago / 1000.0f);
                double angle = Double.parseDouble(parts[1]);
                double distance = Double.parseDouble(parts[2]);
                mLidarProcessor.addPoint(new LidarPoint(normalizedTs, angle, distance), isNewScan);
            } catch (java.lang.NumberFormatException e) {
                e.printStackTrace();
            }
        }
    }

    public boolean isRunning() {
        return mRunning;
    }

    public boolean isEnding() {
        return thread_ending;
    }

    private class ReaderThread implements Runnable {
        @Override
        public void run() {
            while (mRunning) {
                String line;
                try {
                    while (mRunning && mBufferedReader.ready() && (line = mBufferedReader.readLine()) != null) {
                        handleLine(line);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }

}
