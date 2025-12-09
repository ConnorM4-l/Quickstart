package org.firstinspires.ftc.teamcode.util;

public class RegressionVelAccelEstimator {
    private final double sampleSize;
    private final double[] positions;
    private final double[] times;

    private double prevVel = 0;
    private int curIndex = 0;
    private int prevIndex;

    private double vel = 0;
    private double accel = 0;

    //Larger = smoother, more averaging; smaller = more reactive
    public RegressionVelAccelEstimator() {
        this(5);
    }

    public RegressionVelAccelEstimator(int sampleSize) {
        this.sampleSize = sampleSize;

        positions = new double[sampleSize];
        times = new double[sampleSize];

        prevIndex = sampleSize-1;

        reset();
    }

    public void update(double velocity) {
//        positions[curIndex] = curPos;
//        times[curIndex] = ((double) System.nanoTime()) / 1e9;

        prevVel = vel;

        //Calculating new vals
        double[] regressionVals = QuadraticRegression.calc(times, positions);

//        vel = 2 * regressionVals[0] * times[curIndex] + regressionVals[1];
        vel = velocity;

        if (!Double.isFinite(vel)) {
            vel = 0;
        }

        accel = (vel-prevVel)/(times[curIndex]-times[prevIndex]);

        //Updating index
        prevIndex = curIndex;
        curIndex++;
        if (curIndex > sampleSize-1) {
            curIndex = 0;
        }
    }

    public void reset() {
        double time = ((double) System.currentTimeMillis()) / 1000;
        for (int i = 0; i < sampleSize; i++) {
            positions[i] = 0;
            times[i] = time + (-sampleSize + i)/50;
        }
    }

    public double getVel() {
        return vel;
    }

    public double getAccel() {
        return accel;
    }
}