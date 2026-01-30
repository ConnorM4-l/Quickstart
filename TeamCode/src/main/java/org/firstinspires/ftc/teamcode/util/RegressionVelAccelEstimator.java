package org.firstinspires.ftc.teamcode.util;

public class RegressionVelAccelEstimator {
    private final double sampleSize;
    private final double[] velocities;
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

        velocities = new double[sampleSize];
        times = new double[sampleSize];

        prevIndex = sampleSize-1;

        reset();
    }

    public void update(double velocity) {
        velocities[curIndex] = velocity;
        times[curIndex] = ((double) System.nanoTime()) / 1e9;

        //Calculating new vals
        double[] regressionVals = QuadraticRegression.calc(times, velocities);

        accel = 2 * regressionVals[0] * times[curIndex] + regressionVals[1];

        if (!Double.isFinite(accel)) {
            accel = 0;
        }

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
            velocities[i] = 0;
            times[i] = time + (-sampleSize + i)/50;
        }
    }

    public double getVel() {
        return velocities[curIndex];
    }

    public double getAccel() {
        return accel;
    }
}