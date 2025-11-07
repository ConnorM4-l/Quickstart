package org.firstinspires.ftc.teamcode;

public class QuadraticRegression {

    //Desmos with math https://www.desmos.com/calculator/azwixru6rz
    //y = cx^2 + bx + a;   0 = c   1 = b   2 = a
    public static double[] calc(double[] xPoints, double[] yPoints) {
        assert xPoints.length == yPoints.length;

        int N = xPoints.length;
        //Finding avgs
        double Xavg = 0, X2avg = 0, Yavg = 0;

        for (int i = 0; i < N; i++) {
            Xavg += xPoints[i];
            X2avg += xPoints[i] * xPoints[i];

            Yavg += yPoints[i];
        }

        Xavg /= N;
        X2avg /= N;
        Yavg /= N;

        //Finding sums
        double Sxx = 0, Sxy = 0, Sxx2 = 0, Sx2x2 = 0, Sx2y = 0;

        for (int i = 0; i < N; i++) {
            double xDiff = xPoints[i] - Xavg;
            double x2Diff = xPoints[i] * xPoints[i] - X2avg;

            double yDiff = yPoints[i] - Yavg;

            Sxx += xDiff * xDiff;
            Sxx2 += xDiff * x2Diff;
            Sx2x2 += x2Diff * x2Diff;

            Sxy += xDiff * yDiff;
            Sx2y += x2Diff * yDiff;
        }

        double[] returnVals = new double[3];

        //Finding c
        returnVals[0] = (Sx2y * Sxx - Sxy * Sxx2)/(Sxx * Sx2x2 - Sxx2 * Sxx2);

        //Finding b
        returnVals[1] = (Sxy * Sx2x2 - Sx2y * Sxx2)/(Sxx * Sx2x2 - Sxx2 * Sxx2);

        //Finding a
        returnVals[2] = Yavg - returnVals[1] * Xavg - returnVals[0]*X2avg;


        return returnVals;
    }
}