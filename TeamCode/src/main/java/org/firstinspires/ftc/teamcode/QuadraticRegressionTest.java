package org.firstinspires.ftc.teamcode;

public class QuadraticRegressionTest {
    public static void main(String[] args) {
        double[] x = {1, 2, 3, 4};
        double[] y = {2, 4, 6, 8};

        double[] regressionVals = QuadraticRegression.calc(x, y);

        double vel = 2 * regressionVals[0] * (4) + regressionVals[1];

        System.out.println(vel);
    }
}