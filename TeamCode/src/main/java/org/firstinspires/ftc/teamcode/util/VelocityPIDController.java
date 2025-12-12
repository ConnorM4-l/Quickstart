package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class VelocityPIDController {
    public static double kP = 0.05, kD = 0, kI = 0, kFs = 0.13, kFv = 0.00055;
    //Fs and Fv, and then P then D and then I if necessary

    ElapsedTime timer = new ElapsedTime();
    RegressionVelAccelEstimator estimator = new RegressionVelAccelEstimator(15);

    double I = 0;
    double err = 0;
    double currentPosition = 0;
    double vel = 0;
    double elapsedTime;
    double acc;

    public double update(double targetVelocity, double velocity) {
        estimator.update(velocity);

        elapsedTime = Math.min(timer.seconds(), 0.1);
        timer.reset();

        vel = estimator.getVel();
        acc = estimator.getAccel();

        err = targetVelocity - vel;

        I += err * elapsedTime * kI;
        I = Math.max(I, -1);
        I = Math.min(I, 1);

        return kP * err + I - kD * acc + kFs * Math.signum(targetVelocity) + kFv * targetVelocity;
    }


    public double getErr() {
        return err;
    }

    public double getVelocity() {
        return vel;
    }

    public double getAcceleration() {
        return acc;
    }
}
