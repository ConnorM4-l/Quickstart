package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RegressionVelAccelEstimator;

@Configurable
public class VelocityPIDController {
    public static double kP = 0.0007, kD = 0, kI = 0, kFs = 0.05, kFv = 0.00016;
    //Fs and Fv, and then P then D and then I if necessary

    ElapsedTime timer = new ElapsedTime();
    RegressionVelAccelEstimator estimator = new RegressionVelAccelEstimator(15);

    double I = 0;
    double err = 0;
    double currentPosition = 0;
    double vel = 0;
    double elapsedTime;
    double acc;

    public double update(double targetVelocity, double deltaPosition) {
        currentPosition += deltaPosition;
        estimator.update(currentPosition);

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
