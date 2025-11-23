package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP;
    private double kI;
    private double kD;
    private double integralLimit;
    private double maxOutput;
    private double integralSum = 0.0;
    private double lastError = 0.0;
    private ElapsedTime clock = new ElapsedTime();
    private double lastTime = 0;
    public PIDController(double kP, double kI, double kD, double integralLimit, double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralLimit = Math.abs(integralLimit);
        this.maxOutput = Math.abs(maxOutput);
    }

    public double calculate(double target, double current) {

        double currentTime = clock.seconds();

        double dt = (currentTime - lastTime);
        lastTime = currentTime;
        if (dt == 0) {
            return lastError * kP;
        }

        // 2. Calculate Error
        double error = target - current;

        // --- P Term (Proportional) ---
        // P: Output proportional to the current error. Large error -> large output.
        double proportional = kP * error;

        // --- I Term (Integral) ---
        // I: Output proportional to the accumulation of error over time.
        // This eliminates steady-state error.
        this.integralSum += error * dt;

        // Anti-windup: Clamp the integral term to prevent excessive overshoot
        if (this.integralSum > integralLimit) {
            this.integralSum = integralLimit;
        } else if (this.integralSum < -integralLimit) {
            this.integralSum = -integralLimit;
        }

        double integral = kI * this.integralSum;

        // --- D Term (Derivative) ---
        // D: Output proportional to the rate of change of the error.
        // This dampens oscillation and increases stability.
        double errorDerivative = (error - lastError) / dt;
        double derivative = kD * errorDerivative;

        // 3. Combine and Update State
        double output = proportional + integral + derivative;
        this.lastError = error; // Store current error for the next derivative calculation

        // 4. Clamp Output to Max Power
        if (output > maxOutput) {
            return maxOutput;
        } else if (output < -maxOutput) {
            return -maxOutput;
        }

        return output;
    }
    public void reset() {
        this.integralSum = 0.0;
        this.lastError = 0.0;
        clock.reset();
    }

}
