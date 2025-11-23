package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drivetrain {
    private DcMotor leftOdo;
    private DcMotor rightOdo;
    private DcMotor strafeOdo;
    private IMU imu;
    private DcMotor bl = null;
    private DcMotor fl = null;
    private DcMotor br = null;
    private DcMotor fr = null;


    private Coordinate location;
    private Double angle;
    private PIDController angularPid = new PIDController(0.5, 0.01, 0.1, 10, 1);
    private ElapsedTime driveClock = new ElapsedTime();
    private double lastTime = 0;
    private double lastTicksX = 0;
    private double lastTicksY = 0;

    public Drivetrain(HardwareMap hardwareMap) {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");
        strafeOdo = hardwareMap.get(DcMotor.class, "strafeOdo");
        imu = hardwareMap.get(IMU.class, "imu");
        location = new Coordinate();
        angle = 0.0;
    }

    public Drivetrain(HardwareMap hardwareMap, Coordinate loc) {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");
        strafeOdo = hardwareMap.get(DcMotor.class, "strafeOdo");
        imu = hardwareMap.get(IMU.class, "imu");
        location = loc;
        angle = 0.0;
    }
    public Drivetrain(HardwareMap hardwareMap, Coordinate loc, Double ang) {
        bl = hardwareMap.get(DcMotor.class, "bl");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        fr = hardwareMap.get(DcMotor.class, "fr");
        leftOdo = hardwareMap.get(DcMotor.class, "leftOdo");
        rightOdo = hardwareMap.get(DcMotor.class, "rightOdo");
        strafeOdo = hardwareMap.get(DcMotor.class, "strafeOdo");
        imu = hardwareMap.get(IMU.class, "imu");
        location = loc;
        angle = ang;
    }
    public void updateLocation(Double strafeDist)
    {
        location.incermentX( strafeDist * Math.cos(angle) );
        location.incermentY( strafeDist * Math.sin(angle) );
    }

    public void update(double axial, double lateral, double yaw) {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // Note: pushing stick forward gives negative value


        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);

        // Show the elapsed game time and wheel power.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    public Boolean isFacingTower(Coordinate towerPoint)
    {
        return (location.getAngle(towerPoint,angle) < 0.1);
    }
    public void updateFacingPoint(double axial, double lateral, Coordinate point,double curAngle, double angVel) {
        double max;
        double currentTime = driveClock.seconds();
        angle = curAngle;
        double localDisplacementY = (rightOdo.getCurrentPosition() - lastTicksY);
        lastTicksY = currentTime;
        double localDisplacementX = (strafeOdo.getCurrentPosition() - lastTicksX);
        lastTicksX = currentTime;

        location.incermentX(Math.cos(angle)*localDisplacementY + localDisplacementX*Math.sin(Math.PI/2 + angle));
        location.incermentY(Math.sin(angle)*localDisplacementY + localDisplacementX*Math.cos(Math.PI/2 + angle));

        double hx = (location.getX() - point.getX());
        double hy = (location.getY() - point.getY());
        double dt = (currentTime - lastTime);
        lastTime = currentTime;
        double dxdt = hx/dt;
        double dydt = hy/dt;

        double yaw = angularPid.calculate((1 / (1 + Math.pow(hy / hx,2)) * ((dxdt * hx - dydt * hy) / (hx*hx))), angVel);

        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        fl.setPower(frontLeftPower);
        fr.setPower(frontRightPower);
        bl.setPower(backLeftPower);
        br.setPower(backRightPower);
    }
}
