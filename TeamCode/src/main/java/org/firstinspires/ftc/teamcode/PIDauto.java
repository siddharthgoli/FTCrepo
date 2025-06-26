package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Autonomous(name = "PID Auto")
public class PIDauto extends LinearOpMode {

    // declare motors
    DcMotor rightMotor;
    DcMotor leftMotor;

    // declare imu
    IMU imu;

    // initialize pid variables
    static double driveKp = 0.0; // tune this
    static double driveKi = 0.0; // tune this
    static double driveKd = 0.0; // tune this

    static double turnKp = 0.0; // tune this
    static double turnKi = 0.0; // tune this
    static double turnKd = 0.0; // tune this

    final double COUNTS_PER_INCH = 0.0; // tune this
    final double COUNTS_PER_DEGREE = 0.0; // tune this

    @Override
    public void runOpMode() throws InterruptedException {

        // motor setup
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        leftMotor = hardwareMap.get(DcMotor.class, "left");

        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // imu setup
        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        waitForStart();

        if (opModeIsActive()) {
            pidDrive(12);
            pidTurn(90);
            // gyroTurn(90);
            pidDrive(6);
            pidDrive(-6);
            pidTurn(-90);
            // gyroTurn(-90);
            pidDrive(-12);

        }

    }

    public void pidDrive(double inches) {
        // find target tick value
        double targetCounts = inches * COUNTS_PER_INCH;

        // reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize variables
        double error = 0;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;
        double power = 0;

        while (opModeIsActive()) {
            // get current position
            double currentPosition = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2.0;
            
            // find error
            error = targetCounts - currentPosition;

            if (Math.abs(error) < 10) break; // stop if error is within 10 ticks

            // pid formula
            integral += error;
            derivative = error - lastError;
            power = (driveKp * error) + (driveKi * integral) + (driveKd * derivative);

            // clamp power to 60%
            power = Math.max(-0.6, Math.min(0.6, power));

            // set power
            leftMotor.setPower(power);
            rightMotor.setPower(power);

            // telemetry
            telemetry.addData("Drive Error", error);
            telemetry.addData("Power", power);
            telemetry.update();

            // save error
            lastError = error;
        }

        // stop motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void pidTurn(double degrees) {
        // find target tick value
        double targetCounts = degrees * COUNTS_PER_DEGREE;

        // reset encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // initialize variables
        double error = 0;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;
        double power = 0;

        while (opModeIsActive()) {
            // get current position
            double currentPosition = (leftMotor.getCurrentPosition() - rightMotor.getCurrentPosition()) / 2.0;
            
            // find error
            error = targetCounts - currentPosition;

            if (Math.abs(error) < 10) break; // stop if error is within 10 ticks

            // pid formula
            integral += error;
            derivative = error - lastError;
            power = (turnKp * error) + (turnKi * integral) + (turnKd * derivative);

            // clamp power to 40%
            power = Math.max(-0.4, Math.min(0.4, power));

            // set power
            leftMotor.setPower(power);
            rightMotor.setPower(-power);

            // telemetry
            telemetry.addData("Turn Error", error);
            telemetry.addData("Power", power);
            telemetry.update();

            // save error
            lastError = error;
        }

        // stop motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void gyroTurn(double targetAngle) {

        // initialize variables
        double error = 0;
        double lastError = 0;
        double integral = 0;
        double derivative = 0;
        double power = 0;

        // run motor without built in pid
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            // get current heading
            double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // find error
            error = targetAngle - currentHeading;

            // normalize error to [-180, 180]
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            if (Math.abs(error) < 1.0) break; // stop if error is within 1 degree

            // pid formula
            integral += error;
            derivative = error - lastError;
            power = (turnKp * error) + (turnKi * integral) + (turnKd * derivative);

            // clamp power to 40%
            power = Math.max(-0.4, Math.min(0.4, power));

            // set power
            leftMotor.setPower(power);
            rightMotor.setPower(-power);

            // telemetry
            telemetry.addData("Turn Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Target Heading", targetAngle);
            telemetry.update();

            // save error
            lastError = error;
        }

    // stop motors
    leftMotor.setPower(0);
    rightMotor.setPower(0);
}
    
}
