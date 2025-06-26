package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDdemo extends LinearOpMode {

    DcMotor motor;
    ElapsedTime time = new ElapsedTime();

    static double kP = 0.0;
    static double kI = 0.0;
    static double kD = 0.0;

    double error, lastError;
    double integral = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {

            motor.setPower(pid(500));-
        }

    }

    public double pid(double desiredPos) {


        double currentPos = motor.getCurrentPosition();
        error = desiredPos - currentPos;


        integral += error * time.milliseconds();
        double derivative = (error - lastError) / time.milliseconds();
        double power = (kP * error) + (kP * integral) + (kP * derivative);



        lastError = error;


        return power;
    }
}