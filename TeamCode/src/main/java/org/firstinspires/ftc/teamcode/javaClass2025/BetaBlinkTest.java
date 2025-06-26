package org.firstinspires.ftc.teamcode.javaClass2025;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class BetaBlinkTest extends LinearOpMode {
    
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("lfm");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("lbm");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rfm");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rbm");
        
        Servo servo = hardwareMap.servo.get("clawPiv");
        
        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        waitForStart();
        
        if (isStopRequested()) return;
        
        boolean pressed = false;
        boolean toggled = false;
        
        while (opModeIsActive()) {
            
            if(gamepad1.a){
                pressed = true;
            }
            
            if(pressed && !gamepad1.a){
                toggled = !toggled;
            }
            
            servo.setPosition(toggled ? 0 : 1);
            
            
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}