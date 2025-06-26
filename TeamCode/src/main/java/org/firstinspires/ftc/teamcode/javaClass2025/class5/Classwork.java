package org.firstinspires.ftc.teamcode.javaClass2025.class5;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Classwork extends LinearOpMode {
    
    // use falling edge detectors and a p controller to have a motor go between
    // two positions when you press triangle and circle
    
    public DcMotor m1;
    
    double kP = 0.01;
    
    double desiredPosition = 0;
    
    boolean circlePressed, trianglePressed;
    
    @Override
    public void runOpMode() throws InterruptedException {
        
        m1 = hardwareMap.get(DcMotor.class, "m1");
        
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        waitForStart();
        
        while (opModeIsActive()) {
            
            if (gamepad1.circle) {
                circlePressed = true;
            }
            if (!gamepad1.circle && circlePressed) {
                
                desiredPosition = 100;
                
                circlePressed = false;
            }
            
            if (gamepad1.triangle) {
                trianglePressed = true;
            }
            if (!gamepad1.triangle && trianglePressed) {
                
                desiredPosition = 500;
                
                trianglePressed = false;
            }
            
            m1.setPower((desiredPosition - m1.getCurrentPosition()) * kP);
            
            
        }
        
    }
    
}
