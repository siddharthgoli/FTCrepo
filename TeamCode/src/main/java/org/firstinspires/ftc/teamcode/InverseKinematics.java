package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
public class InverseKinematics extends LinearOpMode {

    DcMotor stage1; // 60 rpm, 1:1 outside, 1425.1 ticks
    DcMotor stage2; // 117 rpm, 1:1 outside, 2786.2 ticks

    DcMotor stage2Encoder;

    DcMotor linkageEncoder;

    double COUNTS_PER_RADIAN = Math.toRadians( 8192.0 / 360 );

    PIDController stage2Controller, linkageController;
    static double sP = 0, sI = 0, sD = 0, lP = 0, lI = 0, lD = 0;


    static double outtakePoseX = 10, outtakePoseY = 200;


    static double stagesPoseX = 0, stagesPoseY = 0;

    double shaftToShaftX = 62.039; // mm
    double shaftToShaftY = 43.900; // mm
    double shaftToShaft = pythagorean( shaftToShaftX, shaftToShaftY ); // 76 mm
    double lowerLinkage = 68.356; // mm
    double upperLinkage = 97.578; // mm

    static double desiredLinkageV4BPoseX, desiredLinkageV4BPoseY;
    double lowerStage = 376.714; // mm
    double upperStage = 384.663; // mm
















    @Override
    public void runOpMode() throws InterruptedException {

        stage1 = hardwareMap.get(DcMotor.class, "stage1");
        stage2 = hardwareMap.get(DcMotor.class, "stage2");

        stage1.setDirection(DcMotor.Direction.REVERSE);
        stage2.setDirection(DcMotor.Direction.FORWARD);

        stage2Encoder  = hardwareMap.get(DcMotor.class, "stage2Encoder");
        linkageEncoder = hardwareMap.get(DcMotor.class, "linkageEncoder");

        stage2Encoder.setDirection(DcMotor.Direction.FORWARD);
        linkageEncoder.setDirection(DcMotor.Direction.REVERSE);

        stage2Controller  = new PIDController(sP, sI, sD);
        linkageController = new PIDController(lP, lI, lD);


        while (opModeInInit()) {

        }

        waitForStart();

        while (opModeIsActive()) {

            double stages = distance(outtakePoseX, outtakePoseY, 0, 0);

            if (!(stages > upperStage + lowerStage)) {

                double stagesAngle =
                        Math.acos((stages*stages - upperStage * upperStage - lowerStage * lowerStage) / -2 * upperStage * lowerStage);

                double stagesPhi = Math.atan( ( outtakePoseY ) / ( outtakePoseY ) );

                stagesPoseX = outtakePoseX + upperLinkage * Math.cos( stagesPhi - stagesAngle );
                stagesPoseY = outtakePoseX + upperLinkage * Math.sin( stagesPhi - stagesAngle );

                double desiredUpperStagePhi = Math.atan( ( outtakePoseY - stagesPoseY ) / ( outtakePoseX - stagesPoseX ) ); // stage 2 motor

                double desiredLowerStagePhi = Math.atan( ( stagesPoseY ) / ( stagesPoseX ) );

                desiredLinkageV4BPoseX = 110 * Math.cos( desiredLowerStagePhi );
                desiredLinkageV4BPoseY = 110 * Math.sin( desiredLowerStagePhi );



                double linkageAngle = linkageEncoder.getCurrentPosition() * COUNTS_PER_RADIAN; // i know bc of forward kinematics (encoder pos)


                double linkageLine = Math.sqrt( lowerLinkage*lowerLinkage + shaftToShaft*shaftToShaft - 2*lowerLinkage*shaftToShaft*Math.cos( linkageAngle ) );

                double shaftsPhi = Math.atan( ( -43.9 ) / ( 62.039 ) );

                double linkagesAngle =
                        Math.acos((shaftToShaft*shaftToShaft - linkageLine*linkageLine - lowerLinkage*lowerLinkage) / -2 * linkageLine*lowerLinkage);

                double linkagesPoseX = linkageLine * Math.cos( shaftsPhi - linkagesAngle );
                double linkagesPoseY = linkageLine * Math.sin( shaftsPhi - linkagesAngle );


                double linkagesPhi = Math.atan( ( linkagesPoseY ) / ( linkagesPoseX ) );

                double stage1LinkageAngle =
                        Math.acos(( linkageLine*linkageLine - 110*110 - upperLinkage*upperLinkage) / -2 * 110*upperLinkage);

                double stage1LinkagePoseX = 110 * Math.cos( linkagesPhi - stage1LinkageAngle );
                double stage1LinkagePoseY = 110 * Math.sin( linkagesPhi - stage1LinkageAngle );

                double currentLowerStagePhi = 110 * Math.cos( stage1LinkagePoseY / stage1LinkagePoseX );

                double stage2PID = stage2Controller.calculate( stage2Encoder.getCurrentPosition(), desiredUpperStagePhi * COUNTS_PER_RADIAN);
                double linkagePID = linkageController.calculate( currentLowerStagePhi / COUNTS_PER_RADIAN, desiredLowerStagePhi / COUNTS_PER_RADIAN );

                stage2.setPower(stage2PID);
                stage1.setPower(linkagePID);

            }

        }

    }

    public double distance(double x1, double y1, double x2, double y2) {
        return Math.sqrt( Math.pow( x1-x2, 2 ) + Math.pow( y1-y2, 2 ) );
    }

    public double pythagorean(double l1, double l2) {
        return Math.sqrt( l1*l1 + l2*l2 );
    }
}
