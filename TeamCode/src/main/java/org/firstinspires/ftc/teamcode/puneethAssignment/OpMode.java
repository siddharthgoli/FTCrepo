package org.firstinspires.ftc.teamcode.puneethAssignment;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@TeleOp(name="PuneethTeleOp", group="")
public class OpMode extends LinearOpMode {
    HardwareMap hardwareMap;

    DriveControl driveControl;

    @Override
    public void runOpMode() throws InterruptedException {

        driveControl = new DriveControl(
                new Drivebase(
                        new MecanumDrive(
                                hardwareMap,
                                new Pose2d(0, 0, 0)
                        )
                )
        );

        waitForStart();

        while (opModeIsActive()) {

            driveControl.update(gamepad1);
        }

    }
}
