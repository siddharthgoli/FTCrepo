package org.firstinspires.ftc.teamcode.puneethAssignment;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriveControl {

    Drivebase driveBase;

    public DriveControl(Drivebase driveBase) {
        this.driveBase = driveBase;
    }

    public void update(Gamepad gp1) {
        driveBase.setPower(-gp1.left_stick_y,  gp1.left_stick_x, gp1.right_stick_y);
    }
}
