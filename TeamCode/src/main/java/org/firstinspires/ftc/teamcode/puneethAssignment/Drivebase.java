package org.firstinspires.ftc.teamcode.puneethAssignment;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

public class Drivebase {

    MecanumDrive follower;

    DcMotor lf, lb, rf, rb;
    double lfPower, lbPower, rfPower, rbPower;
    double denominator;

    public Drivebase(MecanumDrive f) {
        this.follower = f;

        lf = follower.leftFront;
        lb = follower.leftBack;
        rf = follower.rightFront;
        rb = follower.rightBack;

    }

    public void setPower(double y, double x, double rx) {
        denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        lfPower = (y + x + rx) / denominator;
        lbPower = (y - x + rx) / denominator;
        rfPower = (y - x - rx) / denominator;
        rbPower = (y + x - rx) / denominator;

        lf.setPower(lfPower);
        lb.setPower(lbPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
    }
}
