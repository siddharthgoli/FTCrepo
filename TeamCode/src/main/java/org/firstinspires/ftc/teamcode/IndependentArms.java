package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class IndependentArms extends LinearOpMode {

    static double intakePoseX = 0.0, intakePoseY = 0.0;
    static double outtakePoseX = 60.0, outtakePoseY = 700.0;

    static double[] intakePose = {intakePoseX, intakePoseY};
    static double[] outtakePose = {outtakePoseX, outtakePoseY};

    // Fixed anchors (mm)
    final double[] P1 = {0.0, 0.0};
    final double[] P2 = {62.039, -43.9};

    // Unfixed points (mm)
    double[] P3, P4, P5, P6;

    // Link lengths (mm)
    final double L23 = 68.356;
    final double L34 = 97.578;
    final double L14 = 110.0;
    final double L45 = 266.714;
    final double L15 = L14 + L45;
    final double L56 = 384.663;

    DcMotor stage1, stage2;
    DcMotor stage1Enc, stage2Enc;

    double COUNTS_PER_DEGREE = 8192.0 / 360;
    PIDController stage1Ctrl, stage2Ctrl;

    static double lP = 0.01, lI = 0, lD = 0;
    static double uP = 0.01, uI = 0, uD = 0;

    enum States {
        INTAKE,
        OUTTAKE
    }
    States state;

    boolean solve;

    @Override
    public void runOpMode() throws InterruptedException {

        stage1 = hardwareMap.get(DcMotor.class, "stage1");
        stage2 = hardwareMap.get(DcMotor.class, "stage2");
        stage1.setDirection(DcMotor.Direction.REVERSE);
        stage2.setDirection(DcMotor.Direction.FORWARD);

        stage1Enc  = hardwareMap.get(DcMotor.class, "stage1Enc");
        stage2Enc = hardwareMap.get(DcMotor.class, "stage2Enc");
        stage1Enc.setDirection(DcMotor.Direction.FORWARD);
        stage2Enc.setDirection(DcMotor.Direction.REVERSE);

        stage1Ctrl = new PIDController(lP, lI, lD);
        stage2Ctrl = new PIDController(uP, uI, uD);

        state = States.INTAKE;
        solve = true;

        solveGivenP6(intakePose);

        while (opModeInInit()) {

            stage1.setPower(
                    stage1Ctrl.calculate(
                            stage1Enc.getCurrentPosition(),
                            angle(P1, P5) * COUNTS_PER_DEGREE
                    )
            );
            stage2.setPower(
                    stage2Ctrl.calculate(
                            stage2Enc.getCurrentPosition(),
                            angle(P5, P6) * COUNTS_PER_DEGREE
                    )
            );

        }

        waitForStart();

        while (opModeIsActive()) {
            switch (state) {
                case INTAKE:
                    if (solve) {
                        solveGivenP6(intakePose);
                        solve = false;
                    }

                    if (gamepad1.a) {
                        state = States.OUTTAKE;
                        solve = true;
                    }
                    break;
                case OUTTAKE:
                    if (solve) {
                        solveGivenP6(outtakePose);
                        solve = false;
                    }


                    if (gamepad1.a) {
                        state = States.INTAKE;
                        solve = true;
                    }
                    break;

            }

            stage1.setPower(
                    stage1Ctrl.calculate(
                            stage1Enc.getCurrentPosition(),
                            angle(P1, P5) * COUNTS_PER_DEGREE
                    )
            );
            stage2.setPower(
                    stage2Ctrl.calculate(
                            stage2Enc.getCurrentPosition(),
                            angle(P5, P6) * COUNTS_PER_DEGREE
                    )
            );

        }

    }


    double dist(double[] a, double[] b) {
        return Math.hypot(a[0] - b[0], a[1] - b[1]);
    }

    double angle(double[] a, double[] b) {
        return Math.toDegrees(
                Math.acos(
                        (a[0] - b[0]) / dist(a, b)
                )
        );
    }

    void solveGivenP6(double[] P6t) {
        double d16 = dist(P1, P6t);

        // --- Triangle P1-P5-P6 ---
        double cosA = (L15*L15 + d16*d16 - L56*L56) / (2 * L15 * d16);
        cosA = Math.max(-1, Math.min(1, cosA));
        double A = Math.acos(cosA);

        double baseAngle = Math.atan2(P6t[1] - P1[1], P6t[0] - P1[0]);
        double theta15 = baseAngle - A;

        P5 = new double[]{P1[0] + L15 * Math.cos(theta15),
                P1[1] + L15 * Math.sin(theta15)};

        // --- Point P4 along P1->P5 ---
        P4 = new double[]{P1[0] + L14 * Math.cos(theta15),
                P1[1] + L14 * Math.sin(theta15)};

        // --- Triangle P2-P4-P3 ---
        double d24 = dist(P2, P4);
        double cosB = (L23*L23 + d24*d24 - L34*L34) / (2 * L23 * d24);
        cosB = Math.max(-1, Math.min(1, cosB));
        double B = Math.acos(cosB);

        double baseAngle2 = Math.atan2(P4[1] - P2[1], P4[0] - P2[0]);

        // >>> flipped branch (minus instead of plus)
        double theta23 = baseAngle2 - B;

        P3 = new double[]{P2[0] + L23 * Math.cos(theta23),
                P2[1] + L23 * Math.sin(theta23)};
    }
}