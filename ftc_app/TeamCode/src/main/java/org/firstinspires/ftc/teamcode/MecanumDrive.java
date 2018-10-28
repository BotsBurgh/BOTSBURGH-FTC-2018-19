package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Drive Test", group = "Linear OpMode")
public class MecanumDrive extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Init Motors
        frontLeft = hardwareMap.get(DcMotor.class,"fl");
        frontRight = hardwareMap.get(DcMotor.class,"fr");
        backLeft = hardwareMap.get(DcMotor.class,"bl");
        backRight = hardwareMap.get(DcMotor.class,"br");

        waitForStart();
        while(opModeIsActive()) {
            // Determine magnitude of motion
            double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
            // Determine angle from x and y then subtracts pi/4
            double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) ;
            double rotation = gamepad1.right_stick_x;


            double fl = r*Math.cos(angle)+rotation;
            double fr = r*Math.sin(angle)-rotation;
            double bl = r*Math.sin(angle)+rotation;
            double br = r*Math.cos(angle)-rotation;

            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);
        }

    }

}
