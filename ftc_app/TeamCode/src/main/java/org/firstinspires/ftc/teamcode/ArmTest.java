package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {
    DcMotor extend, arm;
    double aPower = 0.3;
    double ePower = 0.3;
    @Override
    public void runOpMode() {
        extend = hardwareMap.get(DcMotor.class,"extend");
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(aPower);
        extend.setPower(ePower);
        telemetry.addData(">", "Press start");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                arm.setTargetPosition(arm.getCurrentPosition()+1);
            } else if (gamepad1.b) {
                arm.setTargetPosition(arm.getCurrentPosition()-1);
            }
            if (gamepad1.x) {
                extend.setTargetPosition(extend.getCurrentPosition()+1);
            } else if (gamepad1.y) {
                extend.setTargetPosition(extend.getCurrentPosition()-1);
            }
            telemetry.addData("Arm pos", arm.getCurrentPosition());
            telemetry.addData("Extend pos", extend.getCurrentPosition());
            telemetry.update();
        }

    }
}
