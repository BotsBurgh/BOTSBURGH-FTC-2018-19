package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {
    final static double ARMPOWER = 0.8;
    final static double EXTENDPOWER = 0.2;
    DcMotor extend, arm;
    double c=0;
    double adjusted, diff;
    Sensor pot, limit;
    double resistance;
    @Override
    public void runOpMode() {
        limit = new Sensor(hardwareMap.get(DigitalChannel.class, "lim1"));
        pot = new Sensor(hardwareMap.get(AnalogInput.class, "pot"));

        extend = hardwareMap.get(DcMotor.class,"extend");
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotor.class,"arm");
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        adjusted = pot.getPot();
        diff = pot.getPot();

        resistance = 0;

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        // Start!
        while (opModeIsActive()) {
            if (limit.isPressed()) {
                diff = pot.getPot();
                resistance = 0;
            }
            adjusted = pot.getPot() - diff;

            if (gamepad1.a) {
                if (adjusted < 70.0) {
                    arm.setPower(ARMPOWER);
                } else {
                    arm.setPower(resistance);
                }
            } else if (gamepad1.b) {
                if (adjusted != 0.0) {
                    arm.setPower(-ARMPOWER);
                } else {
                    arm.setPower(resistance);
                }
            } else {
                arm.setPower(0);
            }

            if (gamepad1.x) {
                //moveExt(extend, ePower, 50);
            } else if (gamepad1.y) {
                //moveExt(extend, ePower, -50);
            } else {
                extend.setPower(0);
            }

            telemetry.addData("Reset", limit.isPressed());
            telemetry.addData("Real", pot.getPot()); // Get the angle from the other file
            telemetry.addData("Adjusted", adjusted);
            telemetry.update();
        }
    }
}
