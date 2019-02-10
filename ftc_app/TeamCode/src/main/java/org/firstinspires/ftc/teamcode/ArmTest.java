package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {
    static final double THRESH = 5;
    static final double STEP   = 0.05;
    DcMotor extend, arm;
    double aPower = 1.0;
    double ePower = 0.3;
    double c=0;
    Sensor pot, limit;
    @Override
    public void runOpMode() {
        limit = new Sensor(hardwareMap.get(DigitalChannel.class, "lim1"));
        pot = new Sensor(hardwareMap.get(AnalogInput.class, "pot"));

        extend = hardwareMap.get(DcMotor.class,"extend");
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();

        // Start!
        while (opModeIsActive()) {
            arm.setPower(aPower);
            extend.setPower(ePower);

            if (gamepad1.a) {
                moveArm(arm, aPower);
            } else if (gamepad1.b) {
                moveArm(arm, -aPower);
            } else {
                freezeArm();
            }

            if (gamepad1.x) {
                moveExt(extend, ePower, 50);
            } else if (gamepad1.y) {
                moveExt(extend, ePower, -50);
            } else {
                extend.setPower(0);
            }
        }
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveArm(DcMotor motor, double speed) {
        double zero = pot.getPot();

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Reset the encoder
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // reset the timeout time and start motion.
            motor.setPower(speed);

            // Loop while the arm is in range, the opmode is running, and the limit switch is not pressed.
            while (opModeIsActive() && limit.isPressed() && (pot.getPot() < zero+50) && (pot.getPot() > zero)) {
                // Display it for the driver.
                telemetry.addData("Arm current", "Running at %7d", motor.getCurrentPosition());
                telemetry.addData("Arm degree", pot.getPot());
                telemetry.update();
            }

            // Stop all motion;
            motor.setPower(0);
        }
    }

    public void freezeArm() {
        double zero = pot.getPot();
        if ((pot.getPot()-zero)>THRESH) {
            arm.setPower(c);
            c+=STEP;
        } else if ((pot.getPot()-zero)<THRESH) {
            arm.setPower(-c);
            c-=STEP;
        } else {
            c=0;
        }
    }

    public void moveExt(DcMotor motor, double speed, int tic) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = motor.getCurrentPosition() + tic;
            motor.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            motor.setPower(Math.abs(speed));

            // keep looping while we are still active, there is time left, and both motors are running.
            while (opModeIsActive() && (motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Extend to",  "Running to %7d", target);
                telemetry.addData("Extend current",  "Running at %7d", motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor.setPower(0);
        }
    }
}
