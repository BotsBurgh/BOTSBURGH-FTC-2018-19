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
        telemetry.addData(">", "Press start");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            arm.setPower(aPower);
            extend.setPower(ePower);

            if (gamepad1.a) {
                move(arm, aPower, 1);
            } else if (gamepad1.b) {
                move(arm, aPower, -1);
            }
            if (gamepad1.x) {
                move(extend, ePower, 1);
            } else if (gamepad1.y) {
                move(extend, ePower, -1);
            }
        }
    }
    public void move(DcMotor motor, double speed, int tic) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = motor.getCurrentPosition() + tic;
            motor.setTargetPosition(target);

            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            motor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (motor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d", target);
                telemetry.addData("Path2",  "Running at %7d", motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor.setPower(0);

            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
