package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {
    final private static double ARM_POWER     = 0.75;   // Base power sent to arm. Will be adjusted.
    final private static double EXTEND_POWER  = 0.6;   // Extending power/speed
    final private static int    EXTEND_TIC    = 2000;  // Extend distance (in tics)
    final private static double ARM_MAX       = 90.0;  // The degrees that the arm is at it's maximum angle
    final private static double ARM_MIN       = -50000.0;  // The degrees that the arm is at it's minimum angle
    final private static double FREEZE_THRESH = 5.0;   // The play in the arm (for preventing it from moving)
    final private static double FREEZE_STEP   = 0.001; // The step value for the arm freezing

    @Override
    public void runOpMode() {
        DcMotor extend, arm;
        double adjusted, diff, resistance, current;
        Sensor pot, limit, redreset;
        int extendsteps;

        // Get sensors
        limit = new Sensor(hardwareMap.get(DigitalChannel.class, "lim1")); // Limit button
        pot = new Sensor(hardwareMap.get(AnalogInput.class, "pot")); // Potentiometer
        redreset = new Sensor(hardwareMap.get(ColorSensor.class, "reddeadreset"));

        // Motor for extending the arm
        extend = hardwareMap.get(DcMotor.class,"extend");
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor for tilting the arm
        arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Adjusting the potentiometer
        adjusted = pot.getPot();
        diff = pot.getPot();

        // Resistance variable when the arm is not moving
        resistance = 0;
        current = 0;

        extendsteps = 1;

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();
        diff = pot.getPot();

        arm.setPower(0.3);
        sleep(250);
        arm.setPower(0.15);

        // Start!
        while (opModeIsActive()) {
            arm.setPower(0.2);
            // If the color sensor detects red, then stop all movement.
            if (redreset.getRGB().equals("red")) {
                diff = pot.getPot()-ARM_MAX;
                resistance = 0;
            }
            adjusted = pot.getPot() - diff;

            // If 'a' is pressed, and the adjusted potentiometer is less than ARM_MAX
            if (gamepad1.a) {
                telemetry.addData("Moving arm", "down");
                telemetry.update();
                /*
                if (adjusted < ARM_MAX/2.0) {
                    arm.setPower((ARM_POWER*((extendsteps*extendsteps)/2.0)*((adjusted+1)/500)));
                } else if (adjusted < ARM_MAX) {
                    telemetry.addData("Moving arm", "down");
                    telemetry.update();
                    arm.setPower((ARM_POWER*((extendsteps*extendsteps)/2.0)*((adjusted+1)/1000)));
                }
                */
                //arm.setPower(ARM_POWER/1.5);
                //current = adjusted;
                moveExt(arm, ARM_POWER, 2000);
            // If 'b' is pressed, and the adjusted potentiometer is more than ARM_MIN
            } else if ((gamepad1.b) && (adjusted > ARM_MIN)) {
                telemetry.addData("Moving arm", "up");
                //telemetry.update();
                //arm.setPower(-ARM_POWER);
                //current = adjusted;
                moveExt(arm, ARM_POWER, -2000);
            // Resist movement
            } else {
                if (((adjusted - current) < 0) && (Math.abs(adjusted-current) > FREEZE_THRESH)) {
                    resistance -= FREEZE_STEP;
                } else if (((adjusted - current) > 0) && (Math.abs(adjusted-current) > FREEZE_THRESH)) {
                    resistance += FREEZE_STEP;
                } else {
                    resistance = 0;
                }
                arm.setPower(resistance);
            }

            if ((gamepad1.x) && (extendsteps > 1)) {
                moveExt(extend, EXTEND_POWER, -EXTEND_TIC);
                extendsteps-=1;
            } else if ((gamepad1.y) && (extendsteps < 5)) {
                moveExt(extend, EXTEND_POWER, EXTEND_TIC);
                extendsteps+=1;
            } else {
                extend.setPower(0);
            }

            arm.setPower(0);

            telemetry.addData("Reset", redreset.getRGB().equals("red")); // Reset the potentiometer
            telemetry.addData("Real", pot.getPot()); // Get the angle from the potentiometer
            telemetry.addData("Adjusted", adjusted); // Get the adjusted angle from the potentiometer
            telemetry.addData("Extended Steps", extendsteps);
            telemetry.update();
        }
    }

    public void moveExt(DcMotor motor, double speed, int tic) {
        int target;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            target = motor.getCurrentPosition() + tic;
            motor.setTargetPosition(target);

            // reset the timeout time and start motion.
            motor.setPower(Math.abs(speed));

            // keep looping while we are still active, there is time left, and both motors are running.
            while (opModeIsActive() && (motor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Extend to", "Running to %7d", target);
                telemetry.addData("Extend current", "Running at %7d", motor.getCurrentPosition());
                telemetry.update();
            }
            motor.setPower(0);
        }
    }
}
