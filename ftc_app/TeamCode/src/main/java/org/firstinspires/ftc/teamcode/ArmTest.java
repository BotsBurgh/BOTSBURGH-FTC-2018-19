package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {
    final private static double ARM_POWER     = 0.1;  // Base power sent to arm. Will be adjusted.
    final private static double EXTEND_POWER  = 0.6;  // Extending power/speed
    final private static int    EXTEND_TIC    = 2000; // Extend distance (in tics)
    final private static double ARM_MAX       = 90.0; // The degrees that the arm is at it's maximum angle
    final private static double ARM_MIN       = 0.0;  // The degrees that the arm is at it's minimum angle
    final private static double FREEZE_THRESH = 5.0;  // The play in the arm (for preventing it from moving)
    final private static double FREEZE_STEP   = 0.0001; // The step value for the arm freezing

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
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Adjusting the potentiometer
        adjusted = pot.getPot();
        diff = pot.getPot();

        // Resistance variable when the arm is not moving
        resistance = 0;
        current = 0;

        extendsteps = 0;

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();
        //int ext = 0;
        // Start!
        while (opModeIsActive()) {
            // If the color sensor detects red, then stop all movement.
            if (redreset.getRGB().equals("red")) {
                diff = pot.getPot();
                resistance = 0;
            }

            adjusted = pot.getPot() - diff;

            // If 'a' is pressed, and the adjusted potentiometer is less than ARM_MAX
            if (gamepad1.a) {
                if (adjusted < ARM_MAX/2.0) {
                    telemetry.addData("Moving arm", "up");
                    telemetry.update();
                    arm.setPower((ARM_POWER*((extendsteps*extendsteps)/2.0)*((adjusted+1)/500)));
                } else if (adjusted < ARM_MAX) {
                    telemetry.addData("Moving arm", "down");
                    telemetry.update();
                    arm.setPower((ARM_POWER*((extendsteps*extendsteps)/2.0)*((adjusted+1)/1000)));
                }
                current = adjusted;
            // If 'b' is pressed, and the adjusted potentiometer is more than ARM_MIN
            } else if ((gamepad1.b) && (adjusted > ARM_MIN)) {
                if (adjusted > ARM_MAX/2.0) {
                    arm.setPower(-(ARM_POWER+(extendsteps*0.05)));
                } else if (adjusted < ARM_MAX/2.0) {
                    arm.setPower(-(ARM_POWER+(extendsteps*0.01)));
                }
                current = adjusted;
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

            if ((gamepad1.x) && (extendsteps<3)) {
                if ((adjusted < 15) && (adjusted > 0)) {
                    arm.setPower(0.2);
                    sleep(250);
                    arm.setPower(0);
                }
                moveExt(extend, EXTEND_POWER, EXTEND_TIC);
                extendsteps+=1;
            } else if ((gamepad1.y) && (extendsteps>0)) {
                moveExt(extend, EXTEND_POWER, -EXTEND_TIC);
                extendsteps-=1;
            } else {
                extend.setPower(0);
            }

            telemetry.addData("Reset", limit.isPressed()); // Reset the button
            telemetry.addData("Real", pot.getPot()); // Get the angle from the potentiometer
            telemetry.addData("Adjusted", adjusted); // Get the adjusted angle from the potentiometer
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
        }
    }
}
