/*
Copyright 2019 FIRST Tech Challenge Team 11792
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/



package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Mecanum Drive", group = "Linear OpMode")
public class MecanumDrive extends LinearOpMode {
    static final int    CYCLE_MS    =   50;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private BNO055IMU gyro;
    static final int UP = 1;
    static final int DOWN = 0;

    // Define class members
    DcMotor motor;
    double  power   = 0;
    // Defaults to moving up
    int direction = DOWN;
    boolean switched = false;

    ColorSensor sensorColor;
    DistanceSensor sensorDistance;

    final private static double ARM_POWER     = 0.4;   // Base power sent to arm. Will be adjusted.
    final private static double EXTEND_POWER  = 0.6;   // Extending power/speed
    final private static int    EXTEND_TIC    = 2000;  // Extend distance (in tics)
    final private static double ARM_MAX       = 900.0;  // The degrees that the arm is at it's maximum angle
    final private static double ARM_MIN       = -50000.0;  // The degrees that the arm is at it's minimum angle
    final private static double FREEZE_THRESH = 5.0;   // The play in the arm (for preventing it from moving)
    final private static double FREEZE_STEP   = 0.001; // The step value for the arm freezing


    @Override
    public void runOpMode() {
        DcMotor extend, arm;
        double adjusted, diff, resistance, current;
        Sensor pot, limit, redreset;
        int extendsteps;

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

        extendsteps = 1;

        // Init Motors
        motorFL = hardwareMap.get(DcMotor.class,"fl");
        motorFR = hardwareMap.get(DcMotor.class,"fr");
        motorBL = hardwareMap.get(DcMotor.class,"bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        //Sensor superSensorColor = new Sensor(sensorColor);
        //superSensorColor.getRGB();

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor = hardwareMap.get(DcMotor.class, "elevator");

        Movement movement = new Movement(motorFL, motorFR, motorBL, motorBR, gyro);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData(">", "Press start");
        telemetry.update();

        waitForStart();

        telemetry.addData(">", "Press stop");
        telemetry.update();
        int ext = 0;

        waitForStart();
        while(opModeIsActive()) {

            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = Range.clip((y1 - x1-rotation),-.8,.8);
            double frPower = Range.clip((y1 + x1+rotation),-.8,.8);
            double blPower = Range.clip((y1 + x1-rotation),-.8,.8);
            double brPower = Range.clip((y1 - x1+rotation),-.8,.8);

            if(sensorColor.red()> 40 && (sensorColor.red()>sensorColor.green()) && sensorColor.red() > sensorColor.blue()) {
                if (direction == UP && !switched) { direction = DOWN; switched = true;}
                else if(direction == DOWN && !switched) {direction = UP; switched = true;}
                telemetry.addData("Can Go UP", direction == UP);
            } else {
                switched = false;
            }
            if((gamepad2.left_stick_y > 0 && direction == DOWN) || gamepad2.left_stick_y < 0 && direction == UP || gamepad2.left_stick_y==0) {
                power = 0;
            } else {
                power = gamepad2.left_stick_y;
            }
            // Let User manually change direction
            if(gamepad2.dpad_up) {
                direction = UP;
            } else if(gamepad2.dpad_down) {
                direction = DOWN;
            }

            motor.setPower(power);
            if (limit.isPressed()) {
                diff = pot.getPot();
                resistance = 0;
            }
            adjusted = pot.getPot() - diff;

            if (gamepad2.right_stick_y<0) {
                if (adjusted < 90) {
                    arm.setPower(.3*gamepad2.right_stick_y-.1*ext);
                } else {
                    arm.setPower(resistance);
                }
            } else if (gamepad2.right_stick_y>0) {
                if (adjusted > 0) {
                    arm.setPower(.3*gamepad2.right_stick_y-.1*ext);
                } else {
                    arm.setPower(resistance);
                }
            } else {
                    if(adjusted < 45) {
                        resistance = 0;
                        arm.setPower(resistance);
                    }
                    else if(adjusted < 60 ) {
                        resistance = 0;
                        arm.setPower(resistance);
                    }
                    else if(adjusted > 90) {
                        resistance = .2;
                        arm.setPower(resistance);
                    }

                }


            arm.setPower(0.2);
            // If the color sensor detects red, then stop all movement.
            if (redreset.getRGB().equals("red")) {
                diff = pot.getPot()-ARM_MAX;
                resistance = 0;
            }
            adjusted = pot.getPot() - diff;

            // If 'a' is pressed, and the adjusted potentiometer is less than ARM_MAX
            if (gamepad2.right_stick_y > 0.1) {
                telemetry.addData("Moving arm", "down");
                telemetry.update();
                arm.setPower(ARM_POWER*gamepad2.right_stick_y);
                current = adjusted;
                // If 'b' is pressed, and the adjusted potentiometer is more than ARM_MIN
            } else if (gamepad2.right_stick_y < -0.1) {
                telemetry.addData("Moving arm", "up");
                telemetry.update();
                arm.setPower(-ARM_POWER*gamepad2.right_stick_y);
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

            if ((gamepad2.dpad_left) && (extendsteps > 1)) {
                moveExt(extend, EXTEND_POWER, -EXTEND_TIC);
                extendsteps-=1;
            } else if ((gamepad2.dpad_right) && (extendsteps < 5)) {
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

            movement.quadMove(flPower, frPower, blPower, brPower);
            telemetry.addData("Back Left", motorBL.getCurrentPosition());
            telemetry.addData("Back Right",motorBR.getCurrentPosition());
            telemetry.addData("Front Left",motorFL.getCurrentPosition());
            telemetry.addData("Front right", motorFR.getCurrentPosition());
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
