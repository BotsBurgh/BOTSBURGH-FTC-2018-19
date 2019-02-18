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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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


    @Override
    public void runOpMode() {
        // Init Motors
        motorFL = hardwareMap.get(DcMotor.class,"fl");
        motorFR = hardwareMap.get(DcMotor.class,"fr");
        motorBL = hardwareMap.get(DcMotor.class,"bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

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

        waitForStart();
        while(opModeIsActive()) {

            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;


            /*
            if(x2>.5) {
                x1= .5;
            } else if(x2<-.5) {
                x1= -.5;
            }
            if(y2>.5) {
                y1 = .5;
            } else if(y2 < -.5) {
                y1 = -.5;
            }
            if(rotation2 > .5) {
                rotation = .5;
            } else if(rotation2 < -.5) {
                rotation = -.5;
            }
*/

            double flPower = Range.clip((y1 - x1-rotation),-.5,.5);
            double frPower = Range.clip((y1 + x1+rotation),-.5,.5);
            double blPower = Range.clip((y1 + x1-rotation),-.5,.5);
            double brPower = Range.clip((y1 - x1+rotation),-.5,.5);


            /*
            if(y1>.5) {
                 flPower = .5;
                 frPower = .5;
                 blPower = .5;
                 brPower = .5;
            } else if(y1<-.5) {
                flPower = -.5;
                frPower = -.5;
                blPower = -.5;
                brPower = -.5;
            } else {
                flPower = 0;
                frPower = 0;
                blPower = 0;
                brPower = 0;
            }
*/
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
            if(gamepad2.a) {
                direction = UP;
            }
            if(gamepad2.b) {direction = DOWN;}
            motor.setPower(power);
            movement.quadMove(flPower, frPower, blPower, brPower);
            telemetry.addData("Back Left", motorBL.getCurrentPosition());
            telemetry.addData("Back Right",motorBR.getCurrentPosition());
            telemetry.addData("Front Left",motorFL.getCurrentPosition());
            telemetry.addData("FRont right", motorFR.getCurrentPosition());
            telemetry.update();
        }



    }

}
