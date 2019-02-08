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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Mecanum Drive", group = "Linear OpMode")
public class MecanumDrive extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private BNO055IMU gyro;


    @Override
    public void runOpMode() {
        // Init Motors
        motorFL = hardwareMap.get(DcMotor.class,"fl");
        motorFR = hardwareMap.get(DcMotor.class,"fr");
        motorBL = hardwareMap.get(DcMotor.class,"bl");
        motorBR = hardwareMap.get(DcMotor.class,"br");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Movement movement = new Movement(motorFL, motorFR, motorBL, motorBR, gyro);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            // Uses gamepad 2 to make it constant speed

            double x2 = gamepad2.left_stick_x;
            double y2 = gamepad2.left_stick_y;
            double rotation2 = gamepad2.right_stick_y;
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


            double flPower = y1 - x1-rotation;
            double frPower = y1 + x1+rotation;
            double blPower = y1 + x1-rotation;
            double brPower = y1 - x1+rotation;


            movement.quadMove(flPower, frPower, blPower, brPower);
            telemetry.addData("Back Left", motorBL.getCurrentPosition());
            telemetry.addData("Back Right",motorBL.getCurrentPosition());
            telemetry.addData("Front Left",motorFL.getCurrentPosition());
            telemetry.addData("FRont right", motorFR.getCurrentPosition());
        }

    }

}
