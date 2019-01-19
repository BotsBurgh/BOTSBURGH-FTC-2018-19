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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
// TODO: JavaDoc + comments
@TeleOp(name = "Apache", group = "Linear OpMode")
public class Apache extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Servo s1, s2, s3;
    CRServo wl, wr;
    DcMotor elev;
    double pos=0;
    boolean scanD=true;

    // Camera detection.

    // DROP DOWN
    // use the second motor to pull itself down
    // turn it until you reach 130 deg

    // Then UNHOOK
    // Move both motors to coil itself back up
    // Not sure that it will coil itself back up.

    @Override
    public void runOpMode() {
        // Init Servos
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        wl = hardwareMap.get(CRServo.class, "wl");
        wr = hardwareMap.get(CRServo.class, "wr");
        elev = hardwareMap.get(DcMotor.class,"elev_f");

        wr.setDirection(CRServo.Direction.FORWARD);
        wl.setDirection(CRServo.Direction.FORWARD);
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.FORWARD);
        elev.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elev.setDirection(DcMotor.Direction.FORWARD);

        Movement arm = new Movement(s1, s2, wl);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        arm.armSet(0);
        wl.setPower(0);
        wr.setPower(0);
        double power = 0;
        double elevatorPower = 0;
        waitForStart();

        while(opModeIsActive()) {

            if(gamepad1.x) {
                power = 1;
            } else if(gamepad1.b) {
                power = -1;
            }
            else {
                power = 0.0;
            }


            elev.setPower(gamepad1.left_stick_x);

// Want it to rotate at constant speed.
/*
            if (gamepad1.x) {
                power += .01;

            } else if(gamepad1.b) {
                power -= 0.01;

            }
            if(power> 1) {
                power = 1;
            }
            if(power < -1) {
                power = -1;
            }

            wl.setPower(-power);
            wr.setPower(power);
            */


/*
            if(gamepad1.dpad_left) {
                power-= .01;

            } else if(gamepad1.dpad_right) {
                power += .01;
            }
            */
            wl.setPower(power);
            wr.setPower(-power);




            if (gamepad1.y) {
                pos+=0.01;
                pos=pos%270;
                arm.armSet(pos);
            }
            if (gamepad1.a) {
                pos-=0.01;
                pos=pos%270;
                arm.armSet(pos);
            }
            telemetry.addData("Position",pos);
            telemetry.addData("Power",power);
            telemetry.update();

            // Testing Servos Ignore
            /*
            s1.setDirection(Servo.Direction.FORWARD);
            s2.setDirection(Servo.Direction.REVERSE);
            s3.setDirection(Servo.Direction.REVERSE);
            s2.setPosition(0);
            s1.setPosition(0);
            s3.setPosition(0);
            telemetry.addData("Position","0");
            telemetry.update();
            sleep(1000);

            s2.setPosition(.5);
            s1.setPosition(.5);
            s3.setPosition(.5);
            telemetry.addData("Position","0.5");
            telemetry.update();
            sleep(1000);
            */
        }


    }

}
