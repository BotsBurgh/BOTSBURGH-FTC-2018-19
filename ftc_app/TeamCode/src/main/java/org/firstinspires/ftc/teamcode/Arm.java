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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * TODO: javadoc
 */
@TeleOp(name = "Arm", group = "Linear Op Mode")

public class Arm extends LinearOpMode {

    // Define class members
    private DcMotor motorF, motorB;
    private Servo s1,s2, hook;
    private CRServo wheel;
    private double INTAKE_SPEED=0.5;

    @Override
    public void runOpMode() {
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        hook = hardwareMap.get(Servo.class,"hook");

        wheel = hardwareMap.get(CRServo.class, "wheel");

        Movement arm = new Movement(s1, s2, wheel);

        motorF = hardwareMap.get(DcMotor.class, "front");
        motorB = hardwareMap.get(DcMotor.class, "back");

        Movement base = new Movement(motorF,motorB);

        AnalogInput pot = hardwareMap.analogInput.get("potent");
        Sensors sens = new Sensors(pot);

        wheel.setDirection(CRServo.Direction.FORWARD);
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.FORWARD);
        double pos = 0;
        double hook_pos = 0;
        motorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the start button
        telemetry.addData("> ", "Press Start to run Motors.");
        telemetry.update();
        waitForStart();

        // TODO: Comments
        while(opModeIsActive()) {

            if(gamepad1.left_bumper) {
                base.armBase(-.55);
            } else if(gamepad1.right_bumper) {
                base.armBase(.55);
            } else {
                motorF.setPower(0);
                motorB.setPower(0);
            }

            if (gamepad1.right_trigger>0.3) {
                arm.armIntake(INTAKE_SPEED);
            } else if (gamepad1.left_trigger>0.3) {
                arm.armIntake(-INTAKE_SPEED);
            } else {
                arm.armIntake(0);
            }

            if (gamepad1.a) {
                pos = 0;
                hook.setPosition(0);
            }
            if(gamepad1.x) {
                pos += .1;

            }

            if (gamepad1.b) {
                pos -= .1;
            }

            if(gamepad1.dpad_up) {
                hook_pos += .025;
            }
            if(gamepad1.dpad_down) {
                hook_pos -= .025;
            }
            hook.setPosition(hook_pos);

            arm.armSet(pos);

            telemetry.addData("Height Power", gamepad1.left_stick_x);
            telemetry.addData("Angle", sens.getPot());
            telemetry.addData("Expected Position", pos);
            telemetry.addData("Actual Position Front", motorF.getCurrentPosition());
            telemetry.addData("Actual position back", motorB.getCurrentPosition());

            telemetry.update();
        }

        // Turn off motor and signal done;
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
