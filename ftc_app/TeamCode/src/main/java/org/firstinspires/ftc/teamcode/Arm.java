/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
                arm.armBaseBack();
            } else if(gamepad1.right_bumper) {
                arm.armBaseForward();
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
                pos += .01;

            }

            if (gamepad1.b) {
                pos -= .01;
            }

            if(gamepad1.dpad_up) {
                hook_pos += .05;
            }
            if(gamepad1.dpad_down) {
                hook_pos -= .05;
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
