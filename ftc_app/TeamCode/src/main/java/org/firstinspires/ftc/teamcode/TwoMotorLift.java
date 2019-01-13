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
 * This OpMode ramps a single motor speed up and down repeatedly until Stop is pressed.
 * The code is structured as a LinearOpMode
 *
 * This code assumes a DC motor configured with the name "left_drive" as is found on a pushbot.
 *
 * INCREMENT sets how much to increase/decrease the power each cycle
 * CYCLE_MS sets the update period.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "TWO MOTOR LIFT", group = "Concept")

public class TwoMotorLift extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    DcMotor motorF, motorB;
    Servo s1,s2,s3;
    CRServo wl,wr;


    double  power   = 0;
    boolean rampUp  = true;


    @Override
    public void runOpMode() {

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        wl = hardwareMap.get(CRServo.class, "wl");
        wr = hardwareMap.get(CRServo.class, "wr");

        Movement arm = new Movement(s1, s2, s3, wl, wr);
        // Connect to motor (Assume standard left wheel)
        // Change the text in quotes to match any motor name on your robot.
        motorF = hardwareMap.get(DcMotor.class, "elev_f");
        motorB = hardwareMap.get(DcMotor.class, "elev_r");


        AnalogInput pot = hardwareMap.analogInput.get("potent");
        Sensors sens = new Sensors(pot);


        wr.setDirection(CRServo.Direction.FORWARD);
        wl.setDirection(CRServo.Direction.FORWARD);
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.FORWARD);



        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        int pos = 0;
        motorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

/*
            if(gamepad1.left_bumper) {
                // Checks if its below threshold

                if(sens.getPot()< 23) {

                } else {
                    pos -= 20;
                }


            } else if(gamepad1.right_bumper)  {

                if(sens.getPot()> 120) {

                }
                else {
                    pos += 20;
                }


            }
*/


          //  motor.setTargetPosition(pos);
            if(gamepad1.left_bumper) {
                motorF.setPower(.7);
                //motorB.setPower(-1);
            }
            else if(gamepad1.right_bumper) {
                motorF.setPower(-.7);
                //motorB.setPower(1);
            }
            else {
                motorF.setPower(0);
               // motorB.setPower(0);
            }

            if(gamepad1.dpad_up) {
                motorB.setPower(-1);
            }
            else if(gamepad1.dpad_down) {
                motorB.setPower(1);
            }
            else {
                motorB.setPower(0);
            }



 /*
            if((sens.getPot() < 23 && motor.getCurrentPosition() > motor.getTargetPosition()) ||
                    (sens.getPot()>120 && motor.getCurrentPosition() < motor.getTargetPosition())) {
                motor.setTargetPosition(motor.getCurrentPosition());
            } else {
                motor.setPower(.5);
            }
            */


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
