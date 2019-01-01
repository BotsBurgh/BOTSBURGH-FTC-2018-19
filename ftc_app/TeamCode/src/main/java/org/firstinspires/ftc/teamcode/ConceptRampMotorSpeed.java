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

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@TeleOp(name = "Concept: Ramp Motor Speed", group = "Concept")

public class ConceptRampMotorSpeed extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_FWD     =  1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV     = -1.0;     // Maximum REV power applied to motor

    // Define class members
    DcMotor motor;
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
        motor = hardwareMap.get(DcMotor.class, "elev");

        AnalogInput pot = hardwareMap.analogInput.get("potent");
        Sensors sens = new Sensors(pot);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        wr.setDirection(CRServo.Direction.FORWARD);
        wl.setDirection(CRServo.Direction.FORWARD);
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.FORWARD);



        // Wait for the start button
        telemetry.addData(">", "Press Start to run Motors." );
        telemetry.update();
        int pos = 0;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        // Ramp motor speeds till stop pressed.
        while(opModeIsActive()) {

            if(gamepad1.left_bumper) {
                pos -= 20;

            } else if(gamepad1.right_bumper)  {
                pos += 20;

            }


            motor.setTargetPosition(pos);
            motor.setPower(.3);

            telemetry.addData("Height Power", gamepad1.left_stick_x);
            telemetry.addData("Angle", sens.getPot());
            telemetry.addData("Expected Position", pos);
            telemetry.addData("Postition",motor.getCurrentPosition());
            telemetry.update();
        }

        // Turn off motor and signal done;
        motor.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
