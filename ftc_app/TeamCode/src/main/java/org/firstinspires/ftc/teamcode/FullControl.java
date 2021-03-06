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

// Integrates the Arm and MecanumDrive Classes

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Full Control")
public class FullControl extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private BNO055IMU gyro;
    private DcMotor motorF, motorB, collector;
    private Servo s1,s2, hook;
    private CRServo wheel;
    private double INTAKE_SPEED=0.5;

    @Override
    public void runOpMode() {
        // Driving Part
        motorFL = hardwareMap.get(DcMotor.class,"fl");
        motorFR = hardwareMap.get(DcMotor.class,"fr");
        motorBL = hardwareMap.get(DcMotor.class,"bl");
        motorBR = hardwareMap.get(DcMotor.class,"br");
        collector = hardwareMap.get(DcMotor.class, "collector");

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");


        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        // Arm part

        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        hook = hardwareMap.get(Servo.class,"hook");

        wheel = hardwareMap.get(CRServo.class, "wheel");

        motorF = hardwareMap.get(DcMotor.class, "front");
        motorB = hardwareMap.get(DcMotor.class, "back");

        AnalogInput pot = hardwareMap.analogInput.get("potent");

        wheel.setDirection(CRServo.Direction.FORWARD);
        s1.setDirection(Servo.Direction.REVERSE);
        s2.setDirection(Servo.Direction.FORWARD);

        double pos = .7;
        double hook_pos = 0;
        motorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Movement movement = new Movement(motorFL, motorFR, motorBL, motorBR, gyro);
        Movement base = new Movement(motorF,motorB);
        Movement arm = new Movement(s1,s2,wheel);

        Sensor sens = new Sensor(pot);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        motorF.setTargetPosition(motorF.getCurrentPosition());
        motorB.setTargetPosition(motorB.getCurrentPosition());

        while(opModeIsActive()) {
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = y1 - x1-rotation;
            double frPower = y1 + x1+rotation;
            double blPower = y1 + x1-rotation;
            double brPower = y1 - x1+rotation;

            movement.quadMove(flPower, frPower, blPower, brPower);
            //telemetry.addData("Back Left", motorBL.getCurrentPosition());
           // telemetry.addData("Back Right",motorBL.getCurrentPosition());
            //telemetry.addData("Front Left",motorFL.getCurrentPosition());
            //telemetry.addData("Front right", motorFR.getCurrentPosition());


            if(gamepad1.left_bumper) {
                motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(sens.getPot()<50) {
                    //base.armBase(0);
                } else if(sens.getPot()<60) {
                    //base.armBase(-.3);
                } else if(sens.getPot()<70) {
                    //base.armBase(-.7);
                } else {
                    //base.armBase(-1);
                }

                motorF.setTargetPosition(motorF.getCurrentPosition());
                motorB.setTargetPosition(motorB.getCurrentPosition());


            } else if(gamepad1.right_bumper) {
                motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if(sens.getPot()>90) {
                    //base.armBase(0);
                } else if(sens.getPot() > 60) {
                    //base.armBase(.5);
                    pos = .3;
                } else if(sens.getPot() > 30) {
                    //base.armBase(.7);
                    pos = .3;
                } else {
                    //base.armBase(.9);
                }


                motorF.setTargetPosition(motorF.getCurrentPosition());
                motorB.setTargetPosition(motorB.getCurrentPosition());

            } else if(gamepad1.back) {
                motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //base.armBase(-1);
                motorF.setTargetPosition(motorF.getCurrentPosition());
                motorB.setTargetPosition(motorB.getCurrentPosition());
            } else {
                motorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorF.setPower(.2);
                motorB.setPower(.2);
            }

            if (gamepad1.right_trigger>0.3) {
                arm.armIntake(INTAKE_SPEED);
                collector.setPower(.5);
            } else if (gamepad1.left_trigger>0.3) {
                arm.armIntake(-INTAKE_SPEED);
                collector.setPower(-.5);
            } else {
                arm.armIntake(0);
                collector.setPower(0);
            }

            if (gamepad1.a) {
                if(hook_pos > 0) {
                    hook_pos -= 0.1;
                }
            }
            if(gamepad1.x) {
                if(pos<1) {
                    pos += .01;
                }
                /*
                if(pos<1) {
                    pos += .1;
                }
                */
            }

            if (gamepad1.b) {
                if(pos > 0) {
                    pos -= .01;
                }
                /*
                if(pos > 0) {
                    pos -= .1;
                }
                */
            }
            if(gamepad1.y) {
                if(hook_pos < 1) {
                    hook_pos += 0.01;
                }
            }

            if(gamepad1.dpad_up) {
                if(hook_pos < 1) {
                    hook_pos += 0.1;
                }
            }
            if(gamepad1.dpad_right) {
                if(pos < 1) {
                    pos += .05;
                }
            }
            if(gamepad1.dpad_down) {
                if(hook_pos > 0) {
                    hook_pos -= .1;
                }
            }
            if(gamepad1.dpad_left) {
                if(pos > 0) {
                    pos -= .05;
                }
            }

            hook.setPosition(hook_pos);

            if(sens.getPot()>80) {
                pos = .35;
            }
            //arm.armSet(pos);

            telemetry.addData("Angle", sens.getPot());
            telemetry.addData("Expected Arm Position", pos);
            telemetry.addData("Expected hook position", hook_pos);
            telemetry.addData("Actual Position Front", motorF.getCurrentPosition());
            telemetry.addData("Actual position back", motorB.getCurrentPosition());

            telemetry.update();


        }
    }
}
