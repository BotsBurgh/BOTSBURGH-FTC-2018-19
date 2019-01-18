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
    private DcMotor motorF, motorB;
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

        double pos = .8;
        double hook_pos = 0;
        motorF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Movement movement = new Movement(motorFL, motorFR, motorBL, motorBR, gyro);
        Movement base = new Movement(motorF,motorB);
        Movement arm = new Movement(s1,s2,wheel);

        Sensors sens = new Sensors(pot);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = y1 - x1-rotation;
            double frPower = y1 + x1+rotation;
            double blPower = y1 + x1-rotation;
            double brPower = y1 - x1+rotation;

            movement.quadMove(flPower, frPower, blPower, brPower);
            telemetry.addData("Back Left", motorBL.getCurrentPosition());
            telemetry.addData("Back Right",motorBL.getCurrentPosition());
            telemetry.addData("Front Left",motorFL.getCurrentPosition());
            telemetry.addData("Front right", motorFR.getCurrentPosition());
            if(gamepad1.start) {
                while(Math.abs(sens.getPot()-90)<5) {
                    if(sens.getPot()>90) {
                        base.armBase(-.3);
                    }
                    if(sens.getPot()<90) {
                        base.armBase(.4);
                    }
                }
            }

            if(gamepad1.left_bumper) {
                if(sens.getPot()<20) {
                    base.armBase(0);
                } else if(sens.getPot()<50) {
                    base.armBase(-.1);
                } else if(sens.getPot()<90) {
                    base.armBase(-.3);
                } else {
                    base.armBase(-.5);
                }

            } else if(gamepad1.right_bumper) {
                if(sens.getPot()>115) {
                    base.armBase(0);
                } else if(sens.getPot() > 90) {
                    base.armBase(.1);
                } else if(sens.getPot() > 40) {
                    base.armBase(.4);
                    pos = .15;
                } else {
                    base.armBase(.5);
                }
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
                pos = .1;
                /*
                if(pos<1) {
                    pos += .1;
                }
                */
            }

            if (gamepad1.b) {
                pos = .8;
                /*
                if(pos > 0) {
                    pos -= .1;
                }
                */
            }
            if(gamepad1.y) {
                pos = .15;
            }

            if(gamepad1.dpad_up) {
                hook_pos += 0.1;
            }
            if(gamepad1.dpad_right) {
                hook_pos = .33;
            }
            if(gamepad1.dpad_down) {
                hook_pos = .66;
            }
            if(gamepad1.dpad_left) {
                hook_pos  -= .1;
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
    }
}
