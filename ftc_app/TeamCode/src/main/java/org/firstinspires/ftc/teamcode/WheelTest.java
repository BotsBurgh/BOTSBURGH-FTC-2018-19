package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Intake Test", group = "Linear OpMode")
public class WheelTest extends LinearOpMode {
    private double INTAKE_SPEED=0.5;
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo wheel;
    private BNO055IMU gyro;

    @Override
    public void runOpMode() {
        // Init Servo
        wheel = hardwareMap.get(CRServo.class, "wheel");
        Movement wheelMovement = new Movement(wheel);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            if (gamepad1.right_trigger>0.3) {
                wheelMovement.armIntake(INTAKE_SPEED);
            } else if (gamepad1.right_bumper) {
                wheelMovement.armIntake(-INTAKE_SPEED);
            } else {
                wheelMovement.armIntake(0);
            }
        }

    }

}
