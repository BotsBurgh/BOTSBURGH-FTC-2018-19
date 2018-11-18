package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        Movement movement = new Movement(motorFL, motorFR, motorBL, motorBR, gyro);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {
            // Add deadzones later 
            double x1 = gamepad1.left_stick_x;
            double y1 = gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double flPower = y1 + x1+rotation;
            double frPower = y1 - x1-rotation;
            double blPower = y1 - x1+rotation;
            double brPower = y1 + x1-rotation;

            movement.quadMove(flPower, frPower, blPower, brPower);
        }

    }

}
