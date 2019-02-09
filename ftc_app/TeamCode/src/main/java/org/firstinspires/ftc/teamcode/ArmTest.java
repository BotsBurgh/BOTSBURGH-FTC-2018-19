package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Arm Test", group = "Test")
public class ArmTest extends LinearOpMode {
    DcMotor extend, arm;
    double power = 0;
    @Override
    public void runOpMode() {
        extend = hardwareMap.get(DcMotor.class,"extend");
        arm = hardwareMap.get(DcMotor.class,"arm");


    }
}
