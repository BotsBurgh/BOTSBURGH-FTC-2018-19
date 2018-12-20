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
    double pos=0;
    boolean scanD=true;

    @Override
    public void runOpMode() {
        // Init Servos
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        wl = hardwareMap.get(CRServo.class, "wl");
        wr = hardwareMap.get(CRServo.class, "wr");

        wr.setDirection(DcMotorSimple.Direction.FORWARD);
        wl.setDirection(DcMotorSimple.Direction.FORWARD);
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.FORWARD);

        Movement arm = new Movement(s1, s2, s3, wl, wr);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while(opModeIsActive()) {

            if (gamepad1.x) {
                wl.setPower(0.1);
                wr.setPower(-0.1);
            } else if (!gamepad1.x) {
                wl.setPower(0);
                wr.setPower(0);
            }

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
        }

    }

}
