package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo: Digital Servo", group = "Servo")
public class DigiServoTest extends LinearOpMode {
    Servo digiServo2;
    public void runOpMode() {
        digiServo2 = hardwareMap.get(Servo.class, "digiServo");
        double pos=0;
        Movement digiServo = new Movement(digiServo2);
        waitForStart();
        while (opModeIsActive()) {
            // Do stuff
            if (gamepad1.dpad_up) {
                pos+=0.01;
            } else if (gamepad1.dpad_down) {
                pos-=0.01;
            } else {
                // Do nothing
            }
            digiServo.digiMove(pos);
        }
    }
}
