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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/*

,dodoooodoooodood;      |---------------------------------------------------|     ,ddddddddddddddddc
l.               d      |               .oK.                       k:     :l|     l.               x
l.               d      |               'ko                        x.     .l|     l.               x
l.               d      |              ,ko                         x.     .l|     l.               x
l.               d      |           .:kd'                  .       oc;;;;;c:|     l.               x
l.               d      |        .;xOo.  .                                  |     l.               x
l.  b            d      |      ;xkl.                          .             |     l.           r   x
l.  l            d      |   ;lko,.   .          .__,                        |     l.           e   x
l.  u            d      |xdlc'                '/,||'\;           .          |     l.           d   x
l.  e            d      |        .     BC   ,/'  ||  .\:    RF              |     l.               x
l.               d      |                 ;/.    ||    .\:                  |     l.               x
l.       k'''''',d      |               :/.    ./'.\'    .\c                |     l:''''''k        x
l.       x       d      |             c/.    ./.    .\.    .\l.             |     l.      d        x
l.       x       d      |             :\.    .\'    '/'    ./c              |     l.      d        x
l.       k......'d      |               ;\'    .\',/'    ./:                |     l:......k        x
l.              .d      |                 ,\,    ||    ./;                  |     l'               x
l.               d      |                   '\;  ||  '/,         .          |     l.               x
l.               d      |        .     BF     .\;||,/'      RC         .':l.|     l.               x
l.               d      |                       .--'         .      .:dkk   |     l.               x
l.               d      |           .                            .,lklc'    |     l.               x
l.               d      |                                 .   ..lxO:        |     l.               x
l.               d      |d;;;;;;k      .                    .ckx:           |     l.               x
l.               d      |c      0                          :kk,             |     l.               x
l.               d      |c      0                         c0d               |     l.               x
l.               d      |d      K                        :kO.               |     l.               x
;xxxxxxxxxxxxxxxx:      |---------------------------------------------------|     ;xxxxxxxxxxxxxxxxc

BC is Blue Close (Close to the blue team's crater)
BF is Blue Far   (Far from the blue team's crater)
RC is Red Close  (Close to the red team's crater)
RF is Red Far    (Far from the red team's crater)

*/


// Tests
// TODO: JavaDoc
@Autonomous(name="encoderTest")
public class FourWheelDrive extends LinearOpMode {
    /* Declare OpMode members. */
    DcMotor motorFL, motorFR, motorBL, motorBR;  // Use a Pushbot's hardware


    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    @Override
    public void runOpMode() {

        motorFL = hardwareMap.get(DcMotor.class, "fl");
        motorFR = hardwareMap.get(DcMotor.class, "fr");
        motorBL = hardwareMap.get(DcMotor.class, "bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);

        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.update();
        waitForStart();

        driveForward(15,.7);

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        telemetry.update();

    }

    public void driveForward(double inches, double speed) {
        int newPosition = (int) (inches * COUNTS_PER_INCH);
        motorFL.setTargetPosition(motorFL.getCurrentPosition()+newPosition);
        motorFR.setTargetPosition(motorFR.getCurrentPosition()+newPosition);
        motorBL.setTargetPosition(motorBL.getCurrentPosition()+newPosition);
        motorBR.setTargetPosition(motorBR.getCurrentPosition()+newPosition);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);

        while(motorBL.isBusy() || motorBR.isBusy() || motorBL.isBusy()  || motorBR.isBusy()) {

        }

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}