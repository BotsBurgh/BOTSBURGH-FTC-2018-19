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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.internal.android.dex.util.Unsigned;

import java.util.Locale;

public class Movement {
    private double TURN_POWER  = 0.4;
    private double DRIVE_POWER = 0.6;
    private DcMotor motorFL, motorFR, motorBL, motorBR;
    private BNO055IMU gyro;
    private CRServo servoElevator0, servoElevator1, servoElevator2, servoElevator3, servoElevatorWheel;

    /**
     * Initialize the class
     * @param motorFL The front left motor
     * @param motorFR The front right motor
     * @param motorBL The back left motor
     * @param motorBR The back right motor
     * @param gyro The BNO055IMU gyroscope
     */
    Movement(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, BNO055IMU gyro) {
        this.motorFL   = motorFL;
        this.motorFR   = motorFR;
        this.motorBL   = motorBL;
        this.motorBR   = motorBR;
        this.gyro      = gyro;
    }

    /**
     * Initialize the class with the elevator functionality
     * @param motorFL The front left motor
     * @param motorFR The front right motor
     * @param motorBL The back left motor
     * @param motorBR The back right motor
     * @param gyro The BNO055IMU gyroscope
     * @param servoElevator0 The base servo motors
     * @param servoElevator1 First servos on the arm
     * @param servoElevator2 Second servos on the arm
     * @param servoElevator3 Third servos on the arm
     * @param servoElevatorWheel The intake wheels
     */
    Movement(DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, BNO055IMU gyro, 
    CRServo servoElevator0, CRServo servoElevator1, CRServo servoElevator2, CRServo servoElevator3, CRServo servoElevatorWheel) {
        this.motorFL   = motorFL;
        this.motorFR   = motorFR;
        this.motorBL   = motorBL;
        this.motorBR   = motorBR;
        this.gyro      = gyro;
        this.servoElevator0      = servoElevator0;
        this.servoElevator1      = servoElevator1;
        this.servoElevator2      = servoElevator2;
        this.servoElevator3      = servoElevator3;
        this.servoElevatorWheel  = servoElevatorWheel;
    }


    /**
     * Moves based on the encoder
     * @param inches How much to move forward or backward in inches
     */
    public void moveEnc(int inches) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setTargetPosition(inches);
        motorFR.setTargetPosition(inches);
        motorBL.setTargetPosition(inches);
        motorBR.setTargetPosition(inches);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            motorFL.setPower(DRIVE_POWER);
            motorFR.setPower(DRIVE_POWER);
            motorBL.setPower(DRIVE_POWER);
            motorBR.setPower(DRIVE_POWER);
        } else if (inches > 0) {
            motorFL.setPower(-DRIVE_POWER);
            motorFR.setPower(-DRIVE_POWER);
            motorBL.setPower(-DRIVE_POWER);
            motorBR.setPower(-DRIVE_POWER);
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }
    }

    /**
     * Turns the robot with the gyroscope
     * @param angles Turns the robot with an Orientation object
     */
    public void turn(Orientation angles) {
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Orientation current = gyro.getAngularOrientation();
        if (current.firstAngle > angles.firstAngle) {
            while (current.firstAngle != angles.firstAngle) {
                motorFL.setPower(TURN_POWER);
                motorFR.setPower(-TURN_POWER);
                motorBL.setPower(TURN_POWER);
                motorBR.setPower(-TURN_POWER);
            }
        } else if (current.firstAngle < angles.firstAngle) {
            while (current.firstAngle != angles.firstAngle) {
                motorFL.setPower(-TURN_POWER);
                motorFR.setPower(TURN_POWER);
                motorBL.setPower(-TURN_POWER);
                motorBR.setPower(TURN_POWER);
            }
        } else {
            motorFL.setPower(0);
            motorFR.setPower(0);
            motorBL.setPower(0);
            motorBR.setPower(0);
        }

    }

    /**
     * Basic movement with a left and right side to move. Best for tank drives or POV Drives
     * @param lpower The power to give to the left side
     * @param rpower The power to give to the right side
     */
    public void move(double lpower, double rpower) {
        motorFL.setPower(lpower);
        motorFR.setPower(rpower);
        motorBL.setPower(lpower);
        motorBR.setPower(rpower);
    }

    // TODO: Test this function

    /**
     * Moves servos based on speed
     * @param servo The servo to move
     * @param target The end location
     * @param speed How fast (percent). Must be positive
     */
    public void servoMove(Servo servo, double target, double speed) {
        double increment = 1.0-speed;
        double current=servo.getPosition();
        while (current!=target) {
            if (current<target) {
                current+=increment;
                servo.setPosition(current);
            } else if (current>target) {
                current-=increment;
                servo.setPosition(current);
            } else {
                break;
            }
        }
    }

    /**
     * Moves each of the four motors individually. Best for Mecanum drives.
     * @param flPower Power to the front left wheel
     * @param frPower Power to the front right wheel
     * @param blPower Power to the back left wheel
     * @param brPower Power to the back right wheel
     */
    public void quadMove(double flPower, double frPower, double blPower, double brPower) {
        motorFL.setPower(flPower);
        motorFR.setPower(frPower);
        motorBL.setPower(blPower);
        motorBR.setPower(brPower);
    }

    // TODO: Elevator + JavaDoc
    public void elevator(double speed) {
        // servoElevator.setPower(speed);
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}