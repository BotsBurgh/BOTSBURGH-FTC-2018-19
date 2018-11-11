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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

public class Movement {
    private double TURN_POWER  = 0.4;
    private double DRIVE_POWER = 0.6;
    private DcMotor lf, rf, lb, rb;
    private BNO055IMU gyro;
    // TODO: Add JavaDoc for constructor
    Movement(DcMotor lf, DcMotor rf, DcMotor lb, DcMotor rb, BNO055IMU gyro) {
        this.lf   = lf;
        this.rf   = rf;
        this.lb   = lb;
        this.rb   = rb;
        this.gyro = gyro;
    }
    // TODO: Add JavaDoc for moveEnc
    public void moveEnc(int inches) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setTargetPosition(inches);
        rf.setTargetPosition(inches);
        lb.setTargetPosition(inches);
        rb.setTargetPosition(inches);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches < 0) {
            lf.setPower(DRIVE_POWER);
            rf.setPower(DRIVE_POWER);
            lb.setPower(DRIVE_POWER);
            rb.setPower(DRIVE_POWER);
        } else if (inches > 0) {
            lf.setPower(-DRIVE_POWER);
            rf.setPower(-DRIVE_POWER);
            lb.setPower(-DRIVE_POWER);
            rb.setPower(-DRIVE_POWER);
        } else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }
    }
    // TODO: Add JavaDoc for turn
    public void turn(Orientation angles) {
        gyro.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        Orientation current = gyro.getAngularOrientation();
        if (current.firstAngle > angles.firstAngle) {
            while (current.firstAngle != angles.firstAngle) {
                lf.setPower(TURN_POWER);
                rf.setPower(-TURN_POWER);
                lb.setPower(TURN_POWER);
                rb.setPower(-TURN_POWER);
            }
        } else if (current.firstAngle < angles.firstAngle) {
            while (current.firstAngle != angles.firstAngle) {
                lf.setPower(-TURN_POWER);
                rf.setPower(TURN_POWER);
                lb.setPower(-TURN_POWER);
                rb.setPower(TURN_POWER);
            }
        } else {
            lf.setPower(0);
            rf.setPower(0);
            lb.setPower(0);
            rb.setPower(0);
        }

    }
    // TODO: Add JavaDoc for move
    public void move(double lpower, double rpower) {
        lf.setPower(lpower);
        rf.setPower(rpower);
        lb.setPower(lpower);
        rb.setPower(rpower);
    }
    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}