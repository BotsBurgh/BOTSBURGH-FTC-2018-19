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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
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
@Autonomous(name="Gyro Test")
public class Auto extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    /* Declare OpMode members. */
    DcMotor motorFL,motorFR,motorBL,motorBR;
    DcMotor elevator;
    BNO055IMU gyro;// Additional Gyro device
    Orientation angles;
    Acceleration gravity;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    ElapsedTime runtime  = new ElapsedTime();
    Servo holder;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final String VUFORIA_KEY = "AcM0K6z/////AAABmeiIHPqExEm6uvdttqzvUM8yc5vG8YPI75H9AWdWhYDwS3uA8rxBOa8gofNaaTRkLfYpu0EcoykMACJ9vm2u9D0uBFlsxkOSGnjSGZOH7jjS2A+rm0WyOyZ7krIdfoNm+2yV+nPqoQwFApuUDVN7d/HDXq+iW1P+21ZG1ahvPeDr4zJqoHLf9AvNaUzDWssKFBshs6MXdHPH7TaNAHebpqOwVvwOriBRaM/2ffxi/676+DEGypvu5pRcTwmzkCiP3BEdFVpG8BH1jUEcZ+GQd0s59hhqKV2tJZIQwQgvzZISTGSLZHZ06Ag5tOA+m9zIW5M8UpkdWrFEO7mGBRZnMmW0Ztle8Lg+lEHd6t5lZwuS";



    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.7;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable


    @Override
    public void runOpMode() {

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "gyro".
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        gyro.initialize(parameters);

        sensorColor = hardwareMap.get(ColorSensor.class,"sensor_color_distance");
        sensorDistance = hardwareMap.get(DistanceSensor.class,"sensor_color_distance");

        motorFL = hardwareMap.get(DcMotor.class,"fl");
        motorFR = hardwareMap.get(DcMotor.class,"fr");
        motorBL = hardwareMap.get(DcMotor.class,"bl");
        motorBR = hardwareMap.get(DcMotor.class,"br");

        elevator = hardwareMap.get(DcMotor.class,"elevator");

        holder = hardwareMap.get(Servo.class, "holder");

        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator.setDirection(DcMotorSimple.Direction.REVERSE);


        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gyro.initialize(parameters);

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */



        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();

        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        composeTelemetry();
        telemetry.update();
        runtime.reset();
        holder.setPosition(0);
        waitForStart();

        gyro.startAccelerationIntegration(new Position(), new Velocity(), 250);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }
        }
        runtime.reset();
        int position=0;

        runtime.reset();
        while(runtime.seconds() < .5) {
            elevator.setPower(1);
        }
        runtime.reset();
        while((sensorColor.red()<sensorColor.blue() || sensorColor.red()< sensorColor.green())) {

            elevator.setPower(1);
        }
        elevator.setPower(0);
        runtime.reset();

        while (runtime.seconds()<1.5) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }
                        if(goldMineralX == -1) {
                            telemetry.addData("Gold Mineral Position", "Right");
                            position = 1;
                        } else if((goldMineralX < silverMineral1X) || (goldMineralX < silverMineral2X)) {
                            telemetry.addData("Gold Mineral Position", "Left");
                            position = 2;
                        } else {
                            telemetry.addData("Gold Mineral Position","Center");
                            position = 0;
                        }

                    }

                    if(updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if ((goldMineralX < silverMineral1X) && goldMineralX < silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Left");
                                position = 2;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                telemetry.addData("Gold Mineral Position", "Right");
                                position = 1;
                            } else {
                                telemetry.addData("Gold Mineral Position", "Center");
                                position = 0;
                            }
                        }
                    }
                    telemetry.update();
                }
            }
        }
        gyroStrafe(DRIVE_SPEED,3,0,.5);
        gyroTurn(TURN_SPEED,-45,1);
        gyroDrive(DRIVE_SPEED,4,-45,.5);


        double angle;
        if(position == 1) {
            angle = -45;
            gyroTurn(TURN_SPEED,angle,1);
            gyroDrive(DRIVE_SPEED,30,angle,2);
            gyroDrive(DRIVE_SPEED,-10,angle,1);
        } else if(position == 2) {
            angle = 25;
            gyroTurn(TURN_SPEED,angle,1);
            gyroDrive(DRIVE_SPEED,25,angle,2);
            gyroDrive(DRIVE_SPEED,-10,angle,1);
        } else {
            angle = 2;
            gyroTurn(TURN_SPEED,angle,1);
            gyroDrive(DRIVE_SPEED,25,angle,2);
            gyroDrive(DRIVE_SPEED,-7,angle,1);
        }
        gyroTurn(TURN_SPEED,-90,1);
        if(position == 1) {
            gyroDrive(DRIVE_SPEED,-50,90,3.5);
        } else if(position == 2) {
            gyroDrive(DRIVE_SPEED,-35,90,2);
        } else {
            gyroDrive(DRIVE_SPEED,-60,90,3);
        }

        gyroTurn(TURN_SPEED,-45,2);
        if(position == 1) {
            gyroStrafe(DRIVE_SPEED,10,-135,1);
            gyroDrive(DRIVE_SPEED, -20, -135, 3);
             holder.setPosition(90);
            gyroDrive(DRIVE_SPEED, 45, -135, 2.7);
            gyroDrive(DRIVE_SPEED, 45, -135, 2.7);
        } else if(position == 0) {
            gyroStrafe(DRIVE_SPEED,10,-135,1);
            gyroDrive(DRIVE_SPEED,-20,-135,3);
            holder.setPosition(90);
            gyroDrive(DRIVE_SPEED,45,-135,2.7);
            gyroDrive(DRIVE_SPEED,45,-135,2.7);
        } else {
            gyroStrafe(DRIVE_SPEED,10,-135,1);
            gyroDrive(DRIVE_SPEED, -20, -135, 3);
            holder.setPosition(90);
            gyroDrive(DRIVE_SPEED, 45, -135, 2.7);
            gyroDrive(DRIVE_SPEED, 45, -135, 2.7);
        }



        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // Put a hold after each turn

        telemetry.update();
        runtime.reset();

        /*
        while(runtime.seconds() < .5) {
            elevator.setPower(1);
        } while(sensorColor.red()<sensorColor.blue() || sensorColor.red()< sensorColor.green()) {
            elevator.setPower(1);
        }*/

        elevator.setPower(0);

      //  gyroStrafe(DRIVE_SPEED,2,1);

        /*
        holder.setPosition(1);
        gyroTurn(TURN_SPEED,30);



        //gyroDrive(DRIVE_SPEED, 20.0, 0.0);
      //  gyroStrafe(DRIVE_SPEED,-2,1);

      //  gyroDrive(DRIVE_SPEED, -8,0.0);
        gyroTurn(TURN_SPEED,90.0);
       // gyroDrive(DRIVE_SPEED,45.0,90);
        gyroTurn(TURN_SPEED,135);
        telemetry.update();
       // gyroDrive(DRIVE_SPEED,45,135);

        holder.setPosition(.5);
        sleep(100);
        holder.setPosition(0);
        sleep(100);
        holder.setPosition(1);
        sleep(100);

       // gyroDrive(DRIVE_SPEED, -72, 135);

        runtime.reset(); */

        /*
        while(runtime.seconds()<.5) {
            elevator.setPower(-1);
        } while(sensorColor.red()<sensorColor.blue() || sensorColor.red() < sensorColor.green()) {
            elevator.setPower(-1);
        }*/





        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    void composeTelemetry() {


        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = gyro.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return gyro.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return gyro.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }


    /**
     * Method to drive on a fixed compass bearing (angle), based on encoder counts.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the desired position
     * 2) Driver stops the opmode running.
     *
     * @param speed    Target speed for forward motion.  Should allow for +/- variance for adjusting heading
     * @param distance Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     */
    public void gyroStrafe(double speed, double distance, double angle,double time) {
        /*
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();
        while(runtime.seconds() < distance) {
            motorFL.setPower(-speed);
            motorFR.setPower(speed);
            motorBL.setPower(speed);
            motorBR.setPower(-speed);
        }
        */

        int moveCounts,newBackLeftTarget,newBackRightTarget,newFrontLeftTarget,newFrontRightTarget;

        double leftSpeed,rightSpeed, max;

        double error;
        double steer;

        moveCounts = (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = motorBL.getCurrentPosition()+moveCounts;
        newBackRightTarget = motorBR.getCurrentPosition()-moveCounts;
        newFrontLeftTarget = motorFL.getCurrentPosition()-moveCounts;
        newFrontRightTarget = motorFR.getCurrentPosition()+moveCounts;
        motorBL.setTargetPosition(newBackLeftTarget);
        motorBR.setTargetPosition(newBackRightTarget);
        motorFL.setTargetPosition(newFrontLeftTarget);
        motorFR.setTargetPosition(newFrontRightTarget);

        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBL.setPower(speed);
        motorBR.setPower(speed);
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        runtime.reset();
        while(opModeIsActive()&& (motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy()) && runtime.seconds()<time) {

        }
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void gyroDrive(double speed,
                          double distance,
                          double angle, double time) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;
        int moveCounts;
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            telemetry.update();

            // Determine new target position, and pass to motor controller
            moveCounts = (int) (distance * COUNTS_PER_INCH);
            newBackLeftTarget = motorBL.getCurrentPosition() + moveCounts;
            newBackRightTarget = motorBR.getCurrentPosition() + moveCounts;
            newFrontLeftTarget = motorFL.getCurrentPosition() + moveCounts;
            newFrontRightTarget = motorFR.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            motorBL.setTargetPosition(newBackLeftTarget);
            motorFL.setTargetPosition(newFrontLeftTarget);
            motorBR.setTargetPosition(newBackRightTarget);
            motorFR.setTargetPosition(newFrontRightTarget);

            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            motorBL.setPower(speed);
            motorBR.setPower(speed);
            motorFL.setPower(speed);
            motorFR.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            runtime.reset();
            while (opModeIsActive() &&
                    (motorBL.isBusy() || motorBR.isBusy() ||  motorFL.isBusy() || motorFR.isBusy())&& runtime.seconds()< time) {

                // adjust relative speed based on heading error.
                /*
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                motorFL.setPower(leftSpeed);
                motorBL.setPower(leftSpeed);
                motorFR.setPower(rightSpeed);
                motorBR.setPower(rightSpeed);
                */


                // Display drive status for the driver.

                telemetry.addData("Target", "%7d:%7d", newBackLeftTarget,newBackRightTarget);
                telemetry.addData("Actual", "%7d:%7d", motorBL.getCurrentPosition(),
                        motorBR.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", speed, speed);
                telemetry.update();
            }

            // Stop all motion;
            motorBL.setPower(0);
            motorBR.setPower(0);
            motorFL.setPower(0);
            motorFR.setPower(0);

            // Turn off RUN_TO_POSITION
            motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Method to spin on central axis to point in a new direction.
     * Move will stop if either of these conditions occur:
     * 1) Move gets to the heading (angle)
     * 2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle Absolute Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void gyroTurn(double speed, double angle,double time) {
        telemetry.update();
        // keep looping while we are still active, and not on heading.
        runtime.reset();
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF) && runtime.seconds()<time) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }

    /**
     * Method to obtain & hold a heading for a finite amount of time
     * Move will stop once the requested time has elapsed
     *
     * @param speed    Desired speed of turn.
     * @param angle    Absolute Angle (in Degrees) relative to last gyro reset.
     *                 0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                 If a relative angle is required, add/subtract from current heading.
     * @param holdTime Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold(double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed  Desired speed of turn.
     * @param angle  Absolute Angle (in Degrees) relative to last gyro reset.
     *               0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *               If a relative angle is required, add/subtract from current heading.
     * @param PCoeff Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
        }

        // Send desired speeds to motors.
        motorFL.setPower(leftSpeed);
        motorFR.setPower(rightSpeed);
        motorBL.setPower(leftSpeed);
        motorBR.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        telemetry.update();
        robotError = targetAngle - Double.parseDouble(formatAngle(angles.angleUnit,angles.firstAngle));
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     *
     * @param error  Error angle in robot relative degrees
     * @param PCoeff Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(-error * PCoeff, -1, 1);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = .7;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
}