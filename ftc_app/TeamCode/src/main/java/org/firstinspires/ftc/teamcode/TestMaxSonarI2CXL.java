package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TestMaxSonarI2CXL", group="Test Sensors")  // @Autonomous(...) is the other common choice

public class TestMaxSonarI2CXL extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    MaxSonarI2CXL _sonarLeft;
    MaxSonarI2CXL _sonarRight;
    private SonarArrayManager _sonarManager;

    @Override
    public void runOpMode() {
        _sonarLeft = hardwareMap.get(MaxSonarI2CXL.class, "sonar_left");
    //    _sonarRight = hardwareMap.get(MaxSonarI2CXL.class, "sonar_right");

        _sonarLeft.setI2cAddress(I2cAddr.create8bit(0xE0));
     //   _sonarRight.setI2cAddress(I2cAddr.create8bit(0xDE));

        _sonarManager = new SonarArrayManager();
        _sonarManager.addSonar("left", _sonarLeft);
     //   _sonarManager.addSonar("right", _sonarRight);

//        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow (1, 2, I2cDeviceSynch.ReadMode.REPEAT);
//        _sonar.setReadWindow(readWindow);

        telemetry.addData("Status", "7");
        telemetry.addData("_sonarLeft.getConnectionInfo()", _sonarLeft.getConnectionInfo());
       // telemetry.addData("_sonarRight.getConnectionInfo()", _sonarRight.getConnectionInfo());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        _sonarManager.startAutoPing(100);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            double distanceLeft = _sonarManager.getDistance("left");
        //    double distanceRight = _sonarManager.getDistance("right");

            telemetry.addData("Status", "Distance left: " + distanceLeft);
        //    telemetry.addData("Status", "Distance right: " + distanceRight);
            telemetry.update();
        }

        _sonarManager.stopAutoPing();
    }
}