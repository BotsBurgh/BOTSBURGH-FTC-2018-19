package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Sensors {
    private static int POT_MAX = 270; // Max range in degrees

    private static double Vmin = 0.004; // Minimum voltage

    private static double Vmax = 3.304; // Maximum voltage
    // TODO: Initialize sensors
    BNO055IMU gyro;
    AnalogInput pot;
    // Put all sensor stuff in here.
    // TODO: Constructor + JavaDoc
    Sensors(BNO055IMU gyro, AnalogInput pot) {
        this.gyro = gyro;
        this.pot = pot;
    }
    Sensors (AnalogInput pot) {
        this.pot = pot;
    }
    public double getPot() {
        return (POT_MAX/(Vmax-Vmin))*(pot.getVoltage()-Vmin);
    }
}

