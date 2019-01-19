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
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class Sensors {
    public static int POT_MAX = 270; // Max range in degrees

    public static double Vmin = 0.004; // Minimum voltage

    public static double Vmax = 3.304; // Maximum voltage

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    // TODO: Initialize more sensors
    BNO055IMU gyro; // Initializes gyroscope
    AnalogInput pot; // Initializes potentiometer

    private static final String VUFORIA_KEY = "AcM0K6z/////AAABmeiIHPqExEm6uvdttqzvUM8yc5vG8YPI75H9AWdWhYDwS3uA8rxBOa8gofNaaTRkLfYpu0EcoykMACJ9vm2u9D0uBFlsxkOSGnjSGZOH7jjS2A+rm0WyOyZ7krIdfoNm+2yV+nPqoQwFApuUDVN7d/HDXq+iW1P+21ZG1ahvPeDr4zJqoHLf9AvNaUzDWssKFBshs6MXdHPH7TaNAHebpqOwVvwOriBRaM/2ffxi/676+DEGypvu5pRcTwmzkCiP3BEdFVpG8BH1jUEcZ+GQd0s59hhqKV2tJZIQwQgvzZISTGSLZHZ06Ag5tOA+m9zIW5M8UpkdWrFEO7mGBRZnMmW0Ztle8Lg+lEHd6t5lZwuS";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;


    // Put all sensor stuff in here.
    // TODO: Constructor + JavaDoc
    Sensors(BNO055IMU gyro, AnalogInput pot) { // Constructor (to use in another file)
        this.gyro = gyro;
        this.pot = pot;
    }
    Sensors (AnalogInput pot) { // Another constructor
        this.pot = pot;
    }
    public double getPot() {
        return (POT_MAX/(Vmax-Vmin))*(pot.getVoltage()-Vmin); // Converts voltage to angle (degrees)
    }

    public String track(VuforiaLocalizer vuforia, TFObjectDetector tfod) {
        return null;
    }
}

