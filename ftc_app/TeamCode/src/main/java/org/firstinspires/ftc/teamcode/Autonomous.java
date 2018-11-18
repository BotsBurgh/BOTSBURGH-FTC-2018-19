package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


@Disabled
public class Autonomous extends LinearOpMode {
    public void runOpMode() {
        // Initialize stuff
        waitForStart();
        while (opModeIsActive()) {
            // Do the autonomous stuff

        }
    }
}
