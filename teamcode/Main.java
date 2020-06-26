package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Created by (for example John) on 12/13/2017.
 */

@TeleOp(name = "Main", group = "Main")
@Disabled
public class Main extends LinearOpMode {

    ColorSensor colorSensor;    // Hardware Device Object

    @Override
    public void runOpMode() {

        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Red ", colorSensor.red());
            if (colorSensor.blue()> colorSensor.red())
                telemetry.addData("Blue ",1);
            else telemetry.addData("Esti idiot ",1);
            telemetry.update();

        }

    }

}
