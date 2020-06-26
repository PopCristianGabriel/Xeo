package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by (for example John) on 12/21/2017.
 */

@Disabled
@Autonomous(name = "Servo Sensor", group = "Sensor")
public class Servo_Jewel extends LinearOpMode {

    Servo servo = hardwareMap.servo.get("servo_jewel");
    ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");

    @Override public void runOpMode() {

        double position = 1.0;

        servo.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {

            while (colorSensor.blue()==colorSensor.red())
            {

                servo.setPosition(position);
             position -= 0.01;

            }

            if(colorSensor.blue()>colorSensor.red())
            telemetry.addLine("Da in spate.");

            else if (colorSensor.red()<colorSensor.blue())
            telemetry.addLine("Da in fata.");

            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Red ", colorSensor.red());
            telemetry.addLine();
            telemetry.addLine(Double.toString(servo.getPosition()));
            telemetry.update();

        }

    }

}

