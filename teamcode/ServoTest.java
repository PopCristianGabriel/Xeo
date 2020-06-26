package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp (name = "Servo Test", group = "Servo")
public class ServoTest extends OpMode {

    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    ColorSensor colorSensor;
    @Override
    public void init() {

        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo3 = hardwareMap.servo.get("servo3");
        servo4 = hardwareMap.servo.get("servo4");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");

    }

    @Override
    public void loop(){

        if(colorSensor.blue() > colorSensor.red()) {
            servo1.setPosition(1);
            servo2.setPosition(1);
            servo3.setPosition(1);
            servo4.setPosition(1);
        }
        else{
            servo1.setPosition(0);
            servo2.setPosition(0);
            servo3.setPosition(0);
            servo4.setPosition(0);
        }





    }
}
