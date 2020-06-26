/**
 * Created by Maddie, FTC Team 4962, The Rockettes
 * version 1.0 Aug 11, 2016
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*
	Holonomic concepts from:

	http://www.vexforum.com/index.php/12370-holonomic-drives-2-0-a-video-tutorial-by-cody/0

   Robot wheel mapping:

          X FRONT X
        X           X
      X  FL       FR  X
              X
             XXX
              X
      X  BL       BR  X
        X           X
          X       X
*/
@TeleOp(name = "Concept: HolonomicDrivetrain", group = "Concept")
@Disabled
public class teleopholonom extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorRidicare;
    Servo   servoRidS;
    Servo   servoRidD;
    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;
    private double servoPoz = 0;
    double motorPower = 1;

    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor
     */
    public teleopholonom() {

    }

    @Override
    public void init() {


		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */


        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        servoRidS = hardwareMap.servo.get("servo rid s");
        servoRidD = hardwareMap.servo.get("servo rid d");
        motorRidicare = hardwareMap.dcMotor.get("motor rid");
        //These work without reversing (Tetrix motors).
        //AndyMark motors may be opposite, in which case uncomment these lines:
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void loop() {

        // -----------------------------------------------------------------------------------> Miscare

        // left stick controls direction
        // right stick X controls rotation

        float gamepad1LeftY = -gamepad1.left_stick_y;
        float gamepad1LeftX = -gamepad1.left_stick_x;
        float gamepad1RightX = gamepad1.right_stick_x;

        // holonomic formulas

        float FrontLeft = -gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float FrontRight = gamepad1LeftY - gamepad1LeftX - gamepad1RightX;
        float BackRight = gamepad1LeftY + gamepad1LeftX - gamepad1RightX;
        float BackLeft = -gamepad1LeftY + gamepad1LeftX - gamepad1RightX;

        // clip the right/left values so that the values never exceed +/- 1
        FrontRight = Range.clip(FrontRight, -1, 1);
        FrontLeft = Range.clip(FrontLeft, -1, 1);
        BackLeft = Range.clip(BackLeft, -1, 1);
        BackRight = Range.clip(BackRight, -1, 1);

        // write the values to the motors
        motorFrontRight.setPower(FrontRight * motorPower);
        motorFrontLeft.setPower(FrontLeft * motorPower);
        motorBackLeft.setPower(BackLeft * motorPower);
        motorBackRight.setPower(BackRight * motorPower);


		/*
		 * Telemetry for debugging
		 */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Joy XL YL XR", String.format("%.2f", gamepad1LeftX) + " " +
                String.format("%.2f", gamepad1LeftY) + " " + String.format("%.2f", gamepad1RightX));
        telemetry.addData("f left pwr", "front left  pwr: " + String.format("%.2f", FrontLeft));
        telemetry.addData("f right pwr", "front right pwr: " + String.format("%.2f", FrontRight));
        telemetry.addData("b right pwr", "back right pwr: " + String.format("%.2f", BackRight));
        telemetry.addData("b left pwr", "back left pwr: " + String.format("%.2f", BackLeft));

        //schimbare putere motoare miscare
        if(gamepad1.y)
        {
            motorPower = 1;
        }
        if(gamepad1.a)
        {
            motorPower = 0.3;
        }
        if(gamepad1.x)
        {
            motorPower = 0.1;
        }

        // -----------------------------------------------------------------------------------> /Miscare

        // -----------------------------------------------------------------------------------> Prindere
        servoRidD.setPosition(servoPoz - 0.06);
        servoRidS.setPosition(1-servoPoz);



        if (gamepad2.left_trigger != 0 && servoPoz < 0.90) {
            servoPoz += gamepad2.left_trigger / 25;
        }
        if(gamepad2.right_trigger != 0 && servoPoz > 0.80){
            servoPoz -= gamepad2.right_trigger / 25;
        }
        if(gamepad2.right_trigger != 0 && servoPoz <= 0.80 && servoPoz > 0.40) {
            servoPoz -= gamepad2.right_trigger / 95;
        }

        // ----------------------------------------------------------------------------------->/Prindere


        // -----------------------------------------------------------------------------------> Ridicare
        if (gamepad2.dpad_up)
        {
            motorRidicare.setPower(1);
        }
        if (gamepad2.dpad_down)
        {
            motorRidicare.setPower(-1);
        }
        if(!gamepad2.dpad_down && !gamepad2.dpad_up) {
            motorRidicare.setPower(0);
        }
        telemetry.addData(" Pozitie cod: ", servoPoz);
        telemetry.addData("Real Dreapta: ", servoRidD.getPosition());
        telemetry.addData("Real Stanga: ", servoRidS.getPosition());
        // ----------------------------------------------------------------------------------->/Ridicare
    }

    @Override
    public void stop() {

    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput (double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    String formatRate(float rate) {
        return String.format("%.3f", rate);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}