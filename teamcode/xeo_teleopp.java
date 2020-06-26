/** Team RO001 Xeo TELE-OP code 2017-2018 Relic Recovery*/

package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;


@TeleOp(name = "Xeo17-18: Tele-OP", group = "Xeo17-18")
//@Disabled
public class xeo_teleopp extends OpMode {

    // declarare variabile
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor motorRidicare;
    DcMotor motorRelic;
    Servo   servoRidS_0;
    Servo   servoRidS_1;
    Servo   servoRidD_0;
    Servo   servoRidD_1;

    Servo   relicPrindere;
    Servo   relicRidicare;
    //IntegratingGyroscope gyro;
    // NavxMicroNavigationSensor navxMicro;
    Servo   servoBile;

    private double servoPoz = 0.68;
    private double servoPozS = 0.32;
    private double servoPozD = 0.62;
    private double motorPower = 0.3;
    private double relicMotorPower = 1;
    double standard  = 0.3;
    double timp;
    private int profil;
    private double relicPoz = 1;
    private double ridicarePoz = 1;
    private final int kMaxNumberOfMotors = 4;
    private final double maxOutput = 1.0;
    double alpha = 0, beta = 0;
    double xDir = 0, yDir = 0;
    boolean apasat = false;
    double pozDeschidere = 0.6792;
    double pozInchidere = 0.5926;
    double pozitieRelicRidicare = 0;
    double pozitieRelicPrindere = 0;
    double relicSus = 0.8;
    double relicJos = 0.10;
    double relicPrins = 0.95;
    double relicDeschis = 0;
    double timp1 = 0,timp2 = 0;
    double pozIncheietura = 0;
    boolean inAer = false;
    static final int LED_CHANNEL = 5;
    AdafruitI2cColorSensor colorSensor;
    // public ModernRoboticsI2cRangeSensor rangeSensor;
    // A timer helps provide feedback while calibration is taking place
    ElapsedTime timer = new ElapsedTime();


    /**
     * Constructor
     */


    @Override
    public void init() {        //functie init, se apeleaza la apasarea butonului init de pe driver station

		/*
		 * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */
        colorSensor = (AdafruitI2cColorSensor) hardwareMap.get("color_sensor");
        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        motorRelic = hardwareMap.dcMotor.get("motor relic");
        //These work without reversing (Tetrix motors).
        //AndyMark motors may be opposite, in which case uncomment these lines:
        //motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        //motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        servoRidS_0 = hardwareMap.servo.get("servo rid s 0");
        servoRidD_0 = hardwareMap.servo.get("servo rid d 0");
        servoRidS_1 = hardwareMap.servo.get("servo rid s 1");
        servoRidD_1 = hardwareMap.servo.get("servo rid d 1");
        motorRidicare = hardwareMap.dcMotor.get("motor rid");
        /*servoRidD_0.setPosition(servoPozD);
        servoRidS_0.setPosition(servoPozS);
        servoRidD_1.setPosition(servoPozD);
        servoRidS_1.setPosition(servoPozS);*/
        relicPrindere=hardwareMap.servo.get("servo relic prindere");
        relicRidicare=hardwareMap.servo.get("servo relic ridicare");
        // relicPrindere.setPosition(0);
        // relicRidicare.setPosition(0);
        //  navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        // gyro = (IntegratingGyroscope)navxMicro;
        profil=1;
        servoBile = hardwareMap.servo.get("servo bile");
        servoBile.setPosition(0.1);
        //  rangeSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.opticalDistanceSensor.get("sensor_distance");
        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        //Wait until the gyro calibration is complete
        //  timer.reset();
        //  while (navxMicro.isCalibrating())  {
        //      telemetry.addData("calibrating", "%s", Math.round(timer.seconds())%2==0 ? "|.." : "..|");
        //      telemetry.update();
        //   }
        //  telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        telemetry.clear(); telemetry.update();
        hardwareMap.deviceInterfaceModule.get("dim").setDigitalChannelState(LED_CHANNEL, false);

    }



    @Override
    public void loop() {        //functie loop, se repeta continuu, de la apasarea start pana la apasarea stop

        servoBile.setPosition(0.1);
        //telemetry.addData("distanta", rangeSensor.getDistance(DistanceUnit.CM));
        // telemetry.update();
        /** ----------------------------- <miscare> ----------------------------- */
        /* left stick controls direction
           right stick X controls rotation */

        // Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        //   float unghi = angles.firstAngle;
        //  float var_1 = (float)Math.cos(unghi);
        //  float var_2 = (float)Math.sin(unghi);
        float gamepad1LeftY = -gamepad1.left_stick_y ;
        float gamepad1LeftX = -gamepad1.left_stick_x ;
        float gamepad1RightX = gamepad1.right_stick_x;

            /*--> if(gamepad1RightX < -0.1 || gamepad1RightX > 0.1){
                apasat = true;
            }
            if(gamepad1RightX >= -0.1 && gamepad1RightX <= 0.1 && apasat){
                apasat = false;
                alpha = unghi;
            }

            if(Math.abs(gamepad1LeftX) > 0.1 || Math.abs(gamepad1LeftY) > 0.1) {
                if (gamepad1LeftX != 0) {
                    beta = gamepad1LeftY / gamepad1LeftX;
                    xDir = (Math.cos(alpha) + Math.cos(beta)) / 2;
                    yDir = (Math.sin(alpha) + Math.sin(beta)) / 2;
                } else {
                    xDir = Math.cos(alpha);
                    yDir = Math.sin(alpha);
                }
            } else{
                xDir = 0;
                yDir = 0;
            } */

            /*double FrontLeft = -yDir - xDir - gamepad1RightX;
            double FrontRight = yDir - xDir - gamepad1RightX;
            double BackRight = yDir + xDir - gamepad1RightX;
            double BackLeft = -yDir + xDir - gamepad1RightX; */

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
        if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_right && !gamepad1.dpad_left)
        {
            motorFrontRight.setPower(FrontRight * motorPower);
            motorFrontLeft.setPower(FrontLeft * motorPower);
            motorBackLeft.setPower(BackLeft * motorPower);
            motorBackRight.setPower(BackRight * motorPower);
        }



        //mecanumDrive_Cartesian(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x,angles.firstAngle, motorPower);

        //cutie viteze miscare
        if (gamepad1.y) {
            standard = 0.6;
        }
        if (gamepad1.a) {
            standard = 0.15;
        }
        if (gamepad1.x) {
            standard = 0.3;
        }
        if(gamepad1.right_trigger != 0){
            motorPower  = 0.6;
        }
        else
        if(gamepad1.left_trigger != 0 ){
            motorPower = 0.15;
        }
        else
            motorPower = standard;

        //cod fata spate dreapta stanga
        if (gamepad1.dpad_up)
        {
            motorFrontRight.setPower(motorPower);
            motorFrontLeft.setPower(-motorPower);
            motorBackLeft.setPower(-motorPower);
            motorBackRight.setPower(motorPower);
        }
        if (gamepad1.dpad_down)
        {
            motorFrontRight.setPower(-motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(motorPower);
            motorBackRight.setPower(-motorPower);
        }
        if (gamepad1.dpad_right)
        {
            motorFrontRight.setPower(motorPower);
            motorFrontLeft.setPower(motorPower);
            motorBackLeft.setPower(-motorPower);
            motorBackRight.setPower(-motorPower);
        }
        if (gamepad1.dpad_left)
        {
            motorFrontRight.setPower(-motorPower);
            motorFrontLeft.setPower(-motorPower);
            motorBackLeft.setPower(motorPower);
            motorBackRight.setPower(motorPower);
        }


        /* ----------------------------- </miscare> ----------------------------- */

        //telemetry.addLine();
        //telemetry.addData("Servo Bile:", servoBile);

        /** ---------------------------- <profiluri> ----------------------------  */
        if (gamepad2.y) {
            if (timp2 - timp1 >= 500 || timp1 == 0) {
                if (profil == 1)
                    profil = 2;
                else
                    profil = 1;
                timp1= System.currentTimeMillis();
            }
        }
            if(timp1 != 0)
                timp2= System.currentTimeMillis();
        //  if (gamepad2.a)
        // {
        //      profil=2;
        //  }

        /* ---------------------------- </profiluri> ----------------------------  */
        /** ----------------------------- <prindere> ----------------------------- */
        if (profil==1) {
            servoRidD_0.setPosition(servoPozD + 0.03);
            servoRidS_0.setPosition(servoPozS - 0.03);
            servoRidD_1.setPosition(servoPozD + 0.02);
            servoRidS_1.setPosition(servoPozS - 0.02);
            relicPrindere.setPosition(0);
            relicRidicare.setPosition(0);

            if (gamepad2.a) { // inchidere
                servoPozS = 0.224  ;//224
                servoPozD = 0.705;  //705
            }
            if (gamepad2.b) { // deschidere
                servoPozS = 0.4614;
                servoPozD = 0.4785;
            }
            if (gamepad2.x) // deschis si mai tare
            {
                servoPozS = 0.71;
                servoPozD = 0.29;
            }


            if (gamepad2.left_trigger != 0 && servoPozD > 0.4 && servoPozS < 0.5) {
                servoPozS += gamepad2.left_trigger / 25;
                servoPozD -= gamepad2.left_trigger / 25;
            }
            if (gamepad2.right_trigger != 0 && servoPozS > 0.25 && servoPozD < 0.7) {
                servoPozS -= gamepad2.right_trigger / 25;
                servoPozD += gamepad2.right_trigger / 25;
            }



        }

         /* ----------------------------- </prindere> ----------------------------- */

        /** ---------------------------- <ridicare> ----------------------------- */
        if (profil==1)
        {
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
        }

        telemetry.addData("Pozitie cod: ", servoPoz);
        telemetry.addData(" Dreapta: ",servoPozD);
        telemetry.addData(" Stanga: ", servoPozS);
         /* ----------------------------- </ridicare> ----------------------------- */

        /** ----------------------------- <relic> ----------------------------- */
        if (profil==2)
        {


            //dpad up ridici relic
            //dpad down lasi relic
            //x inchizi servo
            //b deschizi servo
            //trigger intinzi bratu


                inAer = false;

            double treapta = 0.03;
            //// incheietura
            relicRidicare.setPosition(pozIncheietura);
            if(gamepad2.dpad_up)
                pozIncheietura = relicSus;
            if(gamepad2.dpad_down)
                pozIncheietura = relicJos;
            if(gamepad2.left_stick_y < 0.1 || gamepad2.left_stick_y > 0.1)
                pozIncheietura -= gamepad2.left_stick_y / 50;
            ////grip
            if(gamepad2.x)
                relicPrindere.setPosition(relicPrins);
            if(gamepad2.b && !inAer)
                relicPrindere.setPosition(relicDeschis);
            ////motor
            if(gamepad2.right_bumper)
                motorRelic.setPower(-0.5);
            else
            if(gamepad2.left_bumper)
                motorRelic.setPower(0.5);
            else
                motorRelic.setPower(0);

            telemetry.addData("pozitieRelicPrindere", pozitieRelicPrindere);
            telemetry.addData("pozitieRelicRidicare",pozitieRelicRidicare);
            telemetry.update();

        }
        // telemetry.addData("Rotatie: ", angles.firstAngle);
        /* ---------------------------- </relic> ----------------------------- */
    }



    @Override
    public void stop() {        //functie stop, se apeleaza la apasarea butonului stop

        oprireServo();
        oprireMotoare();

    }


    public void oprireServo() {

        servoBile.close();
        servoRidD_0.close();
        servoRidS_0.close();
        servoRidD_1.close();
        servoRidS_1.close();
        relicPrindere.close();
        relicRidicare.close();

    }

    public void oprireMotoare() {

        motorBackRight.close();
        motorBackLeft.close();
        motorFrontRight.close();
        motorFrontLeft.close();
        motorRidicare.close();
        motorRelic.close();

    }

    public void miscareTreptata(Servo servo, double target, double treapta) throws InterruptedException {

        boolean sus = servo.getPosition() > target ? false : true;

        if (sus) {

            while (servo.getPosition() >= target) {

                servo.setPosition(servo.getPosition() + treapta);
                Thread.sleep(50);

            }

        }

        else {

            while (servo.getPosition() <= target) {

                servo.setPosition(servo.getPosition() - treapta);
                Thread.sleep(50);
            }

        }

    }

    //joystick input scale
    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput (double dVal){
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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

    public void mecanumDrive_Cartesian(double x, double y, double rotation, double gyroAngle, double speed) {
        @SuppressWarnings("LocalVariableName")
        double xIn = x;
        @SuppressWarnings("LocalVariableName")
        double yIn = y;
        // Negate y for the joystick.
        yIn = -yIn;
        // Compensate for gyro angle.
        double[] rotated = rotateVector(xIn, yIn, gyroAngle);
        xIn = rotated[0];
        yIn = rotated[1];

        double[] wheelSpeeds = new double[kMaxNumberOfMotors];
        wheelSpeeds[0] = -xIn - yIn - rotation;
        wheelSpeeds[1] = +xIn - yIn - rotation;
        wheelSpeeds[2] = xIn + yIn - rotation;
        wheelSpeeds[3] = -xIn + yIn - rotation;

        normalize(wheelSpeeds);
        motorFrontLeft.setPower(wheelSpeeds[0] * speed);
        motorFrontRight.setPower(wheelSpeeds[1] * speed);
        motorBackLeft.setPower(wheelSpeeds[2] * speed);
        motorBackRight.setPower(wheelSpeeds[3] * speed);
    }

    String formatRate ( float rate){
        return String.format("%.3f", rate);
    }

    String formatDegrees ( double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatAngle (AngleUnit angleUnit,double angle){
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    double[] rotateVector(double x, double y, double angle) {
        double cosA = Math.cos(angle * (Math.PI / 180.0));
        double sinA = Math.sin(angle * (Math.PI / 180.0));
        double[] out = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        return out;
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < kMaxNumberOfMotors; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < kMaxNumberOfMotors; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }


}