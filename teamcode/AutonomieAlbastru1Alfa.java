package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.adafruit.AdafruitI2cColorSensor;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.concurrent.TimeUnit;

import static java.lang.System.arraycopy;
import static java.lang.System.currentTimeMillis;
import static java.lang.System.in;


/**
 * Created by (for example John) on 2/14/2018.
 */

@Autonomous(name = "AutonomieAlbastru1Alfa", group = "AutonomieAlbastru1Alfa")
public class AutonomieAlbastru1Alfa extends LinearOpMode {

    public ModernRoboticsI2cRangeSensor rangeSensor;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    DcMotor mRid;
    double servoPozS = 0.75;
    double servoPozD = 0.25;
    double pozitieServoBileJos = 0.99; // de vazut
    double pozitieServoBileSus = 0.01;
    double vitezaIntoarcere = 0.08;//0.07
    double bilaStanga = - 10;
    double bilaDreapta = 10;
    double treshHold = 8;
    double vitezaMiscare = 0.1;
    int raft = 0;
    double MidS = 0.69;
    double MidD = 0.28;
    Servo servoBile;
    Servo servoBileSD;
    Servo CubSJ;
    Servo CubDJ;
    Servo CubSS;
    Servo CubDS;
   // double CloseS = 0.234;
    //double CloseD = 0.695;



    double OpenS=0.77;
    double OpenD= 0.23;
    double CloseS = 0.224;
    double CloseD = 0.705;
    double rollInitial;
    double altInitial;

    double VitezaIntoarcereMare = 0.23;

    double alpha;
    double beta = 0;
    double zeroRoll = 0;
    private boolean calibration_complete = false;
    double ServS = 0.26;
    double ServD = 0.67;
    double ServSLas = 0.4614;
    double ServDLas = 0.4785;
    boolean terminat ;
    double distRaft1 = 45;
    double distRaft2 = 63;
    double distRaft3 = 81;
    double gramadacuburi = 130;
    double bilaAlbastra = 35000;
    double bilaRosie = 55000;
    static final int LED_CHANNEL = 5;


    static final int gramada=4;


    AdafruitI2cColorSensor colorSensor;
    ColorSensor colorSensorPrioritar;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() throws InterruptedException {

        terminat = false;
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcXkeIj/////AAAAmRV1vqYUzkRcn7rBLlUty9FuNYEzPAn4gEn+mrBl7eKeI3qA1oEWAVrY8uLi+jewlHe7i56Zoza2WB+sEq6MXpjOjQm4MCVCe3CmeVuCQEyVDU9QVGqoO1swzzY6x2k9yRoQgmuIW1BxEwxj4mNeFwPGZEsWJlofpFHnxuKNdO2DzQ0SeNAl+iksEodJBR1NKv22ORk0snNYX1u1b9dzsqZeq/ONXHhwm9KXDqCjDhwCndzm2oMHu7tASBsRTDjVJXnWTqVFsi9QQQqq2IX4Kxybhu/vck51l/f9gPItPE/YoVIL3UqyvKz5YDHpZbqrOYrpFLxtf8NFgNI/Aq5pNwcupiqayd5FM0hQzsWC2IHu";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        double position = 0;

        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");
        colorSensor = (AdafruitI2cColorSensor) hardwareMap.get("color_sensor");
        colorSensorPrioritar = hardwareMap.colorSensor.get("color_sensor_2");

        servoBile = hardwareMap.servo.get("servo bile");
        servoBileSD=hardwareMap.servo.get("servo bile SD");
        servoBile.setPosition(pozitieServoBileSus);
        servoBileSD.setPosition(0.45);

        CubSJ = hardwareMap.servo.get("servo rid s 0");
        CubDJ = hardwareMap.servo.get("servo rid d 0");
        CubSS = hardwareMap.servo.get("servo rid s 1");
        CubDS = hardwareMap.servo.get("servo rid d 1");
        mRid = hardwareMap.dcMotor.get("motor rid");
        CubSJ.setPosition(OpenS);
        CubSS.setPosition(OpenS);
        CubDJ.setPosition(OpenD);
        CubDS.setPosition(OpenD);
        rangeSensor = (ModernRoboticsI2cRangeSensor) hardwareMap.opticalDistanceSensor.get("sensor_distance");

        servoBileSD.setPosition(0.5);
        servoBile.setPosition(pozitieServoBileSus);

        while (!calibration_complete && !Thread.currentThread().isInterrupted()) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */

            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
            if (calibration_complete)
                telemetry.addData("navX-Micro", "M-am calibrat");
            telemetry.update();
        }


        rollInitial = navx_device.getPitch();
        telemetry.addData("Roll: ", rollInitial);
        telemetry.addLine();
        altInitial = navx_device.getAltitude();

        navx_device.zeroYaw();

        relicTrackables.activate();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);



        while (!opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            switch (vuMark) {

                case LEFT:
                    raft = 1;
                    break;
                case CENTER:
                    raft = 2;
                    break;
                case RIGHT:
                    raft = 3;
                    break;
                default:
                    raft = 2;
                    break;


            }

            telemetry.addData("Rotatie: ", navx_device.getYaw());
            telemetry.addLine();
            telemetry.addData("Albastru: ", colorSensor.blue());
            telemetry.addLine();
            telemetry.addData("Rosu: ", colorSensor.red());
            telemetry.addLine();
            telemetry.addData("distanta initiala:", rangeSensor.getDistance(DistanceUnit.CM));
            //telemetry.addData("Altitudine: ", navx_device.getAltitude());
            telemetry.addData("Roll: ", navx_device.getPitch());
            telemetry.addLine();
            telemetry.addData("VuMark:", vuMark);
            telemetry.addData("Unghi ", navx_device.getYaw());
            telemetry.addData("poz servo", CubSJ.getPosition());
            telemetry.update();

        }



        while (opModeIsActive() && ! terminat) {

            zeroRoll = navx_device.getPitch();
            servoBile.setPosition(pozitieServoBileJos);
            TimeUnit.MILLISECONDS.sleep(1200);
            prindeCub(ServS,ServD);


            ///--------------------------------------------


            telemetry.addData("Rotatie: ", navx_device.getYaw());
            telemetry.update();


            if (colorSensorPrioritar.red() > colorSensorPrioritar.blue()) {
                josBila(true);      // am dat jos bila rosie
                servoBile.setPosition(pozitieServoBileSus);
              //  TimeUnit.MILLISECONDS.sleep(500);

            } else if (colorSensorPrioritar.red() < colorSensorPrioritar.blue()) {
                josBila(false);
                servoBile.setPosition(pozitieServoBileSus);
              //  TimeUnit.MILLISECONDS.sleep(500);
            }

            else {

                if (colorSensor.red() > bilaRosie) {

                    josBila(false);
                    servoBile.setPosition(pozitieServoBileSus);
                   // TimeUnit.MILLISECONDS.sleep(500);

                }

                else if(colorSensor.blue() > bilaAlbastra){

                    josBila(true);      // am dat jos bila rosie
                    servoBile.setPosition(pozitieServoBileSus);
                    //TimeUnit.MILLISECONDS.sleep(500);

                }


            }

            servoBile.setPosition(pozitieServoBileSus);

            oprire();

            TimeUnit.MILLISECONDS.sleep(300);
            //-------------------------------------------------------------------------------------



            da_teJosDePeRampa(vitezaMiscare+0.09, zeroRoll, altInitial);
            //rotire(0,1.8,0.04);


            if(raft!=1) {
              //  mergiDiagonalaStanga(vitezaMiscare + 0.1);
              //  TimeUnit.MILLISECONDS.sleep(400);


            //    mergiDreapta(vitezaMiscare + 0.1);
            //    TimeUnit.MILLISECONDS.sleep(300);
            }

            else
            {

                mergiFata(-vitezaMiscare,0);
                TimeUnit.MILLISECONDS.sleep(300);

            }


          raft--;


            if(raft!=0)

            du_teLaRaft(raft,vitezaMiscare+0.02);

            else {
                du_teLaRaft(1, vitezaMiscare - 0.005);

            }
            rotireTreptata(-90,treshHold,vitezaIntoarcere+0.04);
            if(raft ==0) {
                mergiDreapta(vitezaMiscare);
                TimeUnit.MILLISECONDS.sleep(200);
                oprire();
            }

            raft++;

            puneCub(vitezaMiscare+0.07);
            mergiFata(-vitezaMiscare-0.2,0);
            TimeUnit.MILLISECONDS.sleep(200);

            servos(MidS,MidD);

               if(raft==1)
                       rotireTreptata(90,treshHold+1,vitezaIntoarcere+0.04);
                               else
           if(raft==2)
            rotireTreptata(90,treshHold+1,vitezaIntoarcere+0.04);
            else
                if(raft==3)
                    rotireTreptata(110,treshHold+1,vitezaIntoarcere+0.04);



            mergiFata(0.99,0);
            TimeUnit.MILLISECONDS.sleep(950);



            servos(CloseS,CloseD);

            oprire();
            TimeUnit.MILLISECONDS.sleep(350);


            mRid.setPower(1);


            TimeUnit.MILLISECONDS.sleep(350);




            mergiFata(-0.99,0);
            TimeUnit.MILLISECONDS.sleep(750);

            mRid.setPower(0);

            rotireTreptata(-90,treshHold+1,vitezaIntoarcere+0.04);



            if(raft!=1)
            {


                if(raft==3)
                {mergiDiagonalaStanga(-vitezaMiscare-0.1);
            TimeUnit.MILLISECONDS.sleep(1500);}
                else
                {
                    mergiFata(vitezaMiscare+0.1,0);
                    TimeUnit.MILLISECONDS.sleep(1250);
                }



            }
            else
            {mergiDiagonalaDreapta(-vitezaMiscare-0.1);
                TimeUnit.MILLISECONDS.sleep(1500);}

            servos(OpenS,OpenD);

            mergiFata(vitezaMiscare,0);
            TimeUnit.MILLISECONDS.sleep(400);

            mergiFata(-vitezaMiscare,0);
            TimeUnit.MILLISECONDS.sleep(1000);

            oprire();
                /*
                     if(raft==1)
                     {mergiDreapta(vitezaMiscare+0.07);
            TimeUnit.MILLISECONDS.sleep(300);}
                    if(raft==2)
                    {mergiDreapta(vitezaMiscare+0.07);
                        TimeUnit.MILLISECONDS.sleep(300);}
                 if(raft==3)
            {mergiDreapta(-vitezaMiscare-0.07);
                TimeUnit.MILLISECONDS.sleep(300);}
                  */



            //puneCub(vitezaMiscare+0.031);


            //------------------------------------------------------------------------------------------

            oprire();

            oprireTot();



            terminat = true;
        }
        oprireTot();
    }



    public void mergiDiagonalaStanga(double viteza){
        motorFrontLeft.setPower(viteza);
        motorBackRight.setPower(-viteza);
    }


    public void mergiDiagonalaDreapta(double viteza){
        motorFrontRight.setPower(-viteza);
        motorBackLeft.setPower(viteza);
    }

    public void du_teLaRaft(int raft, double viteza) throws InterruptedException{

        double dist1 = rangeSensor.getDistance(DistanceUnit.CM);
        double dist2 = dist1;
        mergiFata(viteza, 0);
        do{

            telemetry.addData("sunt la raftul:", raft);
            telemetry.update();

            dist2 = rangeSensor.getDistance(DistanceUnit.CM);

            if(dist1 - dist2 > 5) {

                raft--;
                if(raft != 0)
                    TimeUnit.MILLISECONDS.sleep(1000);
                telemetry.addData("sunt la raftul:", raft);
                telemetry.update();
            }

        }while(raft > 0 && opModeIsActive());

        oprire();


    }

    public void servos(double s, double d){
        CubDJ.setPosition(d);
        CubDS.setPosition(d);
        CubSJ.setPosition(s);
        CubSS.setPosition(s);

    }
    public void rotire(double tinta, double treshHold, double viteza) throws InterruptedException {
        int incercari = 0;
        boolean suntPeTinta = false;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga = viteza;
            double mergiDreapta = -viteza;

            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive()) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga
                telemetry.addData("Rotatie: ", navx_device.getYaw());
                telemetry.addLine();
                telemetry.addData("incercari:", incercari);
                telemetry.addLine();
                telemetry.addData("treshhold:", treshHold);
                telemetry.addLine();
                telemetry.addData("viteza:", viteza);
                telemetry.update();
                motorFrontLeft.setPower(mergiStanga);
                motorFrontRight.setPower(mergiStanga);
                motorBackRight.setPower(mergiStanga);
                motorBackLeft.setPower(mergiStanga);
            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)     // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= tinta - treshHold && opModeIsActive()) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

                    telemetry.addData("Rotatie: ", navx_device.getYaw());
                    telemetry.addLine();
                    telemetry.addData("incercari:", incercari);
                    telemetry.addLine();
                    telemetry.addData("treshhold:", treshHold);
                    telemetry.addLine();
                    telemetry.addData("viteza:", viteza);
                    telemetry.update();

                    motorFrontLeft.setPower(mergiDreapta);
                    motorFrontRight.setPower(mergiDreapta);
                    motorBackRight.setPower(mergiDreapta);
                    motorBackLeft.setPower(mergiDreapta);
                }
                oprire();
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;


            incercari++;
            if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }

            if (suntPeTinta)
                oprire();

        }


    }

    public double actualizeazaViteza(double tinta, double unghiActual,int incercari){
        double diferenta = Math.abs(tinta - unghiActual);
        double scadere = 0;
        if(incercari % 2 == 0) {
            scadere = incercari / 2 * 0.01;
        }
        else
            scadere = (incercari-1) / 2 * 0.01;

        if(diferenta <= 30){

            return vitezaIntoarcere - scadere;
        }
        else
            return VitezaIntoarcereMare;
    }


    public void puneCub(double viteza) throws InterruptedException {

        mergiFata(viteza, 0);
        TimeUnit.MILLISECONDS.sleep(700);

        oprire();

        lasaCub(ServSLas, ServDLas);
        TimeUnit.MILLISECONDS.sleep(300);


        mergiFata(-viteza, 0);
        TimeUnit.MILLISECONDS.sleep(400);

        mRid.setPower(-1);
        TimeUnit.MILLISECONDS.sleep(500);

        mRid.setPower(0);

        ///inchideServo();

         Asez_servo(raft);

        mergiFata(viteza, 0);
        TimeUnit.MILLISECONDS.sleep(1000);
        mergiFata(-viteza, 0);
        TimeUnit.MILLISECONDS.sleep(400);
        oprire();

    }

    public void  Asez_servo(double raft) throws InterruptedException {

        if(raft==1)
          servos(MidS,CloseD);
        else
            if(raft==2)
                servos(CloseS,CloseD);
                        else
                             if(raft==3)
                                 servos(CloseS,MidD);

    }

    public void rotireTreptata(double tinta, double treshHold, double viteza) throws InterruptedException {
        int incercari = 0;
        boolean suntPeTinta = false;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga;
            double mergiDreapta;

            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive()) {// cat timp sunt in dreapta tintei vreau sa merg in stanga
                viteza = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                if(viteza > 0.2)
                    viteza -= 0.05;
                mergiStanga = viteza;
                mergiDreapta = -viteza;
                telemetry.addData("Rotatie: ", navx_device.getYaw());
                telemetry.addLine();
                telemetry.addData("incercari:", incercari);
                telemetry.addLine();
                telemetry.addData("treshhold:", treshHold);
                telemetry.addLine();
                telemetry.addData("viteza:", viteza);
                telemetry.update();
                motorFrontLeft.setPower(mergiStanga);
                motorFrontRight.setPower(mergiStanga);
                motorBackRight.setPower(mergiStanga);
                motorBackLeft.setPower(mergiStanga);
            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)     // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= tinta - treshHold && opModeIsActive()) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta
                    viteza = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                    mergiStanga = viteza;
                    mergiDreapta = -viteza;
                    telemetry.addData("Rotatie: ", navx_device.getYaw());
                    telemetry.addLine();
                    telemetry.addData("incercari:", incercari);
                    telemetry.addLine();
                    telemetry.addData("treshhold:", treshHold);
                    telemetry.addLine();
                    telemetry.addData("viteza:", viteza);
                    telemetry.update();

                    motorFrontLeft.setPower(mergiDreapta);
                    motorFrontRight.setPower(mergiDreapta);
                    motorBackRight.setPower(mergiDreapta);
                    motorBackLeft.setPower(mergiDreapta);
                }
                oprire();
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= tinta - treshHold && navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;


            incercari++;
            /*if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }
            */

            if (suntPeTinta)
                oprire();

        }


    }



    public void mergiFata(double viteza, double plus){
        motorFrontLeft.setPower(-viteza);
        motorFrontRight.setPower(viteza + plus);
        motorBackRight.setPower(viteza );
        motorBackLeft.setPower(-viteza - plus);
    }

    public void mergiDreapta(double viteza){
        motorFrontLeft.setPower(viteza);
        motorFrontRight.setPower(viteza);
        motorBackRight.setPower(-viteza);
        motorBackLeft.setPower(-viteza);
    }



    public void oprireTot() {

        navx_device.close();
        rangeSensor.close();
        colorSensor.close();
        colorSensorPrioritar.close();
        hardwareMap.deviceInterfaceModule.get("dim").setDigitalChannelState(LED_CHANNEL, false);
        oprire();

    }

    public void prindeCub(double stanga, double dreapta) throws InterruptedException {

        CubSJ.setPosition(MidS);
        CubDJ.setPosition(MidD);
        CubSS.setPosition(MidS);
        CubDS.setPosition(MidD);

      //  TimeUnit.MILLISECONDS.sleep(300);
        mRid.setPower(-1);
        TimeUnit.MILLISECONDS.sleep(300);
        mRid.setPower(0);

        CubSJ.setPosition(CloseS);
        CubDJ.setPosition(CloseD);
        CubSS.setPosition(CloseS);
        CubDS.setPosition(CloseD);;


        TimeUnit.MILLISECONDS.sleep(500);
        mRid.setPower(1);
        TimeUnit.MILLISECONDS.sleep(600);
        mRid.setPower(0);

    }


    public void lasaCub(double stanga, double dreapta) throws InterruptedException {
        CubSJ.setPosition(stanga);
        CubDJ.setPosition(dreapta);
        CubSS.setPosition(stanga);
        CubDS.setPosition(dreapta);
    }



    ////////----------------------------------------------------------------------------------------------------------
    public void da_teJosDePeRampa(double viteza, double rollInitial, double altInitial){

        mergiFata(viteza, 0);

        double treshHold = .2;
        while (Math.abs(navx_device.getPitch()) <= 6 && opModeIsActive()) {
            idle();
        }
        while (Math.abs(navx_device.getPitch()) >=3 && opModeIsActive()) {
            telemetry.addData("Roll: ", Math.abs(navx_device.getPitch()));
            telemetry.update();
        }
        oprire();
    }


    public void josBila(boolean stanga) throws InterruptedException {

        if (stanga) {

            servoBileSD.setPosition(1);

        }
        else {

            servoBileSD.setPosition(0);

        }

        TimeUnit.MILLISECONDS.sleep(500);
        servoBileSD.setPosition(0.5);

    }

    public void du_teLaRaft2(int raft, double viteza){

        double tinta;

        switch(raft) {

            case 1:
                tinta=distRaft1;
                break;
            case 2:
                tinta=distRaft2;
                break;
            case 3:
                tinta=distRaft3;
                break;
            case 4:
                tinta=gramadacuburi;
                break;

            default:
                tinta=distRaft2;

        }

        double distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);

        while(distantaCurenta < tinta)
        {
            distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);
            mergiDreapta(viteza);
            telemetry.addData("distanta Curenta :",distantaCurenta);
            telemetry.update();
        }
        oprire();
    }

    public void oprire(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    public void inchideServo(){
        CubSJ.setPosition(CloseS);
        CubDJ.setPosition(CloseD);
        CubSS.setPosition(CloseS);
        CubDS.setPosition(CloseD);
    }

    public void rotireSpeciala2(double tinta, double treshHold, double viteza) throws InterruptedException {

        int incercari = 0;
        boolean suntPeTinta = false;
        boolean amTrecut;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga = viteza;
            double mergiDreapta = -viteza;
            amTrecut = false;
            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive() && !amTrecut) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga
                telemetry.addData("Rotatie: ", navx_device.getYaw());
                telemetry.addLine();
                telemetry.addData("incercari:", incercari);
                telemetry.addLine();
                telemetry.addData("treshhold:", treshHold);
                telemetry.addLine();
                telemetry.addData("viteza:", viteza);
                telemetry.update();
                motorFrontLeft.setPower(mergiStanga);
                motorFrontRight.setPower(mergiStanga);
                motorBackRight.setPower(mergiStanga);
                motorBackLeft.setPower(mergiStanga);
                double unghi1 = navx_device.getYaw();
                TimeUnit.MILLISECONDS.sleep(100);
                double unghi2 = navx_device.getYaw();
                if (unghi2 > unghi1)
                    amTrecut = true;

            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= 180 - treshHold && navx_device.getYaw() <= tinta + treshHold) {    // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            }
            amTrecut = false;
            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= 180 - treshHold && opModeIsActive() && !amTrecut) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

                    telemetry.addData("Rotatie: ", navx_device.getYaw());
                    telemetry.addLine();
                    telemetry.addData("incercari:", incercari);
                    telemetry.addLine();
                    telemetry.addData("treshhold:", treshHold);
                    telemetry.addLine();
                    telemetry.addData("viteza:", viteza);
                    telemetry.update();

                    motorFrontLeft.setPower(mergiDreapta);
                    motorFrontRight.setPower(mergiDreapta);
                    motorBackRight.setPower(mergiDreapta);
                    motorBackLeft.setPower(mergiDreapta);
                    double unghi1 = navx_device.getYaw();
                    TimeUnit.MILLISECONDS.sleep(100);
                    double unghi2 = navx_device.getYaw();
                    if (unghi2 < unghi1)
                        amTrecut = true;
                }
                oprire();
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= 180 - treshHold && navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;
            amTrecut = false;

            incercari++;
            if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }

            if (suntPeTinta)
                oprire();

        }

    }

    double actualizeazaVitezaPtRafturi(double dist){
        if(dist > 50)
            return 0.8;
        else
        if(dist <50 && dist > 30)
            return 0.5;
        else
            return vitezaMiscare+0.07;
    }

    public void du_teLaRaft2Treptat(int raft, double viteza){

        double tinta;

        switch(raft) {

            case 1:
                tinta=distRaft1;
                break;
            case 2:
                tinta=distRaft2;
                break;
            case 3:
                tinta=distRaft3;
                break;
            case 4:
                tinta=gramadacuburi;
                break;

            default:
                tinta=distRaft2;

        }



        double distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);
        if(distantaCurenta < tinta) {
            while (distantaCurenta < tinta) {
                double dist = Math.abs(distantaCurenta-rangeSensor.getDistance(DistanceUnit.CM));
                viteza = actualizeazaVitezaPtRafturi(dist);
                distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);
                mergiDreapta(viteza);
                telemetry.addData("distanta Curenta :", distantaCurenta);
                telemetry.update();
            }
            oprire();
        }

        else{
            while(distantaCurenta > tinta)
            {
                double dist = Math.abs(distantaCurenta-rangeSensor.getDistance(DistanceUnit.CM));
                viteza = actualizeazaVitezaPtRafturi(dist);
                distantaCurenta = rangeSensor.getDistance(DistanceUnit.CM);
                mergiDreapta(-viteza);
                telemetry.addData("distanta Curenta :",distantaCurenta);
                telemetry.update();
            }
            oprire();
        }

    }
    public void rotireSpecialaTreptata(double tinta, double treshHold, double viteza) throws InterruptedException {

        int incercari = 0;
        boolean suntPeTinta = false;
        boolean amTrecut;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga;
            double mergiDreapta;
            mergiStanga = viteza;
            mergiDreapta = -viteza;
            amTrecut = false;
            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive() && !amTrecut) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga
                mergiStanga = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                mergiDreapta = -actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                telemetry.addData("Rotatie: ", navx_device.getYaw());
                telemetry.addLine();
                telemetry.addData("incercari:", incercari);
                telemetry.addLine();
                telemetry.addData("treshhold:", treshHold);
                telemetry.addLine();
                telemetry.addData("viteza:", viteza);
                telemetry.update();
                motorFrontLeft.setPower(mergiStanga);
                motorFrontRight.setPower(mergiStanga);
                motorBackRight.setPower(mergiStanga);
                motorBackLeft.setPower(mergiStanga);
                if (navx_device.getYaw() >= 0)
                    amTrecut = true;
            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= 180 - treshHold || navx_device.getYaw() <= tinta + treshHold) {    // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            }
            amTrecut = false;
            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= -tinta - treshHold && opModeIsActive() && !amTrecut) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta
                    mergiStanga = actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                    mergiDreapta = -actualizeazaViteza(tinta,navx_device.getYaw(),incercari);
                    telemetry.addData("Rotatie: ", navx_device.getYaw());
                    telemetry.addLine();
                    telemetry.addData("incercari:", incercari);
                    telemetry.addLine();
                    telemetry.addData("treshhold:", treshHold);
                    telemetry.addLine();
                    telemetry.addData("viteza:", viteza);
                    telemetry.update();

                    motorFrontLeft.setPower(mergiDreapta);
                    motorFrontRight.setPower(mergiDreapta);
                    motorBackRight.setPower(mergiDreapta);
                    motorBackLeft.setPower(mergiDreapta);
                    if (navx_device.getYaw() >= -180 && navx_device.getYaw() <= -160)
                        amTrecut = true;
                }
                oprire();
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= 180 - treshHold || navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;
            amTrecut = false;

            incercari++;
            /*if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }
            */

            if (suntPeTinta)
                oprire();

        }

    }
    public void rotireSpeciala(double tinta, double treshHold, double viteza) throws InterruptedException {

        int incercari = 0;
        boolean suntPeTinta = false;
        boolean amTrecut;
        while (!suntPeTinta && opModeIsActive()) {
            double mergiStanga = viteza;
            double mergiDreapta = -viteza;
            amTrecut = false;
            while (navx_device.getYaw() >= tinta + treshHold && opModeIsActive() && !amTrecut) {      // cat timp sunt in dreapta tintei vreau sa merg in stanga
                telemetry.addData("Rotatie: ", navx_device.getYaw());
                telemetry.addLine();
                telemetry.addData("incercari:", incercari);
                telemetry.addLine();
                telemetry.addData("treshhold:", treshHold);
                telemetry.addLine();
                telemetry.addData("viteza:", viteza);
                telemetry.update();
                motorFrontLeft.setPower(mergiStanga);
                motorFrontRight.setPower(mergiStanga);
                motorBackRight.setPower(mergiStanga);
                motorBackLeft.setPower(mergiStanga);
                if (navx_device.getYaw() >= 0)
                    amTrecut = true;
            }
            oprire();
            TimeUnit.MILLISECONDS.sleep(500);


            if (navx_device.getYaw() >= 180 - treshHold || navx_device.getYaw() <= tinta + treshHold) {    // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;

            }
            amTrecut = false;
            if (!suntPeTinta) {     // daca nu m-am pozitionat bine, incerc sa merg in dreapta. Robotul nu se poate opri decat in limitele treshHoldului sau in partea stanga
                while (navx_device.getYaw() <= -tinta - treshHold && opModeIsActive() && !amTrecut) {    // cat timp sunt in partea stanga a tintei vreau sa merg in dreapta

                    telemetry.addData("Rotatie: ", navx_device.getYaw());
                    telemetry.addLine();
                    telemetry.addData("incercari:", incercari);
                    telemetry.addLine();
                    telemetry.addData("treshhold:", treshHold);
                    telemetry.addLine();
                    telemetry.addData("viteza:", viteza);
                    telemetry.update();

                    motorFrontLeft.setPower(mergiDreapta);
                    motorFrontRight.setPower(mergiDreapta);
                    motorBackRight.setPower(mergiDreapta);
                    motorBackLeft.setPower(mergiDreapta);
                    if (navx_device.getYaw() >= -180 && navx_device.getYaw() <= -160)
                        amTrecut = true;
                }
                oprire();
                TimeUnit.MILLISECONDS.sleep(500);
            }

            if (navx_device.getYaw() >= 180 - treshHold || navx_device.getYaw() <= tinta + treshHold)   // daca sunt pe o parte sau o alta a tintei si in limitele tresholdului ma opresc
                suntPeTinta = true;
            amTrecut = false;

            incercari++;
            if (incercari % 2 == 0 && incercari != 0) {   // daca dupa doua incercari nu a reusit, il ajut putin . O incercare inseamna o rotire spre stanga si o rotire spre dreapta sau viceversa
                // treshHold += 0.5;
                viteza -= 0.01;
            }

            if (suntPeTinta)
                oprire();

        }

    }

}