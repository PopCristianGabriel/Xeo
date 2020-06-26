package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.concurrent.TimeUnit;

/**
 * Created by (for example John) on 2/15/2018.
 */
@Disabled
@Autonomous(name = "Cod Nou",group = "Autonom")
public class CodHolonomAuto  extends LinearOpMode{

    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    double vitezeHolonom[] = new double[4];
    double alpha;
    double beta = 0;
    private boolean calibration_complete = false;

    @Override
    public void runOpMode() throws InterruptedException{

        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);

        motorFrontRight = hardwareMap.dcMotor.get("motor front right");
        motorFrontLeft = hardwareMap.dcMotor.get("motor front left");
        motorBackLeft = hardwareMap.dcMotor.get("motor back left");
        motorBackRight = hardwareMap.dcMotor.get("motor back right");

        while (!calibration_complete) {
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

        navx_device.zeroYaw();

        telemetry.addData("Rotatie: ", navx_device.getYaw());
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            TimeUnit.MILLISECONDS.sleep(5000);
            alpha=navx_device.getYaw()-45;
            beta=90;
            formuleHolonom(alpha,beta,vitezeHolonom);
            da_teJosDePeRampa(vitezeHolonom);

        }

    }

    public void formuleHolonom(double alpha, double beta, double[] vitezeHolonom) {

        double xM, yM, scale=0.75;
        xM = (Math.cos(alpha)+Math.cos(beta))/2;
        yM = (Math.sin(alpha)+Math.sin(beta))/2;
        vitezeHolonom[0] = scale*(-xM-yM);
        vitezeHolonom[1] = scale*(-xM+yM);
        vitezeHolonom[2] = scale*(xM+yM);
        vitezeHolonom[3] = scale*(xM-yM);
        telemetry.addData("Rotatie: ", navx_device.getYaw());
        telemetry.update();

    }

    public void da_teJosDePeRampa(double viteza[]){

        motorFrontLeft.setPower(viteza[0]);
        motorFrontRight.setPower(viteza[1]);
        motorBackRight.setPower(viteza[2]);
        motorBackLeft.setPower(viteza[3]);

        double treshHold = .5;
        while(Math.abs(navx_device.getRoll()) >= treshHold && opModeIsActive()) {
            idle();
        }
        oprire();
    }

    public void oprire(){
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

}
