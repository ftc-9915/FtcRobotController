package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

@Config
public class Flywheel implements SubsystemBase {
    private DcMotorEx flywheel;
    private static int ticksPerRev = 28;
    private static String name = "launcherMotor";

    PIDFCoefficients PIDF;
    public static double kP = 15;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 13.7;

    //in degrees
    private double shooterAngle;
    //in meters
    private double shooterHeight;
    private double goalHeight = 0.9017;
    private double  wheelRadius;

    private VoltageSensor batteryVoltageSensor;


    //Common Shooting RPMs
    public static double powerShotRPM;

    
    public Flywheel(HardwareMap hardwareMap){

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        flywheel = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //remove rpm limit
//        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
//        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//        flywheel.setMotorType(motorConfigurationType);

        PIDFCoefficients PIDF = new PIDFCoefficients(kP,  kI,   kD,   kF * 13.21 / batteryVoltageSensor.getVoltage());
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

    }

    @Override
    public boolean isBusy() {
        return flywheel.isBusy();
    }


    public boolean setRPM (double rpm){

        double rotationsPerSecond = rpm / 60;
        double ticksPerSecond = rotationsPerSecond * ticksPerRev;
        flywheel.setVelocity(ticksPerSecond);

        return true;
    }


    public double getRPM () {
        double ticksPerSecond = flywheel.getVelocity();
        double rotationsPerSecond = ticksPerSecond / ticksPerRev;
        double rpm = Math.abs(rotationsPerSecond * 60);

        return rpm;
    }


    public boolean setRawPower(double power) {
        if(!isBusy()) {
            flywheel.setPower(power);
            return true;
        }
        return false;
    }



    //Math probably inaccurate, will need calibration curve
//    public void setRPMWithDistance(double horizontalDistance){
//        double linearVelocity = horizontalDistance / Math.cos(shooterAngle) * Math.sqrt(2 * (shooterHeight - goalHeight - Math.tan(shooterAngle))
//                * (horizontalDistance) / 9.8);
//
//        double radiansPerSecond = linearVelocity / wheelRadius;
//        double rpm = radiansPerSecond * 60 / (2*Math.PI);
//    }

}
