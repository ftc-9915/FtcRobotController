package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

@Config
public class Shooter implements SubsystemBase {
    private DcMotorEx flywheel;
    private int ticksPerRev = 28;
    private String name = "launcherMotor";

    PIDFCoefficients PIDF;
    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;

    //in degrees
    private double shooterAngle;
    //in meters
    private double shooterHeight;
    private double goalHeight = 0.9017;
    private double  wheelRadius;

    
    
    public Shooter(HardwareMap hardwareMap){

        flywheel = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //remove rpm limit
        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheel.setMotorType(motorConfigurationType);

        PIDFCoefficients PIDF = new PIDFCoefficients(kP,  kI,   kD,  kF);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        flywheel.setVelocity(0);
    }

    @Override
    public boolean isBusy() {
        return flywheel.isBusy();
    }


    public boolean setRPM (double rpm){
        if(!isBusy()) {
            //respond to active updates
            PIDFCoefficients PIDF = new PIDFCoefficients(kP,  kI,   kD,  kF);
            flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);

            double rotationsPerSecond = rpm / 60;
            double ticksPerSecond = rotationsPerSecond * ticksPerRev;
            flywheel.setVelocity(ticksPerSecond);

            return true;
        }
        return false;
    }


    public double getRPM () {
        double ticksPerSecond = flywheel.getVelocity();
        double rotationsPerSecond = ticksPerSecond / ticksPerRev;
        double rpm = rotationsPerSecond * 60;

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
