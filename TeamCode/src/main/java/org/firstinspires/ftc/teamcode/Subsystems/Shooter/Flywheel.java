package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

@Config
public class Flywheel implements SubsystemBase {
    public DcMotorEx flywheelMotor;
    private static int MOTOR_TICKS_PER_REV = 28;
    public static double MOTOR_MAX_RPM = 5400;
    private static int MOTOR_GEAR_RATIO = 1;
    private static String name = "launcherMotor";

    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(30, 0, 0, 15.5);

    double lastKp = 0.0;
    double lastKi = 0.0;
    double lastKd = 0.0;
    double lastKf = getMotorVelocityF();
    public double activeTargetRPM = 3200; // target RPM when flywheel is on
    boolean flywheelOn;

    //in degrees
    private double shooterAngle;
    //in meters
    private double shooterHeight;
    private double goalHeight = 0.9017;
    private double  wheelRadius;

    private VoltageSensor batteryVoltageSensor;

    //logging
    private FtcDashboard dashboard;



    //Common Shooting RPMs
    public static double powerShotRPM;

    
    public Flywheel(HardwareMap hardwareMap){

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //unlock max rpm
        MotorConfigurationType motorConfigurationType = flywheelMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        flywheelMotor.setMotorType(motorConfigurationType);

        //remove rpm limit
//        MotorConfigurationType motorConfigurationType = flywheel.getMotorType().clone();
//        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//        flywheel.setMotorType(motorConfigurationType);

//        PIDFCoefficients PIDF = new PIDFCoefficients(kP,  kI,   kD,   kF * 13.21 / batteryVoltageSensor.getVoltage());

        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        setPIDFCoefficients(MOTOR_VELO_PID);

        turnOff();

    }

    @Override
    public boolean isBusy() {
        return flywheelMotor.isBusy();
    }


    public void  setRPM (double rpm){
        if (rpm == 0) {
            turnOff();
        } else {
            flywheelOn = true;
//        PIDF = new PIDFCoefficients(kP,  kI,   kD,   kF * 13.21 / batteryVoltageSensor.getVoltage());
            activeTargetRPM = rpm;
            //change PIDF on the fly when needed
            if (lastKp != MOTOR_VELO_PID.p || lastKi != MOTOR_VELO_PID.i || lastKd != MOTOR_VELO_PID.d || lastKf != MOTOR_VELO_PID.f) {
                setPIDFCoefficients(MOTOR_VELO_PID);

                lastKp = MOTOR_VELO_PID.p;
                lastKi = MOTOR_VELO_PID.i;
                lastKd = MOTOR_VELO_PID.d;
                lastKf = MOTOR_VELO_PID.f;
            }

            flywheelMotor.setVelocity(rpmToTicksPerSecond(rpm));
        }
    }

    public void turnOn() {
        setRPM(activeTargetRPM);
        flywheelOn = true;
    }

    public void turnOff() {
        flywheelMotor.setVelocity(0);
        flywheelOn = false;
    }

    public void toggleOnOff() {
        if (flywheelOn) {
            turnOff();
        } else {
            turnOn();
        }
    }

    //within 4%
    public boolean atTargetRPM() {
        return UtilMethods.inRange(getVelocity(), 0.96 * rpmToTicksPerSecond(activeTargetRPM), 1.04 * rpmToTicksPerSecond(activeTargetRPM));
    }


    public double getRPM () {
        double ticksPerSecond = flywheelMotor.getVelocity();
        double rotationsPerSecond = ticksPerSecond / MOTOR_TICKS_PER_REV;
        double rpm = Math.abs(rotationsPerSecond * 60);

        return rpm;
    }

    public double getVelocity() {
        return flywheelMotor.getVelocity();
    }

    public boolean setRawPower(double power) {
        if(!isBusy()) {
            flywheelMotor.setPower(power);
            return true;
        }
        return false;
    }



    private void setPIDFCoefficients(PIDFCoefficients coefficients) {
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d, coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        ));
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return rpm * MOTOR_TICKS_PER_REV / MOTOR_GEAR_RATIO / 60;
    }

    //Math probably inaccurate, will need calibration curve
//    public void setRPMWithDistance(double horizontalDistance){
//        double linearVelocity = horizontalDistance / Math.cos(shooterAngle) * Math.sqrt(2 * (shooterHeight - goalHeight - Math.tan(shooterAngle))
//                * (horizontalDistance) / 9.8);
//
//        double radiansPerSecond = linearVelocity / wheelRadius;
//        double rpm = radiansPerSecond * 60 / (2*Math.PI);
//    }

    public static double getMotorVelocityF() {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 * 60.0 / (MOTOR_MAX_RPM * MOTOR_TICKS_PER_REV);
    }

}

