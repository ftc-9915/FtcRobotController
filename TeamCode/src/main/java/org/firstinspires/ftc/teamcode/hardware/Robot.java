package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    public boolean encodersReseted = false;

    // --- Robot Geometry --- //
    // Wheels
    double wheelDiameter = 4;
    double wheelInchesPerRotation = Math.PI * wheelDiameter;
    int motorTicksPerRotation = 1120;
    double gearRatioMotorToWheel = 2/1;
    // double type for higher accuracy when multiplying by distanceInch in driveForward() method
    public double robotTicksPerInch = motorTicksPerRotation / (gearRatioMotorToWheel * wheelInchesPerRotation);

    // -- Constants -- //
    static final int ARM_INCREMENT = 3;
    static final double CLAW_OPEN_POS = 0.7;
    static final double CLAW_CLOSE_POS = 0.15;

    // TODO: test these and edit with accurate values
    public static final double FEEDER_SERVO_RESET_POS = 1.0;
    public static final double FEEDER_SERVO_PUSH_POS = 0.0;
    public static final double TILT_SERVO_DOWN = 1.0;
    public static final double TILT_SERVO_UP = 0.0;


    // -- Motors -- //
    public DcMotor armMotor;
    public DcMotor launcherMotor;
    public DcMotor collectorMotor;

    // Chassis Motors
    public DcMotor chassisLeftFront;
    public DcMotor chassisLeftBack;
    public DcMotor chassisRightFront;
    public DcMotor chassisRightBack;

    // -- Servos -- //
    public Servo feederServo;
    public Servo tiltServo;
    public Servo clawServo;

    /*
     * Robot init() method for driving to a position. Use if you want to use the drive(), strafe(), or driveMecanum() methods
     */
    public void initForRunToPosition(HardwareMap hardwareMap) {
        this.init(hardwareMap);
        this.setModeChassisMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Robot init() method; use if you want to have the chassis motors run without a specified target position
     */
    public void initRegular(HardwareMap hardwareMap) {
        this.init(hardwareMap);
        this.setModeChassisMotors(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * This init() method is called by the other init() methods. Do not call this method outside of this class.
     */
    private void init(HardwareMap hardwareMap) {
        // -- Chassis -- //
        this.chassisLeftFront = hardwareMap.dcMotor.get("leftFront");
        this.chassisRightFront = hardwareMap.dcMotor.get("rightFront");
        this.chassisRightBack = hardwareMap.dcMotor.get("rightBack");
        this.chassisLeftBack = hardwareMap.dcMotor.get("leftBack");
        this.chassisLeftFront.setDirection(DcMotor.Direction.REVERSE);
        this.chassisLeftBack.setDirection(DcMotor.Direction.REVERSE);

        // -- Launcher -- //
//        this.launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
//        // Make the motor's speed constant using encoders
//        this.launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        this.feederServo = hardwareMap.servo.get("feederServo");
//        this.feederServo.setPosition(FEEDER_SERVO_RESET_POS);

        // -- Collector and Delivery -- //
//        this.collectorMotor = hardwareMap.dcMotor.get("collectorMotor");
//        this.tiltServo = hardwareMap.servo.get("tiltServo");
//        this.tiltServo.setPosition(TILT_SERVO_DOWN);

        // -- Arm -- //
        this.armMotor = hardwareMap.dcMotor.get("armMotor");
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setTargetPosition(0);
        this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.armMotor.setPower(1.0);

        this.clawServo = hardwareMap.servo.get("clawServo");
        this.clawServo.setPosition(CLAW_CLOSE_POS);
    }

    /*
     * Set the runMode of all the chassis motors
     * @param runMode: the runMode the chassis motors should be set to
     */
    public void setModeChassisMotors(DcMotor.RunMode runMode) {
        this.chassisLeftFront.setMode(runMode);
        this.chassisRightFront.setMode(runMode);
        this.chassisLeftBack.setMode(runMode);
        this.chassisRightBack.setMode(runMode);
    }

    public void resetChassisEncoders() {
        if(this.chassisLeftFront.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            setModeChassisMotors(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    
    /*
     * Strafes right if distanceInch is positive; strafes left if distanceInch is negative
     * @param power: the power set to the motors, must be positive (signs are determined within the method)
     * @param distanceInch: distance for robot to strafe in inches
     * @return whether the robot has reached that distance
     */
    public boolean strafe(double power, double distanceInch) {
        if(!encodersReseted) {
            this.resetChassisEncoders();
            encodersReseted = true;
        }
        // Getting the sign of the argument to determine which direction we're strafing
        int direction = (int)Math.signum(distanceInch);

        this.chassisLeftFront.setTargetPosition((int)(distanceInch * robotTicksPerInch));
        this.chassisRightFront.setTargetPosition(-(int)(distanceInch * robotTicksPerInch));
        this.chassisLeftBack.setTargetPosition(-(int)(distanceInch * robotTicksPerInch));
        this.chassisRightBack.setTargetPosition((int)(distanceInch * robotTicksPerInch));

        this.chassisLeftFront.setPower(direction * power);
        this.chassisRightFront.setPower(-direction * power);
        this.chassisLeftBack.setPower(-direction * power);
        this.chassisRightBack.setPower(direction * power);

        setModeChassisMotors(DcMotor.RunMode.RUN_TO_POSITION);

        if (!this.chassisLeftFront.isBusy() || !this.chassisLeftBack.isBusy() || !this.chassisRightFront.isBusy() || !this.chassisRightBack.isBusy()) {
            encodersReseted = false;
            return true;
        } else {
            return false;
        }
    }

    /*
     * Stop the robot by setting the power of all motors to 0.0
     */
    public void stop() {
        this.chassisLeftBack.setPower(0.0);
        this.chassisRightBack.setPower(0.0);
        this.chassisLeftFront.setPower(0.0);
        this.chassisRightFront.setPower(0.0);
    }

}
