package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;

import static org.firstinspires.ftc.teamcode.Common.UtilMethods.ensureRange;
import static org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveConstants.RUN_USING_ENCODER;

//TODO Add Second Wobble Goal Claw
@Config
public class WobbleArm implements SubsystemBase {

    //Command  Logic Variables
    public enum Mode {
        IDLE, // do nothing
        LOWER_ARM, //open claw and lower arm
        LIFT_ARM,  // close claw lift arm and hold
        PLACE_GOAL, //lower arm and open servo
        STORE_ARM,  //close claw, lift arm back to storage position
    }

    private FtcDashboard dashboard;
    private ElapsedTime timer;

    private Mode mode;

    private int  armTargetPosition;

    //Subsystem Components
    public DcMotorEx armMotor;
    public Servo clawServoLeft;
    public Servo clawServoRight;

    //Subsystem Component Names
    public String armName = "armMotor";
    public String clawServoLeftName = "clawServo";
    public String clawServoRightName = "clawServo2";

    //Subsystem Constants
    public static PIDFCoefficients ARM_PID;     //can implement pid if needed

    public static double POSITION_FORGIVENESS  = 0.05; // percentage out of 1

    public static int ARM_POS_PLACE_GOAL = -450;
    public static int ARM_POS_PICKUP_GOAL = -480;
    public static int ARM_POS_LIFT_ARM = -200;
    public static int ARM_POS_OVER_WALL = -350;

    public static int ARM_UPPER_LIMIT = 0;
    public static int ARM_LOWER_LIMIT = -550;

    public static double DEFAULT_ARM_POWER = 0.4;

    public static double LEFT_CLAW_OPEN_POS = 0.4;
    public static double LEFT_CLAW_CLOSE_POS = 0.125;

    public static double RIGHT_CLAW_OPEN_POS = 0.3;
    public static double RIGHT_CLAW_CLOSE_POS = 0.55;

    public WobbleArm(HardwareMap hardwareMap){

        timer = new ElapsedTime();
        timer.reset();

        mode = Mode.IDLE;

        armMotor = hardwareMap.get(DcMotorEx.class, armName);
        clawServoLeft = hardwareMap.servo.get(clawServoLeftName);
        clawServoRight = hardwareMap.servo.get(clawServoRightName);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setArmPos(0);

        //in case PID is added
        if (RUN_USING_ENCODER && ARM_PID != null) {
            armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ARM_PID);
        }

        clawServoLeft.setPosition(LEFT_CLAW_CLOSE_POS);
        clawServoRight.setPosition(RIGHT_CLAW_CLOSE_POS);
    }

    // commands

    /**
     *  Opens claw and lowers arm to pickup goal position.
     *  Intended to be used before picking up a goal.
     */
    public void openClawLowerArm() {
        openClaw();
        timer.reset();
        mode = Mode.LOWER_ARM;
    }
    /**
     *  Closes the claw and moves arm to the lift arm position.
     *  Intended to be used to pickup goal.
     */
    public void closeClawLiftArm() {
        closeClaw();
        timer.reset();
        mode = Mode.LIFT_ARM;
    }

    /**
     *  Lowers the arm, opens the claw, then lifts the arm.
     *  Intended to be used to place a goal in auto.
     */
    public void placeGoal() {
        timer.reset();
        setArmPos(ARM_POS_PLACE_GOAL);
        mode = Mode.PLACE_GOAL;
    }

    /**
     *  Closes the claw, sets arm to storage position, and turns arm off.
     */
    public void storeArm() {
        timer.reset();
        closeClaw();
        setArmPos(0);
        mode = Mode.STORE_ARM;
    }

    //utility methods
    public void setArmPos(int position) {
        armTargetPosition = position;

        int normalizedPosition = UtilMethods.ensureRange(position, ARM_LOWER_LIMIT, ARM_UPPER_LIMIT);
        armMotor.setTargetPosition(normalizedPosition);
        armMotor.setPower(DEFAULT_ARM_POWER);
    }

    public void setArmPos(int position, double power) {
        armTargetPosition = position;

        int normalizedPosition = UtilMethods.ensureRange(position, ARM_LOWER_LIMIT, ARM_UPPER_LIMIT);
        armMotor.setTargetPosition(normalizedPosition);
        armMotor.setPower(power);
    }

    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public boolean isArmAtPosition() {
        return UtilMethods.atTarget(armTargetPosition, getArmPosition(), armTargetPosition * POSITION_FORGIVENESS );
    }

    public void  openClaw() {
        clawServoLeft.setPosition(LEFT_CLAW_OPEN_POS);
        clawServoRight.setPosition(RIGHT_CLAW_OPEN_POS);
    }

    public void closeClaw() {
        clawServoLeft.setPosition(LEFT_CLAW_CLOSE_POS);
        clawServoRight.setPosition(RIGHT_CLAW_CLOSE_POS);
    }

    public void cancelCommand() {
        mode = Mode.IDLE;
    }

    @Override
    public boolean isBusy() {
        return mode !=  Mode.IDLE;
    }

    public void  update() {
        switch (mode) {
            case IDLE:
                //do nothing
                break;
            case LIFT_ARM:
                //wait for claw action before lifting arm
                if(timer.seconds() > 0.5) {
                    setArmPos(ARM_POS_LIFT_ARM);
                    //hold arm with extra power
                    if (isArmAtPosition() || timer.seconds() > 1) {
                        setArmPos(armTargetPosition, 0.6);
                        mode = Mode.IDLE;
                    }
                }
                break;
            case STORE_ARM:
                if (isArmAtPosition() || timer.seconds() > 2) {
                    armMotor.setPower(0);
                    mode = Mode.IDLE;
                }
                break;
            case PLACE_GOAL:
              if  (isArmAtPosition() || timer.seconds() > 1) {
                    openClaw();
                    timer.reset();
                    setArmPos(ARM_POS_LIFT_ARM);
                    mode = Mode.LIFT_ARM;
              }
                break;
            case LOWER_ARM:
                if  (timer.seconds() > 0.5) {
                    setArmPos(ARM_POS_PICKUP_GOAL);
                    if (isArmAtPosition() || timer.seconds() > 1) {
                        setArmPos(armTargetPosition, 0.6);
                        mode = Mode.IDLE;
                    }
                }
        }
    }

}
