package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.util.MathUtil;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.teamcode.Common.UtilMethods;

import static org.firstinspires.ftc.teamcode.Common.UtilMethods.ensureRange;

//TODO Add Second Wobble Goal Claw
@Config
public class WobbleArm implements SubsystemBase {

    //Subsystem Components
    public DcMotorEx armMotor;
    public Servo clawServoLeft;
    public Servo clawServoRight;

    //Subsystem Component Names
    public String armName = "armMotor";
    public String clawServoLeftName = "clawServo";
    public String clawServoRightName = "clawServo2";


    //Subsystem Constants
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
        armMotor = hardwareMap.get(DcMotorEx.class, armName);
        armMotor.setTargetPositionTolerance(25);
        clawServoLeft = hardwareMap.servo.get(clawServoLeftName);
        clawServoRight = hardwareMap.servo.get(clawServoRightName);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawServoLeft.setPosition(LEFT_CLAW_CLOSE_POS);
        clawServoRight.setPosition(RIGHT_CLAW_CLOSE_POS);
    }


    @Override
    public boolean isBusy() {
        return armMotor.isBusy();
    }


    //teleop methods
    public void liftArm() {
            armMotor.setTargetPosition(ARM_POS_LIFT_ARM);
            armMotor.setPower(DEFAULT_ARM_POWER);

    }

    public void placeGoal() {
          armMotor.setTargetPosition(ARM_POS_PLACE_GOAL);
            armMotor.setPower(DEFAULT_ARM_POWER);

    }

    public void pickUpSecondGoal() {
            armMotor.setTargetPosition(ARM_POS_PICKUP_GOAL);
            armMotor.setPower(DEFAULT_ARM_POWER);
    }

    public void setArmPos(int position) {
        int normalizedPosition = UtilMethods.ensureRange(position, ARM_LOWER_LIMIT, ARM_UPPER_LIMIT);
        armMotor.setTargetPosition(normalizedPosition);
        armMotor.setPower(DEFAULT_ARM_POWER);
    }

    public double getArmPosition() {
        return armMotor.getCurrentPosition();
    }

    public boolean openClaw() {
        clawServoLeft.setPosition(LEFT_CLAW_OPEN_POS);
        clawServoRight.setPosition(RIGHT_CLAW_OPEN_POS);

        if (clawServoLeft.getPosition() == LEFT_CLAW_OPEN_POS && clawServoRight.getPosition() == RIGHT_CLAW_OPEN_POS) {
            return true;
        }
        return false;
    }

    public boolean closeClaw() {
        clawServoLeft.setPosition(LEFT_CLAW_CLOSE_POS);
        clawServoRight.setPosition(RIGHT_CLAW_CLOSE_POS);

        return true;
    }


    //auto methods
//    public boolean pickupGoalAuto() {
//        if (!isBusy()) {
//            UtilMethods.sleep(200);
//            armMotor.setTargetPosition(ARM_POS_PICKUP_GOAL);
//            armMotor.setPower(DEFAULT_ARM_POWER);
//            UtilMethods.sleep(500);
//            clawServoLeft.setPosition(LEFT_CLAW_CLOSE_POS);
//            UtilMethods.sleep(500);
//
//            return true;
//        }
//
//        return false;
//
//    }

//    public boolean placeGoalAuto() {
//        if (!isBusy()) {
//            UtilMethods.sleep(200);
//            armMotor.setTargetPosition(ARM_POS_PLACE_GOAL);
//            armMotor.setPower(DEFAULT_ARM_POWER);
//            UtilMethods.sleep(500);
//            clawServoLeft.setPosition(LEFT_CLAW_OPEN_POS);
//            UtilMethods.sleep(500);
//
//            return true;
//        }
//
//        return false;
//
//    }

//    public boolean liftArmAuto() {
//
//        if (!isBusy()) {
//            UtilMethods.sleep(200);
//            armMotor.setTargetPosition(ARM_POS_LIFT_ARM);
//            armMotor.setPower(DEFAULT_ARM_POWER);
//            UtilMethods.sleep(500);
//            return true;
//        }
//
//        return false;
//
//    }


}
