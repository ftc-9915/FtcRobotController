package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

//TODO Add Second Wobble Goal Claw
@Config
public class WobbleArm implements SubsystemBase {

    //Subsystem Components
    private DcMotor armMotor;
    private Servo clawServo;

    //Subsystem Component Names
    private String armName = "armMotor";
    private String clawName = "clawServo";

    //Subsystem Constants
    private static int ARM_POS_PLACE_GOAL = -525;
    private static int ARM_POS_PICKUP_GOAL = -510;
    private static int ARM_POS_LIFT_ARM = -200;

    private static double DEFAULT_ARM_POWER = 0.3;

    private static double CLAW_OPEN_POS = 0.7;
    private static double CLAW_CLOSE_POS = 0.15;





    public WobbleArm(HardwareMap hardwareMap){
        armMotor = hardwareMap.dcMotor.get(armName); // this stuff is going to be replaced by robot class later
        clawServo = hardwareMap.servo.get(clawName);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawServo.setPosition(CLAW_CLOSE_POS);
    }


    @Override
    public boolean isBusy() {
        return armMotor.isBusy();
    }

    //teleop methods
    public boolean liftArm() {
        if(!armMotor.isBusy()){
            armMotor.setTargetPosition(ARM_POS_LIFT_ARM);
            armMotor.setPower(DEFAULT_ARM_POWER);

            return true;
        }
        return false;
    }

    public boolean lowerArm() {
        if(!armMotor.isBusy()){
            armMotor.setTargetPosition(ARM_POS_PICKUP_GOAL);
            armMotor.setPower(DEFAULT_ARM_POWER);

            return true;
        }
        return false;
    }

    public boolean openClaw() {
        clawServo.setPosition(CLAW_OPEN_POS);
        if (clawServo.getPosition() == CLAW_OPEN_POS) {
            return true;
        }
        return false;
    }

    public boolean closeClaw() {
        clawServo.setPosition(CLAW_CLOSE_POS);
        return true;
    }


    //auto methods
    public boolean pickupGoalAuto() {
        if (!isBusy()) {
            UtilMethods.sleep(200);
            armMotor.setTargetPosition(ARM_POS_PICKUP_GOAL);
            armMotor.setPower(DEFAULT_ARM_POWER);
            UtilMethods.sleep(500);
            clawServo.setPosition(CLAW_CLOSE_POS);

            return true;
        }

        return false;

    }

    public boolean placeGoalAuto() {
        if (!isBusy()) {
            armMotor.setTargetPosition(ARM_POS_PLACE_GOAL);
            armMotor.setPower(DEFAULT_ARM_POWER);
            UtilMethods.sleep(600);
            clawServo.setPosition(CLAW_OPEN_POS);
            UtilMethods.sleep(500);

            return true;
        }

        return false;

    }

    public boolean liftArmAuto() {

        if (!isBusy()) {
            armMotor.setTargetPosition(ARM_POS_LIFT_ARM);
            armMotor.setPower(DEFAULT_ARM_POWER);
            return true;
        }

        return false;

    }


}
