package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

@Config
public class Hopper implements SubsystemBase {


    //Subsystem Components
    public Servo liftServo;
    public Servo pushServo;

    //Subsystem Component Names
    public String liftName = "liftServo";
    public String pushName = "pushServo";

    //Subsystem Constants
    public static  double LIFT_UP_POS = 0.49;
    public static  double LIFT_DOWN_POS = 0.75;

    public static  double PUSH_OUT_POS = 0.7;
    public static double PUSH_INIT_POS = 0.7;
    public static  double PUSH_IN_POS = 0.54;

    //Subsystem State
    public enum LiftMode {
        LIFT_UP,
        LIFT_DOWN
    }

    public enum PushMode {
        PUSH_IN,
        PUSH_OUT
    }

    public LiftMode liftMode = LiftMode.LIFT_DOWN;
    public PushMode pushMode = PushMode.PUSH_OUT;



    public Hopper (HardwareMap hardwareMap) {
        liftServo = hardwareMap.servo.get(liftName);
        pushServo = hardwareMap.servo.get(pushName);

        liftServo.setPosition(LIFT_UP_POS);
        pushServo.setPosition(PUSH_INIT_POS);
    }

    public boolean setLiftUpPos(){
        liftServo.setPosition(LIFT_UP_POS);

        if (liftServo.getPosition() == LIFT_UP_POS){
            LiftMode liftMode = LiftMode.LIFT_UP;
            return true;
        }
        return false;
    }

    public boolean setLiftDownPos(){
        liftServo.setPosition(LIFT_DOWN_POS);

        if (liftServo.getPosition() == LIFT_DOWN_POS){
            LiftMode liftMode = LiftMode.LIFT_DOWN;
            return true;
        }

        return false;
    }

    public boolean toggleLiftUpDown() {
        if (liftMode == LiftMode.LIFT_DOWN) {
            return setLiftUpPos();
        }
        return setLiftDownPos();
    }

    public void setPushOutPos(){
        pushServo.setPosition(PUSH_OUT_POS);
        pushMode = PushMode.PUSH_OUT;
    }

    public void setPushInPos(){
        pushServo.setPosition(PUSH_IN_POS);
        pushMode = PushMode.PUSH_IN;
    }


    public LiftMode getLiftMode() {
        return liftMode;
    }

    public PushMode getPushMode() {
        return pushMode;
    }


    @Override
    public boolean isBusy() {
        return false;
    }


}
