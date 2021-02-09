package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

@Config
public class Hopper implements SubsystemBase {


    //Subsystem Components
    private Servo liftServo;
    private Servo pushServo;

    //Subsystem Component Names
    private String liftName = "liftServo";
    private String pushName = "liftServo";

    //Subsystem Constants
    static final double LIFT_UP_POS = 0.50;
    static final double LIFT_DOWN_POS = 0.75;

    static final double PUSH_OUT_POS = 0.70;
    static final double PUSH_IN_POS = 0.52;

    //Subsystem State
    enum LiftMode {
        LIFT_UP,
        LIFT_DOWN
    }

    enum PushMode {
        PUSH_IN,
        PUSH_OUT
    }

    LiftMode liftMode = LiftMode.LIFT_DOWN;
    PushMode pushMode = PushMode.PUSH_OUT;



    public Hopper (HardwareMap hardwareMap) {
        liftServo = hardwareMap.servo.get(liftName);
        pushServo = hardwareMap.servo.get(pushName);

        liftServo.setPosition(LIFT_DOWN_POS);
        pushServo.setPosition(PUSH_OUT_POS);
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

    public boolean setPushOutPos(){
        pushServo.setPosition(PUSH_OUT_POS);

        if (pushServo.getPosition() == PUSH_OUT_POS){
            PushMode pushMode = PushMode.PUSH_OUT;
            return true;
        }
        return false;
    }

    public boolean setPushInPos(){
        pushServo.setPosition(PUSH_IN_POS);

        if (pushServo.getPosition() == PUSH_IN_POS){
            PushMode pushMode = PushMode.PUSH_IN;
            return true;
        }
        return false;
    }

    public boolean togglePushInOut() {
        if (pushMode == PushMode.PUSH_OUT) {
            return setPushInPos();
        }
        return setPushOutPos();
    }


    @Override
    public boolean isBusy() {
        return false;
    }
}
