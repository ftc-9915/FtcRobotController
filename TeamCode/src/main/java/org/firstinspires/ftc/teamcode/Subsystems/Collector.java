package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@Config
public class Collector implements SubsystemBase {



    //Subsystem Components
    private DcMotor collectorMotor;
    public Servo ringBlockServo;
    public Servo ringGuardServo;

    //Subsystem Component Names
    private String collectorName = "collectorMotor";
    private String ringBlockServoName = "ringBlockServo";
    private String ringGuardServoName = "ringGuardServo";

    //Subsystem Constants
    public static double COLLECTOR_ON_SPEED = 1.0;
    public static double COLLECTOR_REVERSE_SPEED = -1.0;

    public static double RAISE_RING_BLOCK = 0.625;
    public static double LOWER_RING_BLOCK = 0.1;

    public static double RAISE_RING_GUARD = 0.25;
    public static double LOWER_RING_GUARD = 0.73;

    //Subsystem State
    enum Mode {
        COLLECTOR_ON,
        COLLECTOR_OFF
    }

    Mode currentMode = Mode.COLLECTOR_OFF;


    public Collector(HardwareMap hardwareMap){
        collectorMotor = hardwareMap.dcMotor.get(collectorName);
        collectorMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ringBlockServo = hardwareMap.servo.get(ringBlockServoName);
        ringBlockServo.setPosition(RAISE_RING_BLOCK);

        ringGuardServo = hardwareMap.servo.get(ringGuardServoName);
        ringGuardServo.setPosition(RAISE_RING_GUARD);
    }

    public boolean setRawPower(double power){
        if (!isBusy()){
            collectorMotor.setPower(power);

            currentMode = Mode.COLLECTOR_ON;
            if(power == 0){
                currentMode = Mode.COLLECTOR_OFF;
            }

            return true;

        }
        return false;
    }

    public boolean turnCollectorOnWithRingGuard(){
        lowerRingGuard();
        return setRawPower(COLLECTOR_ON_SPEED);
    }

    public boolean turnCollectorOn(){
        return setRawPower(COLLECTOR_ON_SPEED);
    }

    public boolean turnCollectorOff(){
        raiseRingGuard();
        return setRawPower(0);
    }

    public boolean turnCollectorReverse() { return setRawPower(COLLECTOR_REVERSE_SPEED);}

    public boolean toggleCollectorOnOff(){
        if (currentMode == Mode.COLLECTOR_OFF) {
            return turnCollectorOnWithRingGuard();
        }
        return turnCollectorOff();
    }

    public void lowerRingBlock() {
        ringBlockServo.setPosition(LOWER_RING_BLOCK);
    }

    public void raiseRingBlock() {
        ringBlockServo.setPosition(RAISE_RING_BLOCK);
    }

    public void raiseRingGuard() { ringGuardServo.setPosition(RAISE_RING_GUARD); }

    public void lowerRingGuard() { ringGuardServo.setPosition(LOWER_RING_GUARD); }

    @Override
    public boolean isBusy() {
        return collectorMotor.isBusy();
    }

}
