package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;


@Config
public class Collector implements SubsystemBase {



    //Subsystem Components
    private DcMotor collectorMotor;
    public Servo ringBlockServo;

    //Subsystem Component Names
    private String collectorName = "collectorMotor";
    private String ringBlockServoName = "ringBlockServo";

    //Subsystem Constants
    public static double COLLECTOR_ON_SPEED = 1.0;
    public static double COLLECTOR_REVERSE_SPEED = -1.0;

    public static double RAISE_RING_BLOCK = 0.625;
    public static double LOWER_RING_BLOCK = 0.1;

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

    public boolean turnCollectorOn(){
        return setRawPower(COLLECTOR_ON_SPEED);
    }

    public boolean turnCollectorOff(){
        return setRawPower(0);
    }

    public boolean turnCollectorReverse() { return setRawPower(COLLECTOR_REVERSE_SPEED);}

    public boolean toggleCollectorOnOff(){
        if (currentMode == Mode.COLLECTOR_OFF) {
            return turnCollectorOn();
        }
        return turnCollectorOff();
    }

    public void lowerRingBlock() {
        ringBlockServo.setPosition(LOWER_RING_BLOCK);
    }

    public void raiseRingBlock() {
        ringBlockServo.setPosition(RAISE_RING_BLOCK);
    }

    @Override
    public boolean isBusy() {
        return collectorMotor.isBusy();
    }

}
