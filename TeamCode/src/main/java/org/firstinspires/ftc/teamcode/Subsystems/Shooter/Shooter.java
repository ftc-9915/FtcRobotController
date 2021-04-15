package org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.SubsystemBase;

public class Shooter implements SubsystemBase {

    //Subsystem Components
    public Hopper hopper;
    public Flywheel flywheel;

    //Subsystem State
    public enum Mode {
        IDLE,
        RAMP_UP_FLYWHEEL,
        SHOOT_RING,
        RETRACT_SERVO
    }
    private ElapsedTime timer;
    private Mode mode;
    private int rings;

    //Logging
    private FtcDashboard dashboard;

    public Shooter (HardwareMap hardwareMap) {
        hopper = new Hopper(hardwareMap);
        flywheel = new Flywheel(hardwareMap);

        rings = 0;
        timer = new ElapsedTime();
        mode = Mode.IDLE;
    }

    public void shootRings(int ringsToShoot, double  rpm) {
        rings = ringsToShoot;
        timer.reset();
        flywheel.setRPM(rpm);
        hopper.setLiftUpPos();
        mode = Mode.RAMP_UP_FLYWHEEL;
    }

    public void shootRings(int ringsToShoot) {
        rings = ringsToShoot;
        timer.reset();
        flywheel.setRPM(flywheel.activeTargetRPM);
        hopper.setLiftUpPos();
        mode = Mode.RAMP_UP_FLYWHEEL;
    }

    public void cancelCommand() {
        flywheel.setRPM(0);
        mode = Mode.IDLE;
    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("mode", mode);
        packet.put("RPM", flywheel.getRPM());
        packet.put("Target RPM", flywheel.activeTargetRPM);
        packet.put("Upper Bound",  4000);
        packet.put("Lower  Bound", 0);

        switch (mode) {
            case IDLE:
                //do nothing
                break;
            case RAMP_UP_FLYWHEEL:
                //wait until flywheel is at target rpm, then shoot ring
                if (flywheel.atTargetRPM() || timer.seconds() > 3) {
                    hopper.setPushInPos();
                    timer.reset();
                    rings--;
                    mode = Mode.SHOOT_RING;
                }
                break;
            case SHOOT_RING:
                //wait before retracting servo
                if (timer.seconds() > 0.3) {
                    hopper.setPushOutPos();
                    timer.reset();
                    mode = Mode.RETRACT_SERVO;
                }
                break;
            case RETRACT_SERVO:
                if (timer.seconds() > 0.3) {
                    if (rings > 0)
                        mode = Mode.RAMP_UP_FLYWHEEL;
                    else {
                        flywheel.setRPM(0);
                        mode = Mode.IDLE;
                    }
                }
                break;
        }

    }

@Override
public boolean isBusy() {
        return mode != Mode.IDLE;
    }


}
