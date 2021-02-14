package org.firstinspires.ftc.teamcode.Commands;

import android.text.format.Time;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;

public class ShootCommand extends Command {

    //constants
    public static double RPM_FORGIVENESS = 100;
    public static double TIMEOUT = 8;


    //Actively updated variables
    public double targetRpm;
    public int rings;

    //Inner Subsystems
    Flywheel flywheel;
    Hopper hopper;

    public static boolean shootSyncCommand(int rings, double targetRpm, Flywheel flywheel, Hopper hopper){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        flywheel.setRPM(targetRpm);
        hopper.setLiftUpPos();

        //Loop until rings are all shot or reach timeout
        while (rings > 0 || timer.seconds() < TIMEOUT) {
            if (UtilMethods.inRange(targetRpm, targetRpm - RPM_FORGIVENESS, targetRpm + RPM_FORGIVENESS)){
                hopper.setPushInPos();
                hopper.setPushOutPos();
                rings--;
            }
        }

        return true;
    }

    //asynchronous command when paired with update() function to shoot rings
    public ShootCommand(int rings, double targetRpm, Flywheel flywheel, Hopper hopper) {
        //set rpm speed and hopper
        flywheel.setRPM(targetRpm);
        hopper.setLiftUpPos();

        this.targetRpm = targetRpm;
        this.rings = rings;
        this.flywheel = flywheel;
        this.hopper = hopper;
    }


    //checks on every call if there are rings ready to shoot, sets command as finished if no rings are left
    @Override
    public void update() {
        if (rings > 0){
            if (UtilMethods.inRange(targetRpm, targetRpm - RPM_FORGIVENESS, targetRpm + RPM_FORGIVENESS)){
                hopper.setPushInPos();
                hopper.setPushOutPos();
                rings--;
            }
        }

        else {
           isFinished = true;
        }
    }


}
