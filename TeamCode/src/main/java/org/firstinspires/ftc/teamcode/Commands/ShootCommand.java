package org.firstinspires.ftc.teamcode.Commands;

import android.text.format.Time;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;

import java.util.Timer;

public class ShootCommand extends Command {

    //constants
    public static double RPM_FORGIVENESS = 200;
    public static double TIMEOUT = 8;


    //Actively updated variables
    public double targetRpm;
    public int rings;

    //Inner Subsystems
    Flywheel flywheel;
    Hopper hopper;

    public static boolean shootSyncCommand(int rings, double targetRpm, Flywheel flywheel, Hopper hopper){

        ShootCommand command = new ShootCommand(3, targetRpm, flywheel, hopper);
        while (!command.isFinished()) {
            command.update();
        }

        return true;

    }


    //asynchronous command when paired with update() function to shoot rings
    public ShootCommand(int rings, double targetRpm, Flywheel flywheel, Hopper hopper) {
        //set isFinished to false
        super();

        //set rpm speed and hopper
        flywheel.setRPM(targetRpm);
        hopper.setLiftUpPos();

        this.targetRpm = targetRpm;
        this.rings = rings;
        this.flywheel = flywheel;
        this.hopper = hopper;
    }


    //checks on every call if there are rings ready to shoot, sets command as finished if no rings are left
    public void update() {
        if (rings > 0){
            if (UtilMethods.inRange(targetRpm, targetRpm - RPM_FORGIVENESS, targetRpm + RPM_FORGIVENESS)){
                hopper.setPushInPos();
                UtilMethods.sleep(500);
                hopper.setPushOutPos();
                rings--;
            }
        }

        else {
           isFinished = true;
        }
    }




}
