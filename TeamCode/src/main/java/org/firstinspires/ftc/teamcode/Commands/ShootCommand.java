package org.firstinspires.ftc.teamcode.Commands;

import android.text.format.Time;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousPathBAsync;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;

import java.util.Timer;

public class ShootCommand  {

    //constants
    public static double RPM_FORGIVENESS = 200;
    public static double TIMEOUT = 8;


    //Actively updated variables
    public double targetRpm;
    public int rings;

    //Inner Subsystems
    Flywheel flywheel;
    Hopper hopper;


    //returns true if a ring was shot, return false otherwise
    public static boolean shootCommandAsync(double targetRpm, double RPM_FORGIVENESS,  ElapsedTime timer, Flywheel flywheel, Hopper hopper) {
        hopper.setLiftUpPos();
        if (UtilMethods.inRange(flywheel.getRPM(), targetRpm - RPM_FORGIVENESS, targetRpm + RPM_FORGIVENESS)
                &&  hopper.getPushMode() == Hopper.PushMode.PUSH_OUT && timer.seconds() > 2) {
            hopper.setPushInPos();
            timer.reset();
        }
        if (hopper.getPushMode() == Hopper.PushMode.PUSH_IN && timer.seconds() > 2) {
            hopper.setPushOutPos();
            timer.reset();
            return true;
        }
        return false;

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
    public void update() {
        if (rings > 0){
            if (UtilMethods.inRange(targetRpm, targetRpm - RPM_FORGIVENESS, targetRpm + RPM_FORGIVENESS)){
                hopper.setPushInPos();
                UtilMethods.sleep(500);
                hopper.setPushOutPos();
                rings--;
            }
        }

    }




}
