package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Archive;

import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

public abstract class AutonomousPath {

    MecanumDrivebase drive;
    WobbleArm wobbleArm;
    Flywheel flywheel;
    Collector collector;
    Hopper hopper;

    public AutonomousPath(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper){
        this.drive = drive;
        this.wobbleArm = wobbleArm;
        this.flywheel = flywheel;
        this.collector = collector;
        this.hopper = hopper;
    }


    abstract boolean followPath();
}
