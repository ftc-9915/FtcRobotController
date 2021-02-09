package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

public abstract class AutonomousPath {

    MecanumDrivebase drive;
    WobbleArm wobbleArm;
    Shooter shooter;
    Collector collector;

    public AutonomousPath(MecanumDrivebase drive, WobbleArm wobbleArm, Shooter shooter, Collector collector){
        this.drive = drive;
        this.wobbleArm = wobbleArm;
        this.shooter = shooter;
        this.collector = collector;
    }


    abstract boolean followPath();
}
