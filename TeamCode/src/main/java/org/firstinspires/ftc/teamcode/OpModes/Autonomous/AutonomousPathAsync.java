package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

public abstract class AutonomousPathAsync {

    MecanumDrivebase drive;
    WobbleArm wobbleArm;
    Collector collector;
    Shooter shooter;

    public AutonomousPathAsync(MecanumDrivebase drive, WobbleArm wobbleArm, Shooter shooter, Collector collector){
        this.drive = drive;
        this.wobbleArm = wobbleArm;
        this.collector = collector;
        this.shooter = shooter;
    }


    abstract void followPathAsync(Telemetry telemetry);
}
