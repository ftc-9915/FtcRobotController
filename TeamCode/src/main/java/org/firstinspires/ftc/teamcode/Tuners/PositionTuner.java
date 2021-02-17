package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.Camera;

@Config
@TeleOp(group = "drive")
public class PositionTuner extends LinearOpMode {

    public static double flywheelRpm = 0;
    public static double liftServoPosition = 0;
    public static double pushServoPosition = 0;
    public static double leftClawPosition = 0;
    public static double rightClawPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebase drive = new MecanumDrivebase(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Hopper hopper = new Hopper(hardwareMap);
        Collector collector = new Collector(hardwareMap);
        WobbleArm wobbleArm = new WobbleArm(hardwareMap);


        waitForStart();

        while (!isStopRequested()) {

            flywheel.setRPM(flywheelRpm);
            hopper.liftServo.setPosition(liftServoPosition);
            hopper.pushServo.setPosition(pushServoPosition);
            wobbleArm.clawServoLeft.setPosition(leftClawPosition);
            wobbleArm.clawServoRight.setPosition(rightClawPosition);


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();



        }
    }
}