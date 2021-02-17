package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;

@Config
@TeleOp(group = "drive")
public class FlywheelTuner extends LinearOpMode {

    public static double RPM = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebase drive = new MecanumDrivebase(hardwareMap);
        drive.setPoseEstimate(PoseLibrary.START_POS_BLUE_2);

        Flywheel flywheel = new Flywheel(hardwareMap);
        Hopper hopper = new Hopper(hardwareMap);


        waitForStart();

        while (!isStopRequested()) {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            Telemetry dashboardTelemetry = dashboard.getTelemetry();

            hopper.setLiftUpPos();

            if (gamepad1.y) {
                hopper.setPushInPos();
            }

            if (gamepad1.x) {
                hopper.setPushOutPos();
            }


            flywheel.setRPM(RPM);

            dashboardTelemetry.addData("Desired RPM", RPM);
            dashboardTelemetry.addData("Current RPM", flywheel.getRPM());
            dashboardTelemetry.update();




        }
    }
}