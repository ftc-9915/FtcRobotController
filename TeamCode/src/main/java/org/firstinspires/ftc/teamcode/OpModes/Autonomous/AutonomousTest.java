package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.CoordinateConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Autonomous Test", group = "test")
public class AutonomousTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Starting Position
        drive.setPoseEstimate(CoordinateConstants.START_POS_BLUE_2);

        Pose2d shootingPose = new Pose2d(0, 24, Math.toRadians(0.0));
        Pose2d placeGoalPose = new Pose2d(22, 24, Math.toRadians(0.0));
        Pose2d pickUpGoalPose = new Pose2d(-63,63, Math.toRadians(180.0));



        waitForStart();

        if (isStopRequested()) return;



        Trajectory goToShootingPose = drive.trajectoryBuilder(CoordinateConstants.START_POS_BLUE_2)
                .splineTo(shootingPose.vec(), shootingPose.getHeading())
                .build();

        Trajectory goToPlaceGoalPose = drive.trajectoryBuilder(placeGoalPose)
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .build();

        Trajectory goToPickUpGoalPose = drive.trajectoryBuilder(pickUpGoalPose)
                .splineTo(pickUpGoalPose.vec(), pickUpGoalPose.getHeading())
                .build();

        drive.followTrajectory(goToShootingPose);

        sleep(1000);

        drive.followTrajectory(goToPlaceGoalPose);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose);

        sleep(1000);

        while(!isStopRequested() && opModeIsActive());

    }
}


