package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

import java.util.Arrays;

@Config
public class AutonomousPathB extends AutonomousPath {


    public static int goalX = -30;
    public static int goalY = 59;

    Pose2d shootingPose = new Pose2d(0, 24, Math.toRadians(0.0));
    Pose2d placeGoalPose = new Pose2d(22, 24, Math.toRadians(0.0));
    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(27, 57, Math.toRadians(0.0));
    Pose2d placeSecondGoalPose2 = new Pose2d(20, 30, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(12, 27, Math.toRadians(0.0));



    public AutonomousPathB(MecanumDrivebase drive, WobbleArm wobbleArm, Shooter shooter, Collector collector) {
        super(drive, wobbleArm, shooter, collector);
    }

    public boolean followPath() {
        Trajectory goToShootingPose = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPose.vec(), shootingPose.getHeading())
                .build();

        Trajectory goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPose.end())
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .addDisplacementMarker(() -> {
                    wobbleArm.placeGoalAuto();
                })
                .build();

        Trajectory goToPickUpGoalPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
                .addDisplacementMarker(()->{
                    wobbleArm.liftArmAuto();
                })
                .lineToLinearHeading(pickUpGoalPose1)
                .build();

        Trajectory goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
                .lineToConstantHeading(pickUpGoalPose2.vec(), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    wobbleArm.pickupGoalAuto();
                })
                .build();


        Trajectory goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .addDisplacementMarker(() -> {
                    wobbleArm.liftArmAuto();
                })
                .lineToSplineHeading(placeSecondGoalPose1)
                .build();

        Trajectory goToPlaceSecondGoalPart2 = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(placeSecondGoalPose2.vec())
                .addDisplacementMarker(()->{
                    wobbleArm.placeGoalAuto();
                })
                .build();

        Trajectory goToParkingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart2.end())
                .lineToConstantHeading(parkPose.vec())
                .build();


        drive.followTrajectory(goToShootingPose);

        drive.followTrajectory(goToPlaceGoalPose);

        UtilMethods.sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);
        drive.followTrajectory(goToPickUpGoalPose2);

        UtilMethods.sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);
        drive.followTrajectory(goToPlaceSecondGoalPart2);
        drive.followTrajectory(goToParkingPose);

        UtilMethods.sleep(1000);

        wobbleArm.liftArmAuto();

        return true;
    }

}
