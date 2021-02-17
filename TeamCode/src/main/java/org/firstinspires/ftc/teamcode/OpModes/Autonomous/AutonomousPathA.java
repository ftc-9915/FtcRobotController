package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;

import org.firstinspires.ftc.teamcode.Commands.ShootCommand;
import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

import java.util.Arrays;


@Config
public class AutonomousPathA extends AutonomousPath{

    //Treakable values for tuning
    public static int goalX = -32;
    public static int goalY = 53;
    public static double shootingPoseAngle = -30.0;
    public static double shootingPoseRPM = 4000;


    // shootingPose not used
    public static Pose2d shootingPose = new Pose2d(0, 24, Math.toRadians(0.0));

    public static Pose2d placeGoalAndShootingPose = new Pose2d(-5, 55, Math.toRadians(shootingPoseAngle));

    public static Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    public static Pose2d placeSecondGoalPose = new Pose2d(0, 57, Math.toRadians(0.0));
    public static Pose2d parkingPose = new Pose2d(10, 30, Math.toRadians(0.0));

    public AutonomousPathA(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper) {
        super(drive, wobbleArm, flywheel, collector, hopper);
    }

    public boolean followPath() {
        Trajectory goToShootingAndPlaceGoalPose = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(placeGoalAndShootingPose.vec(), placeGoalAndShootingPose.getHeading())
                .addDisplacementMarker(() -> {
                    flywheel.setRPM(shootingPoseRPM);
                    wobbleArm.placeGoalAuto();
                    wobbleArm.liftArmAuto();
                    UtilMethods.sleep(500);
                    hopper.setPushInPos();
                    UtilMethods.sleep(200);
                    hopper.setPushOutPos();
                    UtilMethods.sleep(500);
                    hopper.setPushInPos();
                    UtilMethods.sleep(200);
                    hopper.setPushOutPos();
                    UtilMethods.sleep(500);
                    hopper.setPushInPos();
                    UtilMethods.sleep(200);
                    hopper.setPushOutPos();
                })
                .build();

        Trajectory goToPickUpGoalPose1 = drive.trajectoryBuilder(goToShootingAndPlaceGoalPose.end())
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
                .addDisplacementMarker(()->{
                    wobbleArm.liftArmAuto();
                })
                .lineToSplineHeading(placeSecondGoalPose)
                .addDisplacementMarker(() -> {
                    wobbleArm.placeGoalAuto();
                })
                .build();

        Trajectory goToParking = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .addDisplacementMarker(() -> {
                    wobbleArm.liftArmAuto();
                })
                .lineToConstantHeading(parkingPose.vec())
                .build();


        drive.followTrajectory(goToShootingAndPlaceGoalPose);

        UtilMethods.sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);

        UtilMethods.sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose2);

        UtilMethods.sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);

        drive.followTrajectory(goToParking);

        UtilMethods.sleep(1000);

        return true;

    }



}
