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

import static org.firstinspires.ftc.teamcode.Common.UtilMethods.sleep;


@Config
public class AutonomousPathC extends AutonomousPath{

    public static int goalX = -35;
    public static int goalY = 55;
    public static double shootingPoseAngle = 7;
    public static double shootingPoseRPM = 3350;

    Pose2d shootingPosePt1 = new Pose2d (-24,21);
    Pose2d shootingPosePt2 = new Pose2d(6.8066, 26.37388, Math.toRadians(shootingPoseAngle));
    Pose2d placeGoalPose = new Pose2d(48, 55, Math.toRadians(0.0));
    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(48, 57, Math.toRadians(0.0));
    Pose2d parkingPose = new Pose2d(12, 57, Math.toRadians(0.0));

    public AutonomousPathC(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper) {
        super(drive, wobbleArm, flywheel, collector, hopper);
    }

    public boolean followPath() {
        Trajectory goToShootingPose = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
                .addDisplacementMarker(() -> {
                    flywheel.setRPM(shootingPoseRPM);
                    UtilMethods.sleep(1000);
                    hopper.setPushInPos();
                    UtilMethods.sleep(500);
                    hopper.setPushOutPos();
                    UtilMethods.sleep(1000);
                    hopper.setPushInPos();
                    UtilMethods.sleep(500);
                    hopper.setPushOutPos();
                    UtilMethods.sleep(1000);
                    hopper.setPushInPos();
                    UtilMethods.sleep(500);
                    hopper.setPushOutPos();
                })
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
                .addDisplacementMarker(() -> {
                    wobbleArm.placeGoalAuto();
                })
                .build();

        Trajectory goToParkingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(parkingPose.vec())
                .build();


        drive.followTrajectory(goToShootingPose);

        drive.followTrajectory(goToPlaceGoalPose);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);
        drive.followTrajectory(goToPickUpGoalPose2);

        sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);

        sleep(1000);

        drive.followTrajectory(goToParkingPose);

        sleep(1000);

        wobbleArm.liftArmAuto();

        return true;
    }

}
