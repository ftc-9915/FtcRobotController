package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.CoordinateConstants;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.AutonomousPath;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Common.Robot.ARM_POS_PLACE_GOAL;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_OPEN_POS;

@Config
public class AutonomousPathA extends AutonomousPath {

    public static int armPosPlaceGoal = -525;
    public static int armPosPickupGoal = -510;
    public static int armPosLiftArm = -200;

    public static int goalX = -32;
    public static int goalY = 53;

    // shootingPose not used
    Pose2d shootingPose = new Pose2d(0, 24, Math.toRadians(0.0));

    Pose2d placeGoalAndShootingPose = new Pose2d(-5, 55, Math.toRadians(-10.0));

    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose = new Pose2d(0, 57, Math.toRadians(0.0));
    Pose2d parkingPose = new Pose2d(10, 30, Math.toRadians(0.0));

    public void followPath(SampleMecanumDrive drive, DcMotor armMotor, Servo clawServo) {
        Trajectory goToShootingAndPlaceGoalPose = drive.trajectoryBuilder(CoordinateConstants.START_POS_BLUE_2)
                .splineTo(placeGoalAndShootingPose.vec(), placeGoalAndShootingPose.getHeading())
                .addDisplacementMarker(() -> {
                    placeGoal(armMotor, clawServo, ARM_POS_PLACE_GOAL, CLAW_OPEN_POS);
                })
                .build();

        Trajectory goToPickUpGoalPose1 = drive.trajectoryBuilder(goToShootingAndPlaceGoalPose.end())
                .addDisplacementMarker(()->{
                    clawServo.setPosition(CLAW_OPEN_POS);
                    armMotor.setTargetPosition(-100);
                    armMotor.setPower(0.3);
                    sleep(500);
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
                    armMotor.setTargetPosition(armPosPickupGoal);
                    armMotor.setPower(0.3);
                    sleep(500);
                })
                .build();


        Trajectory goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .addDisplacementMarker(()->{
                    armMotor.setTargetPosition(armPosLiftArm);
                    armMotor.setPower(0.3);
                })
                .lineToSplineHeading(placeSecondGoalPose)
                .addDisplacementMarker(() -> {
                    placeGoal(armMotor, clawServo, ARM_POS_PLACE_GOAL, CLAW_OPEN_POS);
                })
                .build();

        Trajectory goToParking = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armPosLiftArm);
                    armMotor.setPower(0.3);
                    sleep(500);
                })
                .lineToConstantHeading(parkingPose.vec())
                .build();


        drive.followTrajectory(goToShootingAndPlaceGoalPose);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose2);

        clawServo.setPosition(CLAW_CLOSE_POS);

        sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);

        clawServo.setPosition(CLAW_OPEN_POS);

        drive.followTrajectory(goToParking);

        sleep(1000);


    }



}
