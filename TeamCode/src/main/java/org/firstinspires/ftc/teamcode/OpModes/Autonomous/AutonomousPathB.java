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

import static org.firstinspires.ftc.teamcode.Common.Robot.ARM_POS_LIFT_ARM;
import static org.firstinspires.ftc.teamcode.Common.Robot.ARM_POS_PICKUP_GOAL;
import static org.firstinspires.ftc.teamcode.Common.Robot.ARM_POS_PLACE_GOAL;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_OPEN_POS;
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

    public void followPath(SampleMecanumDrive drive, DcMotor armMotor, Servo clawServo) {
        Trajectory goToShootingPose = drive.trajectoryBuilder(CoordinateConstants.START_POS_BLUE_2)
                .splineTo(shootingPose.vec(), shootingPose.getHeading())
                .build();

        Trajectory goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPose.end())
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .addDisplacementMarker(() -> {
                    placeGoal(armMotor, clawServo, ARM_POS_PLACE_GOAL, CLAW_OPEN_POS);
                    sleep(500);
                })
                .build();

        Trajectory goToPickUpGoalPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
                .addDisplacementMarker(()->{
                    armMotor.setTargetPosition(ARM_POS_LIFT_ARM);
                    armMotor.setPower(0.3);
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
                    armMotor.setTargetPosition(ARM_POS_PICKUP_GOAL);
                    armMotor.setPower(0.3);
                    sleep(500);
                    clawServo.setPosition(CLAW_CLOSE_POS);
                })
                .build();


        Trajectory goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(ARM_POS_LIFT_ARM);
                    armMotor.setPower(0.3);
                })
                .lineToSplineHeading(placeSecondGoalPose1)
                .build();

        Trajectory goToPlaceSecondGoalPart2 = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(placeSecondGoalPose2.vec())
                .addDisplacementMarker(()->{
                    placeGoal(armMotor, clawServo, ARM_POS_PLACE_GOAL, CLAW_OPEN_POS);
                    sleep(500);
                })
                .build();

        Trajectory goToParkingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart2.end())
                .lineToConstantHeading(parkPose.vec())
                .build();


        drive.followTrajectory(goToShootingPose);

        drive.followTrajectory(goToPlaceGoalPose);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);
        drive.followTrajectory(goToPickUpGoalPose2);

        sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);
        drive.followTrajectory(goToPlaceSecondGoalPart2);
        drive.followTrajectory(goToParkingPose);

        sleep(1000);

        armMotor.setTargetPosition(-200);
        armMotor.setPower(0.3);

    }

}
