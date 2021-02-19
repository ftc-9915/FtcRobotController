package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

import java.util.Arrays;

@Config
public class AutonomousPathBAsync extends AutonomousPathAsync {

    private static final double RPM_FORGIVENESS = 200;
    //Treakable values for tuning
    public static int goalX = -30;
    public static int goalY = 59;
    public static double shootingPoseAngle = 7;
    public static double shootingPoseRPM = 3350;
    //Pre declare trajectories
    Trajectory goToShootingPose;
    Trajectory goToPlaceGoalPose;
    Trajectory goToPickUpGoalPose1;
    Trajectory goToPickUpGoalPose2;
    Trajectory goToPlaceSecondGoalPart1;
    Trajectory goToPlaceSecondGoalPart2;
    Trajectory goToParkingPose;
    //Set starting state and rings to shoot and timer
    State currentState = State.DRIVE_TO_SHOOT;
    int rings = 3;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Poses
    Pose2d shootingPose = new Pose2d(6.8066, 26.37388, Math.toRadians(shootingPoseAngle));
    Pose2d placeGoalPose = new Pose2d(22, 24, Math.toRadians(0.0));
    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(27, 57, Math.toRadians(0.0));
    Pose2d placeSecondGoalPose2 = new Pose2d(20, 30, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(12, 27, Math.toRadians(0.0));

    //build trajectories on construction
    public AutonomousPathBAsync(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper) {
        super(drive, wobbleArm, flywheel, collector, hopper);

        //Trajectories
        goToShootingPose = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPose.vec(), shootingPose.getHeading())
                .build();

        goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPose.end())
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .build();

        goToPickUpGoalPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
                .lineToLinearHeading(pickUpGoalPose1)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpGoalPose2))
                .build();

        goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
                .lineToConstantHeading(pickUpGoalPose2.vec(), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .lineToSplineHeading(placeSecondGoalPose1)
                .addDisplacementMarker(() -> {
                    drive.followTrajectoryAsync(goToPlaceSecondGoalPart2);
                })
                .build();

        goToPlaceSecondGoalPart2 = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(placeSecondGoalPose2.vec())
                .build();

        goToParkingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart2.end())
                .lineToConstantHeading(parkPose.vec())
                .build();
    }

    @Override
    public void followPathAsync(Telemetry telemetry) {

        switch (currentState) {
            case DRIVE_TO_SHOOT:
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                if (!drive.isBusy()) {
                    currentState = State.SHOOT;
                    drive.followTrajectoryAsync(goToShootingPose);
                }
                break;
            case SHOOT:
                if (!drive.isBusy()) {
                    flywheel.setRPM(shootingPoseRPM);
                    hopper.setLiftUpPos();
                    if (rings > 0) {
                        if (UtilMethods.inRange(shootingPoseRPM, shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.5) {
                            hopper.setPushOutPos();
                            hopperPositionIn = false;
                            rings--;
                        }
                    }
                    else {
                        currentState = State.DRIVE_TO_PLACE_GOAL;
                        drive.followTrajectoryAsync(goToPlaceGoalPose);
                    }
                }
                break;

            case DRIVE_TO_PLACE_GOAL:
                if (!drive.isBusy()) {
                    currentState = State.PLACE_GOAL;
                    timer.reset();
                }
                break;

            case PLACE_GOAL:
                //Trigger action depending on timer using else if logic
                if (timer.seconds() > 2) {
                    currentState = State.DRIVE_TO_SECOND_GOAL;
                    wobbleArm.liftArm();
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToPickUpGoalPose1);
                } else if (timer.seconds() > 1) {
                    wobbleArm.openClaw();
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.lowerArm();
                }
                break;
            case DRIVE_TO_SECOND_GOAL:
                if (!drive.isBusy()) {
                    currentState = State.PICKUP_SECOND_GOAL;
                    timer.reset();
                }
                break;
            case PICKUP_SECOND_GOAL:
                //Trigger action depending on timer using else if logic
                if (timer.seconds() > 2) {
                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
                    wobbleArm.liftArm();
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToPlaceSecondGoalPart1);
                } else if (timer.seconds() > 1) {
                    wobbleArm.closeClaw();
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.lowerArm();
                }
                break;
            case DRIVE_TO_PLACE_SECOND_GOAL:
                if (!drive.isBusy()) {
                    currentState = State.PLACE_SECOND_GOAL;
                    timer.reset();
                }
                break;
            case PLACE_SECOND_GOAL:
                //Trigger action depending on timer using else if logic
                if (timer.seconds() > 2) {
                    currentState = State.PARK;
                    wobbleArm.liftArm();
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToParkingPose);
                } else if (timer.seconds() > 1) {
                    wobbleArm.openClaw();
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.lowerArm();
                }
                break;
            case PARK:
                if (!drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                break;
        }

        // Anything outside of the switch statement will run independent of the currentState

        // We update drive continuously in the background, regardless of state
        drive.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Continually write pose to `PoseStorage`
        PoseLibrary.autoEndingPose = poseEstimate;

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Current State", currentState);
        telemetry.update();

    }


    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        DRIVE_TO_SHOOT,
        SHOOT,   // First, follow a splineTo() trajectory
        DRIVE_TO_PLACE_GOAL,   // Then, follow a lineTo() trajectory
        PLACE_GOAL,         // Then we want to do a point turn
        DRIVE_TO_SECOND_GOAL,   // Then, we follow another lineTo() trajectory
        PICKUP_SECOND_GOAL,         // Then we're gonna wait a second
        DRIVE_TO_PLACE_SECOND_GOAL,         // Finally, we're gonna turn again
        PLACE_SECOND_GOAL,
        PARK,
        IDLE// Our bot will enter the IDLE state when done
    }

}
