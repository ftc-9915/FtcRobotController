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
public class AutonomousPathCAsync extends AutonomousPathAsync {


    //Pre declare trajectories
    Trajectory goToShootingPosePt1;
    Trajectory goToPlaceGoalPose;
    Trajectory goToPickUpGoalPose1;
    Trajectory goToPickUpGoalPose2;
    Trajectory goToPlaceSecondGoalPart1;
    Trajectory goToParkingPose;


    //Set starting state and rings to shoot and timer
    State currentState = State.DRIVE_TO_SHOOT;
    int rings = 3;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Treakable values for tuning
    private static final double RPM_FORGIVENESS = 125;
    public static int goalX = -25;
    public static int goalY = 57;
    public static double shootingPoseAngle = -5;
    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_BC.getRPM();

    //Poses
    Pose2d shootingPosePt1 = new Pose2d (-24,21);
    Pose2d shootingPosePt2 = PoseLibrary.SHOOTING_POSE_BC.getPose2d();
    Pose2d placeGoalPose = new Pose2d(48, 52, Math.toRadians(-0.1));
    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(48, 55, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(19, 50, Math.toRadians(0.0));

    //build trajectories on construction
    public AutonomousPathCAsync(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper) {
        super(drive, wobbleArm, flywheel, collector, hopper);

        //Trajectories
        goToShootingPosePt1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
                .build();

        goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPosePt1.end())
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
                .build();

        goToParkingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
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
                    flywheel.setRPM(shootingPoseRPM);
                    drive.followTrajectoryAsync(goToShootingPosePt1);
                    timer.reset();
                }
                break;
            case SHOOT:
                if (!drive.isBusy()) {
                    flywheel.setRPM(shootingPoseRPM);
                    hopper.setLiftUpPos();
                    if (rings > 0) {
                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.5) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.5) {
                            hopper.setPushOutPos();
                            timer.reset();
                            hopperPositionIn = false;
                            rings--;
                        }
                    }
                    else {
                        currentState = State.DRIVE_TO_PLACE_GOAL;
                        drive.followTrajectoryAsync(goToPlaceGoalPose);
                        flywheel.setRPM(0);
                    }
                }
                break;

            case DRIVE_TO_PLACE_GOAL:
                if (!drive.isBusy()) {
                    wobbleArm.placeGoal();
                    currentState = State.PLACE_GOAL;
                    timer.reset();
                }
                break;

            case PLACE_GOAL:
                //Wait for arm to finish
                if (!wobbleArm.isBusy()) {
                    currentState = State.DRIVE_TO_SECOND_GOAL;
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToPickUpGoalPose1);
                    timer.reset();
                }

                break;
            case DRIVE_TO_SECOND_GOAL:
                // lower arm on the way to second goal
                if(timer.seconds() > 1) {
                    wobbleArm.openClawLowerArm();
                }
                //  after driving to goal, pickup goal
                if (!drive.isBusy()) {
                    currentState = State.PICKUP_SECOND_GOAL;
                    wobbleArm.closeClawLiftArm();
                }
                break;
            case PICKUP_SECOND_GOAL:
                //  after the arm picks up the goal, drive to place second goal
                if (!wobbleArm.isBusy()) {
                    drive.followTrajectoryAsync(goToPlaceSecondGoalPart1);
                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
                }
                break;
            case DRIVE_TO_PLACE_SECOND_GOAL:
                // after driving to zone c, place goal
                if (!drive.isBusy()) {
                    currentState = State.PLACE_SECOND_GOAL;
                    wobbleArm.placeGoal();
                }
                break;
            case PLACE_SECOND_GOAL:
                //after goal is placed, drive to park
                if (!wobbleArm.isBusy()) {
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToParkingPose);
                    currentState = State.PARK;
                }
                break;

            case PARK:
                // after driving to parking pose, store arm
                if (!drive.isBusy()) {
                    wobbleArm.storeArm();
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                break;
        }

        // Anything outside of the switch statement will run independent of the currentState

        // We update drive continuously in the background, regardless of state
        drive.update();
        wobbleArm.update();

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Continually write pose to `PoseStorage`
        PoseLibrary.AUTO_ENDING_POSE = poseEstimate;

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Current State", currentState);
        telemetry.addData("Rings", rings);
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("In Range", UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS));
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
        CHECK_DRIVE,
        PLACE_SECOND_GOAL,
        PARK,
        IDLE// Our bot will enter the IDLE state when done
    }

}
