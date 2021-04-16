package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;

import java.util.Arrays;

@Config
public class AutonomousPathAAsync extends AutonomousPathAsync {


    //Pre declare trajectories
    Trajectory goToShootingPosePt1;
    Trajectory goToPickUpGoalPose1;
    Trajectory goToPickUpGoalPose2;
    Trajectory goToPlaceSecondGoalPart1;
    Trajectory goToParkingPosePart1;
    Trajectory goToParkingPosePart2;

    //Set starting state and rings to shoot and timer
    State currentState = State.DRIVE_TO_SHOOT;
    int rings = 3;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Treakable values for tuning
    private static final double RPM_FORGIVENESS = 125;
    public static int goalX = -26;
    public static int goalY = 59;
    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_A.getRPM();

    //Poses
    public static Pose2d placeGoalAndShootingPose1 = PoseLibrary.SHOOTING_POSE_A.getPose2d();
    public static Pose2d placeGoalAndShootingPose2 = new Pose2d(-5, 55, Math.toRadians(0));
    public static Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    public static Pose2d placeSecondGoalPose = new Pose2d(0, 57, Math.toRadians(0.0));
    public static Pose2d parkingPose1 = new Pose2d(0, 30, Math.toRadians(0.0));
    public static Pose2d parkingPose2 = new Pose2d(15, 30, Math.toRadians(0.0));

    //Telemetry
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    //build trajectories on construction
    public AutonomousPathAAsync(MecanumDrivebase drive, WobbleArm wobbleArm, Shooter shooter, Collector collector) {
        super(drive, wobbleArm, shooter, collector);

        //telemetry
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        //Trajectories

        //place and shoot
        goToShootingPosePt1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(placeGoalAndShootingPose1.vec(), placeGoalAndShootingPose1.getHeading())
                .build();


        goToPickUpGoalPose1 = drive.trajectoryBuilder(goToShootingPosePt1.end())
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
                .lineToSplineHeading(placeSecondGoalPose)
                .build();

        goToParkingPosePart1 = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(parkingPose1.vec())
                .addDisplacementMarker( () -> drive.followTrajectoryAsync(goToParkingPosePart2))
                .build();

        goToParkingPosePart2 = drive.trajectoryBuilder(goToParkingPosePart1.end())
                .lineToConstantHeading(parkingPose2.vec())
                .build();

    }

    @Override
    public void followPathAsync(Telemetry telemetry) {



        switch (currentState) {
            case START:
                drive.followTrajectoryAsync(goToShootingPosePt1);
                currentState = State.DRIVE_TO_SHOOT;
                break;
            case DRIVE_TO_SHOOT:
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                if (!drive.isBusy()) {
                    currentState = State.SHOOT;
                    shooter.shootRings(3, shootingPoseRPM);
                    timer.reset();

                }
                break;

            case SHOOT:
                if (!shooter.isBusy()) {
                    currentState = State.TURN_TO_PLACE_GOAL;
                }
                break;

            case TURN_TO_PLACE_GOAL:
                drive.turnTo(placeGoalAndShootingPose2.getHeading());
                if (timer.seconds() > 1) {
                    timer.reset();
                    currentState = State.PLACE_GOAL;
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
                if (timer.seconds() > 0.5){
                    wobbleArm.openClawLowerArm();
                }
                if (!drive.isBusy()) {
                    currentState = State.PICKUP_SECOND_GOAL;
                    timer.reset();
                }
                break;
            case PICKUP_SECOND_GOAL:
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
                    drive.followTrajectoryAsync(goToParkingPosePart1);
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
        telemetry.update();

    }


    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        START,
        DRIVE_TO_SHOOT,
        SHOOT,   // First, follow a splineTo() trajectory
        DRIVE_TO_PLACE_GOAL,
        TURN_TO_PLACE_GOAL,// Then, follow a lineTo() trajectory
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
