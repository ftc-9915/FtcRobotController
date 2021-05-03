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
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Pose2d_RPM;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Camera;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.powershotStartPose;

@Config
public class AutonomousPathAAsync extends AutonomousPathAsync {


    //Pre declare trajectories
    Trajectory goToDropGoalPosePt1;
    Trajectory goToPowershotStartingPosition;
    Trajectory goToPickUpGoalPose1;
    Trajectory goToPickUpGoalPose2;
    Trajectory goToPickUpGoalPose3;
    Trajectory goToPlaceSecondGoalPart1;
    Trajectory goToParkingPosePart1;
    Trajectory goToParkingPosePart2;

    //Set starting state and rings to shoot and timer
    State currentState = State.DRIVE_TO_SHOOT;
    int rings = 3;
    int powerShotState = 1;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Treakable values for tuning
    private static final double RPM_FORGIVENESS = 125;
    public static int goalX = -28;
    public static int goalY = 59;
    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_A.getRPM();

    //Poses
    public static Pose2d placeGoalAndShootingPose1 = new Pose2d(28, 36, Math.toRadians(90.0));
    public static Pose2d pickUpGoalPose1 = new Pose2d(28, 28, Math.toRadians(90.0));
    public static Pose2d pickUpGoalPose2 = new Pose2d(-20, goalY, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose3 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    public static Pose2d placeSecondGoalPose = new Pose2d(0, 57, Math.toRadians(0.0));
    public static Pose2d parkingPose1 = new Pose2d(0, 30, Math.toRadians(0.0));
    public static Pose2d parkingPose2 = new Pose2d(15, 30, Math.toRadians(0.0));

    //Telemetry
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    BlueGoalVisionPipeline goalPipeline;


    //build trajectories on construction
    public AutonomousPathAAsync(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper, Camera camera) {
        super(drive, wobbleArm, flywheel, collector, hopper, camera);

        drive.changeTimeout(2.0);

        //telemetry
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        //Trajectories

        //place and shoot
        goToPowershotStartingPosition =  drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(powershotStartPose.getPose2d().vec(), powershotStartPose.getPose2d().getHeading())
                .build();


        goToDropGoalPosePt1 = drive.trajectoryBuilder(goToPowershotStartingPosition.end())
                .splineTo(placeGoalAndShootingPose1.vec(), placeGoalAndShootingPose1.getHeading())
                .build();


        goToPickUpGoalPose1 = drive.trajectoryBuilder(goToDropGoalPosePt1.end())
                .lineToLinearHeading(pickUpGoalPose1)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpGoalPose2))
                .build();

        goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
                .lineToLinearHeading(pickUpGoalPose2)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpGoalPose3))
                .build();

        goToPickUpGoalPose3 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .lineToConstantHeading(pickUpGoalPose3.vec(), new MinVelocityConstraint(
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

        dashboardTelemetry.addData("Desired Velocity", Flywheel.rpmToTicksPerSecond(shootingPoseRPM));
        dashboardTelemetry.addData("Current Velocity", flywheel.flywheelMotor.getVelocity());
        dashboardTelemetry.addData("At RPM", flywheel.atTargetRPM());

        switch (currentState) {
            case PREPARE_CAMERA:
                camera.setHighGoalPosition();
                goalPipeline = new BlueGoalVisionPipeline(telemetry);
                camera.webcam.setPipeline(goalPipeline);
                currentState = State.DRIVE_TO_SHOOT;
                break;

            case DRIVE_TO_SHOOT:
                // Check if the drive class isn't busy
                // `isBusy() == true` while it's following the trajectory
                // Once `isBusy() == false`, the trajectory follower signals that it is finished
                // We move on to the next state
                // Make sure we use the async follow function
                if (!drive.isBusy()) {
                    currentState = State.PREPARE_TO_SHOOT_POWERSHOTS;
                    flywheel.setRPM(powershotStartPose.getRPM());
                    drive.followTrajectoryAsync(goToPowershotStartingPosition);
                    rings = 1;
                    timer.reset();
                    powerShotState = 0;
                }
                break;
            //when robot has reached the end of it's generated trajectory, reset timer and rings to 1, then move to shoot rings state
            case PREPARE_TO_SHOOT_POWERSHOTS:
                if (!drive.isBusy()) {
                    rings = 1;
                    timer.reset();
                    currentState = State.SHOOT_RINGS_POWERSHOT;
                }
                flywheel.setRPM(powershotStartPose.getRPM());

                break;

            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS_POWERSHOT:

                flywheel.setRPM(powershotStartPose.getRPM());
                hopper.setLiftUpPos();
                if (rings > 0) {
                    if (UtilMethods.inRange(flywheel.getRPM(), powershotStartPose.getRPM() - 125, powershotStartPose.getRPM() + 125)
                            && hopper.getPushMode() == Hopper.PushMode.PUSH_OUT && timer.seconds() > 0.25) {
                        hopper.setPushInPos();
                        timer.reset();
                    }
                    if (hopper.getPushMode() == Hopper.PushMode.PUSH_IN && timer.seconds() > 0.25) {
                        hopper.setPushOutPos();
                        timer.reset();
                        rings--;
                    }
                } else {
                    currentState = State.TURN_TO;
                    powerShotState++;
                    timer.reset();
                }


                break;


            case TURN_TO:
                if (powerShotState > 2) {
                    currentState = State.TURN_TO_PLACE_GOAL;
                    drive.changeTimeout(0.8);
                    drive.followTrajectoryAsync(goToDropGoalPosePt1);
                }
                else if (powerShotState == 1) {
                    //turn to -6.5 degrees
                    drive.turnAsync(Math.toRadians(-5));
                    currentState = State.WAIT_FOR_TURN_TO_FINISH;
                } else if (powerShotState == 2) {
                    //turn to -13 degrees
                    drive.turnAsync(Math.toRadians(-5));
                    currentState = State.WAIT_FOR_TURN_TO_FINISH;
                }
                flywheel.setRPM(powershotStartPose.getRPM());

                break;


            case WAIT_FOR_TURN_TO_FINISH:
                if(!drive.isBusy()) {
                    currentState = State.PREPARE_TO_SHOOT_POWERSHOTS;
                }
                flywheel.setRPM(powershotStartPose.getRPM());

                break;

//            case SHOOT:
//                if (!drive.isBusy()) {
//                    flywheel.setRPM(shootingPoseRPM);
//                    hopper.setLiftUpPos();
//                    if (rings > 0) {
//                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.5) {
//                            hopper.setPushInPos();
//                            timer.reset();
//                            hopperPositionIn = true;
//                        }
//                        if (hopperPositionIn && timer.seconds() > 0.5) {
//                            hopper.setPushOutPos();
//                            timer.reset();
//                            hopperPositionIn = false;
//                            rings--;
//                        }
//                    }
//                    else {
//                        timer.reset();
//                        currentState = State.TURN_TO_PLACE_GOAL;
//                    }
//                }
//                break;
//
//            case TURN_TO_SHOOT_POWERSHOT_2:
//                drive.turnAsync();
//                break;

            case TURN_TO_PLACE_GOAL:
//                drive.turnTo(placeGoalAndShootingPose2.getHeading());
                if (!drive.isBusy()) {
                    timer.reset();
                    currentState = State.PLACE_GOAL;
                }
                break;

            case PLACE_GOAL:
                if (!drive.isBusy()) {
                    //Trigger action depending on timer using else if logic
                    if (timer.seconds() > 2.5) {
                        //will drive to pose 1 and pose 2 using displacement marker
                        currentState = State.DRIVE_TO_SECOND_GOAL;
                        drive.followTrajectoryAsync(goToPickUpGoalPose1);
                        timer.reset();
                    }
                    else if (timer.seconds() > 2) {
                        wobbleArm.setArmPos(-100);
                    } else if (timer.seconds() > 1.8) {
                        wobbleArm.openClaw();
                    } else if (timer.seconds() > 0.5) {
                        wobbleArm.placeGoal();
                    }
                }
                break;
            case DRIVE_TO_SECOND_GOAL:
                if (timer.seconds() > 0.5){
                    wobbleArm.pickUpSecondGoal();
                }
                if (!drive.isBusy()) {
                    currentState = State.PICKUP_SECOND_GOAL;
                    timer.reset();
                }
                break;
            case PICKUP_SECOND_GOAL:
                //Trigger action depending on timer using else if logic
                if (timer.seconds() > 2) {
                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
                } else if (timer.seconds() > 1) {
                    wobbleArm.closeClaw();
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.pickUpSecondGoal();
                }
                break;
            case DRIVE_TO_PLACE_SECOND_GOAL:
                if(!wobbleArm.isBusy()) {
                    wobbleArm.liftArm();
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToPlaceSecondGoalPart1);
                    currentState = State.CHECK_DRIVE;
                }
                break;

            case CHECK_DRIVE:
                if (!drive.isBusy()) {
                    currentState = State.PLACE_SECOND_GOAL;
                    timer.reset();
                }
                break;
            case PLACE_SECOND_GOAL:
                if(!drive.isBusy()) {
                    //Trigger action depending on timer using else if logic
                    if (timer.seconds() > 3){
                        //will drive to pose 1 and pose 2 using displacement marker
                        currentState = State.PARK;
                        drive.followTrajectoryAsync(goToParkingPosePart1);
                    } else if (timer.seconds() > 2) {
                        wobbleArm.setArmPos(-100);
                    } else if (timer.seconds() > 1.5) {
                        wobbleArm.openClaw();
                    } else if (timer.seconds() > 0.5) {
                        wobbleArm.placeGoal();
                    }
                }
                break;

            case PARK:
                if (!drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                wobbleArm.setArmPos(1);
                flywheel.setRPM(0);
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
        telemetry.addData("RPM", flywheel.getRPM());
        telemetry.addData("In Range", UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS));
        telemetry.addData("Current RPM", flywheel.getRPM());
        telemetry.update();

    }


    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        PREPARE_CAMERA,
        DRIVE_TO_SHOOT,
        PREPARE_TO_SHOOT_POWERSHOTS,
        SHOOT_RINGS_POWERSHOT,
        TURN_TO,
        WAIT_FOR_TURN_TO_FINISH,
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
