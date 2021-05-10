package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Camera;

import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.powershotStartPose;

@Config
public class AutonomousPathBAsync_FourRing extends AutonomousPathAsync {
    //Treakable values for tuning

    private static final double RPM_FORGIVENESS = 125;
    public static int goalX = -26;
    public static int goalY = 60;
    public static double shootingPoseAngle = -5;
    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_BC.getRPM();

    BlueGoalVisionPipeline blueGoalPipeline;

    //Pre declare trajectories
    Trajectory goToPowershotStartingPosition;
    Trajectory goToPlaceGoalPose;
    Trajectory goToPickupRingPose1;
    Trajectory goToPickupRingPose2;
    Trajectory goToPickUpGoalPose1;
    Trajectory goToPickUpGoalPose2;
    Trajectory goToPlaceSecondGoalPart1;
    Trajectory goToPlaceSecondGoalPart2;
    Trajectory goToFinalShootingPose;
    Trajectory goToParkingPose;

    //Set starting state and rings to shoot and timer
    State currentState = State.DRIVE_TO_SHOOT;
    int rings = 3;
    int powerShotState = 0;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Poses
    Pose2d shootingPosePt1 = new Pose2d (-24,21);
    Pose2d shootingPosePt2 = PoseLibrary.SHOOTING_POSE_BC.getPose2d();
    Pose2d shootingPosePt3 = new Pose2d(6.8066, 26.37388, Math.toRadians(-5));

    Pose2d placeGoalPose = new Pose2d(24, 21, Math.toRadians(0.0));
    Pose2d pickUpRingPose1 = new Pose2d(-1, 36, Math.toRadians(180.0));
    Pose2d pickUpRingPose2 = new Pose2d(-21, 36, Math.toRadians(180.0));

    Pose2d pickUpGoalPose1 = new Pose2d(-26, 36, Math.toRadians(110.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(30, 57, Math.toRadians(0.0));
    Pose2d placeSecondGoalPose2 = new Pose2d(27, 28, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(17, 27, Math.toRadians(0.0));

    BlueGoalVisionPipeline goalPipeline;

    //build trajectories on construction
    public AutonomousPathBAsync_FourRing(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper, Camera camera) {
        super(drive, wobbleArm, flywheel, collector, hopper, camera);

        //Trajectories
//        goToShootingPosePt1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
//                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
//                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
//                .build();
        goToPowershotStartingPosition =  drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(powershotStartPose.getPose2d().vec(), powershotStartPose.getPose2d().getHeading())
                .build();

        goToPlaceGoalPose = drive.trajectoryBuilder(new Pose2d(goToPowershotStartingPosition.end().vec(), Math.toRadians(-10)))
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .build();

        goToPickupRingPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
                .addDisplacementMarker(() -> hopper.setLiftDownPos())
                .addDisplacementMarker(() -> hopper.setPushOutPos())
                .addDisplacementMarker(() -> collector.turnCollectorOn())
                .lineToLinearHeading(pickUpRingPose1)
                .addDisplacementMarker(() -> wobbleArm.pickUpSecondGoal())
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickupRingPose2))

                .build();

        goToPickupRingPose2 = drive.trajectoryBuilder(goToPickupRingPose1.end())
                .splineTo(pickUpRingPose2.vec(), pickUpRingPose2.getHeading())
                .splineTo(pickUpGoalPose1.vec(),pickUpGoalPose1.getHeading())
                .addDisplacementMarker(() -> timer.reset())
                .build();


        goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickupRingPose2.end())
                .lineToSplineHeading(placeSecondGoalPose2)
                .build();


        goToFinalShootingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToLinearHeading(shootingPosePt3)
                .build();

        goToParkingPose = drive.trajectoryBuilder(goToFinalShootingPose.end())
                .lineToConstantHeading(parkPose.vec())
                .build();
    }

    @Override
    public void followPathAsync(Telemetry telemetry) {

        blueGoalPipeline = new BlueGoalVisionPipeline(telemetry);
        switch (currentState) {
            case PREPARE_CAMERA:
                camera.setHighGoalPosition();
                goalPipeline = new BlueGoalVisionPipeline(telemetry);
                camera.webcam.setPipeline(goalPipeline);
                currentState = State.DRIVE_TO_SHOOT;
                drive.changeTimeout(2.75, 1.0);
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

                break;

            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS_POWERSHOT:

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
                    currentState = State.DRIVE_TO_PLACE_GOAL;
                    flywheel.setRPM(0);
                    drive.changeTimeout(1.0);
                    drive.followTrajectoryAsync(goToPlaceGoalPose);
                    timer.reset();
                }
                else if (powerShotState == 1) {
                    //turn to -6.5 degrees
                    flywheel.setRPM(2850);
                    drive.turnAsync(Math.toRadians(-5.5));
                    currentState = State.WAIT_FOR_TURN_TO_FINISH;
                } else if (powerShotState == 2) {
                    //turn to -13 degrees
                    flywheel.setRPM(2800);
                    drive.turnAsync(Math.toRadians(-5.5));
                    currentState = State.WAIT_FOR_TURN_TO_FINISH;
                }

                break;


            case WAIT_FOR_TURN_TO_FINISH:
                if(!drive.isBusy()) {
                    currentState = State.PREPARE_TO_SHOOT_POWERSHOTS;
                }

                break;

            case DRIVE_TO_PLACE_GOAL:
                wobbleArm.liftArm();
                if (!drive.isBusy()) {
                    currentState = State.PLACE_GOAL;
                    timer.reset();
                }
                break;

            case PLACE_GOAL:
                if (!drive.isBusy()) {
                    //Trigger action depending on timer using else if logic
                    if (timer.seconds() > 2.5) {
                        //will drive to pose 1 and pose 2 using displacement marker
                        currentState = State.PICKUP_SECOND_GOAL;
                        drive.followTrajectoryAsync(goToPickupRingPose1);
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


            case PICKUP_SECOND_GOAL:
                //Trigger action depending on timer using else if logic
                if (!drive.isBusy() && timer.seconds() > 0.5) {
                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
                    wobbleArm.closeClaw();
                    hopper.setLiftUpPos();
                    collector.turnCollectorOff();
                    timer.reset();
                }
                break;

            case DRIVE_TO_PLACE_SECOND_GOAL:
                if(timer.seconds() > 0.5) {
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
                    //Trigger action depending on timer using else if logic
                if (timer.seconds() > 1.5) {
                    currentState = State.DRIVE_TO_SHOOT_FOUR;
                    rings = 1;
                    wobbleArm.liftArm();
                    flywheel.setRPM(shootingPoseRPM);
                    drive.followTrajectoryAsync(goToFinalShootingPose);
                    timer.reset();
                    //will drive to pose 1 and pose 2 using displacement marker
                } else if (UtilMethods.atTarget(WobbleArm.ARM_POS_PLACE_GOAL, wobbleArm.getArmPosition(), 10) || timer.seconds() > 1) {
                    wobbleArm.openClaw();
                } else  {
                    wobbleArm.placeGoal();
                }

                break;
            case DRIVE_TO_SHOOT_FOUR:
                if(!drive.isBusy()) {
                    flywheel.setRPM(shootingPoseRPM);
                    timer.reset();
                    currentState = State.SHOOT_SECOND;
                } else if (timer.seconds() > 0.5){
                    wobbleArm.setArmPos(1);
                    wobbleArm.closeClaw();
                }
                break;


            case SHOOT_SECOND:
                if (!drive.isBusy()) {
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
                        currentState = State.PARK;
                        drive.followTrajectoryAsync(goToParkingPose);
                    }
                }
                break;

            case PARK:
                if (!drive.isBusy()) {
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
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
        telemetry.addData("Current State", currentState);
        telemetry.addData("Goal Visible", blueGoalPipeline.isGoalVisible());
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Motor Power", blueGoalPipeline.getMotorPower());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("Rings", rings);
        telemetry.addData("In Range", UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS));
        telemetry.update();

    }


    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        PREPARE_CAMERA,
        PREPARE_TO_SHOOT_POWERSHOTS,
        SHOOT_RINGS_POWERSHOT,
        TURN_TO,
        TURN_TO_PICKUP_GOAL,
        WAIT_FOR_TURN_TO_FINISH,
        DRIVE_TO_SHOOT,
        SHOOT,   // First, follow a splineTo() trajectory
        DRIVE_TO_PLACE_GOAL,   // Then, follow a lineTo() trajectory
        PLACE_GOAL,
        DRIVE_TO_RING,// Then we want to do a point turn
        DRIVE_TO_SECOND_GOAL,   // Then, we follow another lineTo() trajectory
        PICKUP_SECOND_GOAL,         // Then we're gonna wait a second
        DRIVE_TO_PLACE_SECOND_GOAL,         // Finally, we're gonna turn again
        CHECK_DRIVE,
        PLACE_SECOND_GOAL,
        DRIVE_TO_SHOOT_FOUR,
        ALIGN_TO_GOAL,
        SHOOT_SECOND,
        PARK,
        IDLE// Our bot will enter the IDLE state when done
    }

}
