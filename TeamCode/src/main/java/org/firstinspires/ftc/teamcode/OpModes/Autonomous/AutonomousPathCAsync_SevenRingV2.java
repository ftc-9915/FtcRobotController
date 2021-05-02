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
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Camera;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.BACK_SHOOTING_POSE;
import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.SHOOTING_POSE_BC;
import static org.firstinspires.ftc.teamcode.Subsystems.WobbleArm.ARM_POS_PLACE_GOAL;

public class AutonomousPathCAsync_SevenRingV2 extends AutonomousPathAsync {


    BlueGoalVisionPipeline goalPipeline;


    //Pre declare trajectories
    Trajectory goToShootingPosePt1;
    Trajectory goToPickupRingPose1;
    Trajectory goToShootingPosePt2;
    Trajectory goToPickupRingPose2;
    Trajectory goToShootingPosePt3;
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
    Pose2d shootingPosePt1 = BACK_SHOOTING_POSE.getPose2d();
    Pose2d pickUpRingPose1 = new Pose2d(-24, 36);
    Pose2d pickUpRingPose2 = new Pose2d(-22, 36);
    Pose2d shootingPosePt2 = PoseLibrary.SHOOTING_POSE_BC.getPose2d();
    Pose2d placeGoalPose = new Pose2d(48, 52, Math.toRadians(-0.1));
    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(48, 55, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(19, 50, Math.toRadians(0.0));

    //build trajectories on construction
    public AutonomousPathCAsync_SevenRingV2(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper, Camera camera) {
        super(drive, wobbleArm, flywheel, collector, hopper, camera);

        //Trajectories
        goToShootingPosePt1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                .addDisplacementMarker(() -> timer.reset())
                .build();

        goToPickupRingPose1 = drive.trajectoryBuilder(goToShootingPosePt1.end())
                .addDisplacementMarker(() -> collector.lowerRingGuard())
                .addDisplacementMarker(() -> collector.turnCollectorOn())
                .splineTo(pickUpRingPose1.vec(), pickUpRingPose1.getHeading())
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToShootingPosePt2))
                .build();

        goToShootingPosePt2 = drive.trajectoryBuilder(goToPickupRingPose1.end())
                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                .addDisplacementMarker(() -> collector.turnCollectorOff())
                .build();

        goToPickupRingPose2 = drive.trajectoryBuilder(goToShootingPosePt2.end())
                .addDisplacementMarker(() -> collector.lowerRingGuard())
                .addDisplacementMarker(() -> collector.turnCollectorOn())
                .splineTo(pickUpRingPose2.vec(), pickUpRingPose2.getHeading())
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToShootingPosePt3))
                .build();

        goToShootingPosePt3 = drive.trajectoryBuilder(goToPickupRingPose2.end())
                .addDisplacementMarker(() -> collector.raiseRingGuard())
                .addDisplacementMarker(() -> collector.turnCollectorOff())
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
            case PREPARE_CAMERA:
                camera.setHighGoalPosition();
                goalPipeline = new BlueGoalVisionPipeline(telemetry);
                camera.webcam.setPipeline(goalPipeline);
                currentState = State.DRIVE_TO_SHOOT;
                break;

            case DRIVE_TO_SHOOT:
                currentState = State.ALIGN_TO_GOAL;
                shootingPoseRPM = BACK_SHOOTING_POSE.getRPM();
                drive.followTrajectoryAsync(goToShootingPosePt1); // resets timer at end of trajectory

                break;

            case ALIGN_TO_GOAL:
                if (!drive.isBusy()) {
                    //if goal is centered for 1 second shoot rings, else reset timer
                    flywheel.setRPM(shootingPoseRPM);

                    if (goalPipeline.isGoalVisible() && timer.seconds() < 0.5) {
                        //returns positive if robot needs to turn counterclockwise
                        double motorPower = goalPipeline.getMotorPower();

                        drive.leftFront.setPower(-motorPower);
                        drive.leftRear.setPower(-motorPower);
                        drive.rightFront.setPower(motorPower);
                        drive.rightRear.setPower(motorPower);
                    } else {
                        rings = 3;
                        timer.reset();
                        drive.setMotorPowers(0, 0, 0, 0);
                        currentState = State.SHOOT;
                    }
                }
                break;

            case SHOOT:
                if (!drive.isBusy()) {
                    hopper.setLiftUpPos();
                    if (rings > 0) {
                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.25) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.25) {
                            hopper.setPushOutPos();
                            timer.reset();
                            hopperPositionIn = false;
                            rings--;
                        }
                    }
                    else {
                        currentState = State.DRIVE_TO_SHOOT_2;
                        drive.followTrajectoryAsync(goToPickupRingPose1); //picks up ring and drives back to shooting position
                    }
                }
                break;

            case DRIVE_TO_SHOOT_2:
                if (!drive.isBusy()) {
                    currentState = State.ALIGN_TO_GOAL_2;
                    shootingPoseRPM = BACK_SHOOTING_POSE.getRPM();
                    timer.reset();
                }
                break;

            case ALIGN_TO_GOAL_2:
                    //if goal is centered for 1 second shoot rings, else reset timer
                    flywheel.setRPM(shootingPoseRPM);

                    if (goalPipeline.isGoalVisible() && timer.seconds() < 0.5) {
                        //returns positive if robot needs to turn counterclockwise
                        double motorPower = goalPipeline.getMotorPower();

                        drive.leftFront.setPower(-motorPower);
                        drive.leftRear.setPower(-motorPower);
                        drive.rightFront.setPower(motorPower);
                        drive.rightRear.setPower(motorPower);
                    } else {
                        rings = 3;
                        timer.reset();
                        drive.setMotorPowers(0, 0, 0, 0);
                        currentState = State.SHOOT_2;
                    }
                break;

            case SHOOT_2:
                if (!drive.isBusy()) {
                    flywheel.setRPM(shootingPoseRPM);
                    hopper.setLiftUpPos();
                    if (rings > 0) {
                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.25) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.25) {
                            hopper.setPushOutPos();
                            timer.reset();
                            hopperPositionIn = false;
                            rings--;
                        }
                    }
                    else {
                        currentState = State.SHOOT_3;
                        drive.followTrajectoryAsync(goToPickupRingPose2); //picks up ring and drives back to shooting position
                    }
                }
                break;



            case DRIVE_TO_SHOOT_3:
                if(!drive.isBusy()) {
                    currentState = State.ALIGN_TO_GOAL_3;
                    shootingPoseRPM = SHOOTING_POSE_BC.getRPM();
                    timer.reset();
                }
                break;

            case ALIGN_TO_GOAL_3:
                //if goal is centered for 1 second shoot rings, else reset timer
                flywheel.setRPM(shootingPoseRPM);

                if (goalPipeline.isGoalVisible() && timer.seconds() < 0.5) {
                    //returns positive if robot needs to turn counterclockwise
                    double motorPower = goalPipeline.getMotorPower();

                    drive.leftFront.setPower(-motorPower);
                    drive.leftRear.setPower(-motorPower);
                    drive.rightFront.setPower(motorPower);
                    drive.rightRear.setPower(motorPower);
                } else {
                    rings = 3;
                    timer.reset();
                    drive.setMotorPowers(0, 0, 0, 0);
                    currentState = State.SHOOT_3;
                }
                break;

            case SHOOT_3:
                if (!drive.isBusy()) {
                    flywheel.setRPM(shootingPoseRPM);
                    hopper.setLiftUpPos();
                    if (rings > 0) {
                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.25) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.25) {
                            hopper.setPushOutPos();
                            timer.reset();
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
                    wobbleArm.setArmPos(-100);
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToPickUpGoalPose1);
                    timer.reset();
                } else if (timer.seconds() > 1.5) {
                    wobbleArm.openClaw();
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.placeGoal();
                }
                break;
            case DRIVE_TO_SECOND_GOAL:
                if(timer.seconds() > 1) {
                    wobbleArm.pickUpSecondGoal();
                }
                if (!drive.isBusy()) {
                    currentState = State.PICKUP_SECOND_GOAL;
                    timer.reset();
                }
                break;
            case PICKUP_SECOND_GOAL:
                //Trigger action depending on timer using else if logic
                if (timer.seconds() > 2.5) {
                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.closeClaw();
                } /*else if (timer.seconds() > 0.5) {
                    wobbleArm.pickUpSecondGoal();
                }*/
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
                    //TODO: Test if arm logic works
                    //Trigger action depending on timer using else if logic
                    if (timer.seconds() > 2) {
                        currentState = State.PARK;
                        wobbleArm.liftArm();
                        //will drive to pose 1 and pose 2 using displacement marker
                        drive.followTrajectoryAsync(goToParkingPose);
                    } else if (UtilMethods.atTarget(WobbleArm.ARM_POS_PLACE_GOAL, wobbleArm.getArmPosition(), 10) || timer.seconds() > 1.5) {
                        wobbleArm.openClaw();
                    } else if (timer.seconds() > 0.5) {
                        wobbleArm.placeGoal();
                    }
                }
                break;

            case PARK:
                if (!drive.isBusy()) {
                    wobbleArm.liftArm();
                    currentState = State.IDLE;
                }
                break;
            case IDLE:
                flywheel.setRPM(0);
                wobbleArm.setArmPos(1);
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
        telemetry.addData("In Range", flywheel.atTargetRPM());
        telemetry.update();

    }


    // This enum defines our "state"
    // This is essentially just defines the possible steps our program will take
    enum State {
        PREPARE_CAMERA,
        DRIVE_TO_SHOOT,
        SHOOT,   // First, follow a splineTo() trajectory
        ALIGN_TO_GOAL,
        DRIVE_TO_SHOOT_2,
        ALIGN_TO_GOAL_2,
        SHOOT_2,
        DRIVE_TO_SHOOT_3,
        ALIGN_TO_GOAL_3,
        SHOOT_3,
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
