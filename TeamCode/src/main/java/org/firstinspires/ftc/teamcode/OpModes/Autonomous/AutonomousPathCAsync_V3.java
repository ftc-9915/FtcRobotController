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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;

@Config
public class AutonomousPathCAsync_V3 extends AutonomousPathAsync {


    //Pre declare trajectories
    Trajectory goToShootingPose;
    Trajectory goToPlaceGoalPose;
    TrajectorySequence goToPickupRings;
    TrajectorySequence goToPickupRingsAndGoal;
    TrajectorySequence goPlaceSecondGoalAndPark;



    BlueGoalVisionPipeline goalPipeline;

    //Set starting state and rings to shoot and timer
    State currentState = State.PREPARE_CAMERA;
    int rings = 3;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Treakable values for tuning
    private static final double RPM_FORGIVENESS = 125;
    public static int goalX = -34;
    public static int goalY = 34;
    public static double shootingPoseAngle = -5;
    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_BC.getRPM();

    //Poses
    Pose2d shootingPosePt1 = new Pose2d (-24,21);
    Pose2d shootingPosePt2 = PoseLibrary.SHOOTING_POSE_BC.getPose2d();
    Pose2d shootingPosePt3 = new Pose2d(2.8066, 26.37388, Math.toRadians(-6));
    Pose2d placeGoalPose = new Pose2d(48, 52, Math.toRadians(-0.1));

    Pose2d prepareToPushRingStack = new Pose2d(-5, 34, Math.toRadians(180.0));


    Pose2d pickUpRingPose1 = new Pose2d(-16, 34.5, Math.toRadians(180.0));
    Pose2d pickUpRingPose2 = new Pose2d(-19, 34.5, Math.toRadians(180.0));


    public static Pose2d pickUpGoalPose1 = new Pose2d(-16, 34, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(150));
    Pose2d placeSecondGoalPose1 = new Pose2d(45, 53, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(15, 50, Math.toRadians(0.0));

    //build trajectories on construction
    public AutonomousPathCAsync_V3(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper, Camera camera) {
        super(drive, wobbleArm, flywheel, collector, hopper, camera);


        //Trajectories
        goToShootingPose = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
                //shoot here
                .build();

        goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPose.end())
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .addTemporalMarker(1.5, () -> wobbleArm.placeGoal())
                .addTemporalMarker(2.0, () -> wobbleArm.openClaw())
                .addDisplacementMarker(() -> hopper.setLiftDownPos())
                .addDisplacementMarker(() -> hopper.setPushOutPos())
                .build();


        goToPickupRings = drive.trajectorySequenceBuilder(goToPlaceGoalPose.end())
                .addDisplacementMarker(() -> wobbleArm.storeArm())
                .lineToSplineHeading(prepareToPushRingStack)
                .addTemporalMarker(0.2, () -> collector.turnCollectorOnWithRingGuard())
                .lineToConstantHeading(pickUpRingPose1.vec(), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .waitSeconds(1.0)
                .lineToSplineHeading(shootingPosePt3)
                .addTemporalMarker(1.25, () -> {
                    collector.raiseRingGuard();
                    hopper.setLiftUpPos();
                })
                //align and shoot here
                .build();

        goToPickupRingsAndGoal  = drive.trajectorySequenceBuilder(goToPickupRings.end())
                .addDisplacementMarker(() -> {
                    collector.turnCollectorOnWithRingGuard();
                    hopper.setLiftDownPos();
                })
                .lineToLinearHeading(pickUpGoalPose1)
                .addDisplacementMarker(() -> wobbleArm.pickUpSecondGoal())
                .splineTo(pickUpGoalPose2.vec(), pickUpGoalPose2.getHeading())
                .addDisplacementMarker(() -> wobbleArm.closeClaw())
                .waitSeconds(0.25)
                .addDisplacementMarker(() -> wobbleArm.liftArm())
                .lineToSplineHeading(shootingPosePt3)
                .addTemporalMarker(1.25, () -> {
                    collector.raiseRingGuard();
                })
                //ALIGN_TO_GOAL
                .build();

        goPlaceSecondGoalAndPark = drive.trajectorySequenceBuilder(goToPickupRingsAndGoal.end())
                .lineToSplineHeading(placeSecondGoalPose1)
                .addTemporalMarker(1.5, () -> wobbleArm.placeGoal())
                .addTemporalMarker(2.0, () -> wobbleArm.openClaw())
                .addDisplacementMarker(() -> wobbleArm.storeArm())
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
                flywheel.setRPM(shootingPoseRPM);
                drive.followTrajectoryAsync(goToShootingPose);
                currentState = State.DRIVE_TO_SHOOT;
                break;

            case DRIVE_TO_SHOOT:
                if (!drive.isBusy()) {
                    currentState = State.SHOOT;
                    timer.reset();
                }
                break;

            case SHOOT:
                flywheel.setRPM(shootingPoseRPM);
                if (rings > 0) {
                    if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.4) {
                        hopper.setPushInPos();
                        timer.reset();
                        hopperPositionIn = true;
                    }
                    if (hopperPositionIn && timer.seconds() > 0.4) {
                        hopper.setPushOutPos();
                        timer.reset();
                        hopperPositionIn = false;
                        rings--;
                    }
                }
                else {
                    rings = 3;
                    currentState = State.PLACE_GOAL;
                    drive.followTrajectoryAsync(goToPlaceGoalPose);
                }
                break;


            case PLACE_GOAL:
                if(!drive.isBusy()) {
                    drive.followTrajectorySequenceAsync(goToPickupRings);
                    currentState = State.PICKUP_RINGS;
                }
                break;

            case PICKUP_RINGS:
                if (!drive.isBusy()) {
                    currentState = State.ALIGN_TO_GOAL;
                    timer.reset();
                }
                break;

            case ALIGN_TO_GOAL:
                flywheel.setRPM(shootingPoseRPM);

                //if goal is centered for 1 second shoot rings, else reset timer
                if (goalPipeline.isGoalVisible() && timer.seconds() < 1) {
                    //returns positive if robot needs to turn counterclockwise
                    double motorPower = goalPipeline.getMotorPower();

                    drive.leftFront.setPower(-motorPower);
                    drive.leftRear.setPower(-motorPower);
                    drive.rightFront.setPower(motorPower);
                    drive.rightRear.setPower(motorPower);
                } else {
                    hopper.setLiftUpPos();
                    rings = 2;
                    timer.reset();
                    drive.setMotorPowers(0,0,0,0);
                    currentState = State.SHOOT_2;
                }

                break;

            case SHOOT_2:
                if (!drive.isBusy()) {
                    if (rings > 0) {
                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.4) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.4) {
                            hopper.setPushOutPos();
                            timer.reset();
                            hopperPositionIn = false;
                            rings--;
                        }
                    }
                    else {
                        currentState = State.PICKUP_RING_AND_SECOND_GOAL;
                        timer.reset();
                        drive.followTrajectorySequence(goToPickupRingsAndGoal);
                        flywheel.setRPM(0);
                    }
                }
                break;



            case PICKUP_RING_AND_SECOND_GOAL:
                if(!drive.isBusy()) {
                    currentState = State.ALIGN_TO_GOAL_2;
                    flywheel.setRPM(PoseLibrary.SHOOTING_POSE_BC.getRPM());
                    timer.reset();
                }
                break;


            case ALIGN_TO_GOAL_2:
                flywheel.setRPM(shootingPoseRPM);
                //if goal is centered for 1 second shoot rings, else reset timer
                if (goalPipeline.isGoalVisible() && timer.seconds() < 1) {
                    //returns positive if robot needs to turn counterclockwise
                    double motorPower = goalPipeline.getMotorPower();

                    drive.leftFront.setPower(-motorPower);
                    drive.leftRear.setPower(-motorPower);
                    drive.rightFront.setPower(motorPower);
                    drive.rightRear.setPower(motorPower);
                } else {
                    hopper.setLiftUpPos();
                    rings = 2;
                    timer.reset();
                    drive.setMotorPowers(0,0,0,0);
                    currentState = State.SHOOT_3;
                }
                break;


            case SHOOT_3:
                if (!drive.isBusy()) {
                    hopper.setLiftUpPos();
                    if (rings > 0) {
                        if (UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS) && !hopperPositionIn && timer.seconds() > 0.4) {
                            hopper.setPushInPos();
                            timer.reset();
                            hopperPositionIn = true;
                        }
                        if (hopperPositionIn && timer.seconds() > 0.4) {
                            hopper.setPushOutPos();
                            timer.reset();
                            hopperPositionIn = false;
                            rings--;
                        }
                    }
                    else {
                        currentState = State.PLACE_SECOND_GOAL;
                        flywheel.setRPM(0);
                        timer.reset();
                        drive.followTrajectorySequence(goPlaceSecondGoalAndPark);
                    }
                }
                break;

            case PLACE_SECOND_GOAL:
                if(!drive.isBusy()) {
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

        telemetry.addData("Current arm position", wobbleArm.currentPosition);
        telemetry.addData("Target arm position", wobbleArm.targetArmPosition);

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
        PREPARE_CAMERA,
        DRIVE_TO_SHOOT,
        SHOOT,   // First, follow a splineTo() trajectory
        DRIVE_TO_PLACE_GOAL,   // Then, follow a lineTo() trajectory
        PLACE_GOAL,
        DRIVE_TO_SHOOT_2,// Then we want to do a point turn
        PICKUP_RINGS,
        ALIGN_TO_GOAL,
        SHOOT_2,
        SHOOT_3,
        DRIVE_TO_SECOND_GOAL,   // Then, we follow another lineTo() trajectory
        PICKUP_SECOND_GOAL,         // Then we're gonna wait a second
        PICKUP_RING_AND_SECOND_GOAL,         // Finally, we're gonna turn again
        ALIGN_TO_GOAL_2,
        PLACE_SECOND_GOAL,
        PARK,
        IDLE// Our bot will enter the IDLE state when done
    }

}
