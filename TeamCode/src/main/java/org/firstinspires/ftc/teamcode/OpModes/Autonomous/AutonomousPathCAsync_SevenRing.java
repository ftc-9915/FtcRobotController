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
import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpTest;
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

@Config
public class AutonomousPathCAsync_SevenRing extends AutonomousPathAsync {


    //Pre declare trajectories
    Trajectory goToShootingPosePt1;
    Trajectory goToPlaceGoalPose;
    Trajectory goToPrepareToAccessRingStackPose1;
    Trajectory goToPushRingStackPose;
    Trajectory goToBackUpByRingStackPose;
    Trajectory goToPickUpRingPose1;

    Trajectory goToPickUpGoalPose1;
    Trajectory goToPickUpGoalPose2;
    Trajectory goToPlaceSecondGoalPart1;
    Trajectory goToParkingPose;
    Trajectory goToShootingPose2;
    Trajectory goToPrepareToAccessRingStackPose2;
    Trajectory goToPickupRingPose2;
    Trajectory goToShootingPose3;

    BlueGoalVisionPipeline goalPipeline;

    //Set starting state and rings to shoot and timer
    State currentState = State.PREPARE_CAMERA;
    int rings = 3;
    ElapsedTime timer = new ElapsedTime();
    boolean hopperPositionIn = false;

    //Treakable values for tuning
    private static final double RPM_FORGIVENESS = 125;
    public static int goalX = -33;
    public static int goalY = 54;
    public static double shootingPoseAngle = -5;
    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_BC.getRPM();

    //Poses
    Pose2d shootingPosePt1 = new Pose2d (-24,21);
    Pose2d shootingPosePt2 = PoseLibrary.SHOOTING_POSE_BC.getPose2d();
    Pose2d shootingPosePt3 = new Pose2d(2.8066, 26.37388, Math.toRadians(-6));
    Pose2d placeGoalPose = new Pose2d(48, 52, Math.toRadians(-0.1));

    Pose2d prepareToPushRingStack = new Pose2d(-5, 34, Math.toRadians(180.0));
    Pose2d pushRingStack = new Pose2d(-12, 34, Math.toRadians(180.0));
    // *then go back to ringPosePt1

    Pose2d pickUpRingPose1 = new Pose2d(-16, 34.5, Math.toRadians(180.0));
    Pose2d pickUpRingPose2 = new Pose2d(-19, 34, Math.toRadians(180.0));


    Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(45, 53, Math.toRadians(0.0));
    Pose2d parkPose = new Pose2d(15, 50, Math.toRadians(0.0));

    //build trajectories on construction
    public AutonomousPathCAsync_SevenRing(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper, Camera camera) {
        super(drive, wobbleArm, flywheel, collector, hopper, camera);

        //Trajectories
        goToShootingPosePt1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
                .build();

        goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPosePt1.end())
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .addDisplacementMarker(() -> hopper.setLiftDownPos())
                .addDisplacementMarker(() -> hopper.setPushOutPos())
                .build();

        //---------------------------------------------------------

        goToPrepareToAccessRingStackPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
                .lineToSplineHeading(prepareToPushRingStack)
                .addTemporalMarker(0.2,() -> collector.turnCollectorOnWithRingGuard())
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpRingPose1))
                .build();

//        goToPushRingStackPose = drive.trajectoryBuilder(goToPrepareToAccessRingStackPose1.end())
//                .lineToConstantHeading(pushRingStack.vec())
//                .addDisplacementMarker(() -> hopper.setLiftDownPos())
//                .addDisplacementMarker(() -> hopper.setPushOutPos())
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToBackUpByRingStackPose))
//                .build();
//
//        goToBackUpByRingStackPose = drive.trajectoryBuilder(goToPushRingStackPose.end())
//                .lineToConstantHeading(prepareToPushRingStack.vec())
//                .addDisplacementMarker(() -> collector.turnCollectorOn())
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpRingPose1))
//                .build();

        goToPickUpRingPose1 = drive.trajectoryBuilder(goToPrepareToAccessRingStackPose1.end())
                .lineToConstantHeading(pickUpRingPose1.vec(), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> timer.reset())
                .build();

        goToShootingPose2 = drive.trajectoryBuilder(goToPickUpRingPose1.end())
                .lineToSplineHeading(shootingPosePt3)
                 .addTemporalMarker(1.25, () -> {
                     collector.raiseRingGuard();
                     hopper.setLiftUpPos();
                 })
                .build();

//        goToPrepareToAccessRingStackPose2 = drive.trajectoryBuilder(goToShootingPose2.end())
//                .lineToSplineHeading(prepareToPushRingStack)
//                .addDisplacementMarker(() -> {
//                    collector.lowerRingGuard();
//                    drive.followTrajectoryAsync(goToPickupRingPose2);
//                })
//                .build();
//
//        goToPickupRingPose2 = drive.trajectoryBuilder(goToPrepareToAccessRingStackPose2.end())
//                .lineToConstantHeading(pickUpRingPose2.vec())
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToShootingPose3))
//                .build();

//        goToShootingPose3 = drive.trajectoryBuilder(goToPickupRingPose2.end())
//                .addTemporalMarker(1.5, () -> {
//                    hopper.setLiftUpPos();
//                    collector.raiseRingGuard();
//                })
//                .lineToSplineHeading(shootingPosePt3)
//                .build();

        //----------------------------------

        goToPickUpGoalPose1 = drive.trajectoryBuilder(goToShootingPose2.end())
                .lineToSplineHeading(pickUpGoalPose1)
                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpGoalPose2))
                .build();

        goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
                .lineToConstantHeading(pickUpGoalPose2.vec(), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
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
                    telemetry.addData("Camera Data", this.goalPipeline.getGoalHeight());
                    telemetry.addData("Motor Power Data", this.goalPipeline.getMotorPower());
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
                        rings = 3;
                        currentState = State.DRIVE_TO_PLACE_GOAL;
                        timer.reset();
                        drive.followTrajectoryAsync(goToPlaceGoalPose);
                        wobbleArm.closeClaw();
                        wobbleArm.placeGoal();
                    }
                }
                break;

            case DRIVE_TO_PLACE_GOAL:
                if (!drive.isBusy()) {
                    currentState = State.PLACE_GOAL;
                    wobbleArm.openClaw();

                    timer.reset();
                }
                break;

            case PLACE_GOAL:
                //Trigger action depending on timer using else if logic
                if (timer.seconds() > 0.75) {
                    currentState = State.DRIVE_TO_SHOOT_2;
                    wobbleArm.setArmPos(-100);
                    //will drive to pose 1 and pose 2 using displacement marker
                    drive.followTrajectoryAsync(goToPrepareToAccessRingStackPose1);
                    timer.reset();
                }
                break;


            case DRIVE_TO_SHOOT_2:
                if(!drive.isBusy() && timer.seconds() > 1) {
                    drive.followTrajectoryAsync(goToShootingPose2);
                    currentState = State.CHECK_DRIVE_1;
                    timer.reset();
                }
                break;

            case CHECK_DRIVE_1:
                if (!drive.isBusy()) {
                    currentState = State.ALIGN_TO_GOAL;
                    timer.reset();
                }
                break;

            case ALIGN_TO_GOAL:
                //if goal is centered for 1 second shoot rings, else reset timer
                flywheel.setRPM(PoseLibrary.SHOOTING_POSE_BC.getRPM());

                if (goalPipeline.isGoalVisible() && timer.seconds() < 1) {
                    //returns positive if robot needs to turn counterclockwise
                    double motorPower = goalPipeline.getMotorPower();

                    drive.leftFront.setPower(-motorPower);
                    drive.leftRear.setPower(-motorPower);
                    drive.rightFront.setPower(motorPower);
                    drive.rightRear.setPower(motorPower);
                } else {
                    rings = 3;
                    timer.reset();
                    drive.setMotorPowers(0,0,0,0);
                    currentState = State.SHOOT_2;
                }


                break;

            case SHOOT_2:
                if (!drive.isBusy()) {
                    telemetry.addData("Camera Data", this.goalPipeline.getGoalHeight());
                    telemetry.addData("Motor Power Data", this.goalPipeline.getMotorPower());
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
                        rings = 3;
                        currentState = State.DRIVE_TO_SECOND_GOAL;
//                        hopper.setLiftDownPos();
//                        hopper.setPushOutPos();
                        timer.reset();
                        drive.followTrajectoryAsync(goToPickUpGoalPose1);
                    }
                }
                break;

//            case SHOOT_3:
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
//                        currentState = State.DRIVE_TO_SECOND_GOAL;
//                        timer.reset();
//                        drive.followTrajectoryAsync(goToPickUpGoalPose1);
//                    }
//                }
//                break;

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
                if (timer.seconds() > 0.3) {
                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
                } else {
                    wobbleArm.closeClaw();
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
                    //TODO: Test if arm logic works
                    //Trigger action depending on timer using else if logic
                    if (timer.seconds() > 2) {
                        currentState = State.PARK;
                        wobbleArm.setArmPos(1);
                        //will drive to pose 1 and pose 2 using displacement marker
                        drive.followTrajectoryAsync(goToParkingPose);
                        timer.reset();
                    } else if (UtilMethods.atTarget(WobbleArm.ARM_POS_PLACE_GOAL, wobbleArm.getArmPosition(), 10) || timer.seconds() > 1.5) {
                        wobbleArm.openClaw();
                    } else {
                        wobbleArm.placeGoal();
                    }
                }
                break;

            case PARK:
                if (!drive.isBusy()) {
                    currentState = State.IDLE;
                } else if (timer.seconds() > 0.5) {
                    wobbleArm.closeClaw();
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
        CHECK_DRIVE_1,
        ALIGN_TO_GOAL,
        SHOOT_2,
        SHOOT_3,
        DRIVE_TO_SECOND_GOAL,   // Then, we follow another lineTo() trajectory
        PICKUP_SECOND_GOAL,         // Then we're gonna wait a second
        DRIVE_TO_PLACE_SECOND_GOAL,         // Finally, we're gonna turn again
        CHECK_DRIVE,
        PLACE_SECOND_GOAL,
        PARK,
        IDLE// Our bot will enter the IDLE state when done
    }

}
