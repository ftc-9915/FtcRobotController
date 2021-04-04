//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Common.UtilMethods;
//import org.firstinspires.ftc.teamcode.Subsystems.Collector;
//import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveConstants;
//import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
//import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
//import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
//
//import java.util.Arrays;
//
//@Config
//public class AutonomousPathBAsync_FourRing extends AutonomousPathAsync {
//    //Treakable values for tuning
//
//    private static final double RPM_FORGIVENESS = 125;
//    public static int goalX = -25;
//    public static int goalY = 59;
//    public static double shootingPoseAngle = -5;
//    public static double shootingPoseRPM = PoseLibrary.SHOOTING_POSE_BC.getRPM();
//
//    //Pre declare trajectories
//    Trajectory goToShootingPosePt1;
//    Trajectory goToPlaceGoalPose;
//    Trajectory goToPickupRingPose1;
//    Trajectory goToPickupRingPose2;
//    Trajectory goToPickUpGoalPose1;
//    Trajectory goToPickUpGoalPose2;
//    Trajectory goToPlaceSecondGoalPart1;
//    Trajectory goToPlaceSecondGoalPart2;
//    Trajectory goToParkingPose;
//
//    //Set starting state and rings to shoot and timer
//    State currentState = State.DRIVE_TO_SHOOT;
//    int rings = 3;
//    ElapsedTime timer = new ElapsedTime();
//    boolean hopperPositionIn = false;
//
//    //Poses
//    Pose2d shootingPosePt1 = new Pose2d (-24,21);
//    Pose2d shootingPosePt2 = PoseLibrary.SHOOTING_POSE_BC.getPose2d();
//    Pose2d placeGoalPose = new Pose2d(22, 25, Math.toRadians(0.0));
//    Pose2d pickUpRingPose1 = new Pose2d(-15, 36, Math.toRadians(180.0));
//    Pose2d pickUpRingPose2 = new Pose2d(-25, 36, Math.toRadians(180.0));
//
//    Pose2d pickUpGoalPose1 = new Pose2d(-20, goalY, Math.toRadians(180.0));
//    Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
//    Pose2d placeSecondGoalPose1 = new Pose2d(27, 57, Math.toRadians(0.0));
//    Pose2d placeSecondGoalPose2 = new Pose2d(23, 33, Math.toRadians(0.0));
//    Pose2d parkPose = new Pose2d(17, 27, Math.toRadians(0.0));
//
//    //build trajectories on construction
//    public AutonomousPathBAsync_FourRing(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper) {
//        super(drive, wobbleArm, flywheel, collector, hopper);
//
//        //Trajectories
//        goToShootingPosePt1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
//                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
//                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
//                .build();
//
//        goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPosePt1.end())
//                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
//                .build();
//
//        goToPickupRingPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
//                .lineToLinearHeading(pickUpRingPose1)
//                .addDisplacementMarker(() -> hopper.setLiftDownPos())
//                .addDisplacementMarker(() -> hopper.setPushOutPos())
//                .addDisplacementMarker(() -> collector.turnCollectorOn())
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickupRingPose2))
//                .build();
//
//
//        goToPickupRingPose2 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
//                .lineToLinearHeading(pickUpRingPose2)
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpGoalPose1))
//                .build();
//
//
//        goToPickUpGoalPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
//                .lineToLinearHeading(pickUpGoalPose1)
//                .addDisplacementMarker(() -> drive.followTrajectoryAsync(goToPickUpGoalPose2))
//                .build();
//
//        goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
//                .splineTo(pickUpGoalPose2.vec(), pickUpGoalPose2.getHeading(), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
//                                )
//                        ),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//        //
//
//        goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
//                .lineToSplineHeading(placeSecondGoalPose1)
//                .addDisplacementMarker(() -> {
//                    drive.followTrajectoryAsync(goToPlaceSecondGoalPart2);
//                })
//                .build();
//
//        goToPlaceSecondGoalPart2 = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
//                .lineToConstantHeading(placeSecondGoalPose2.vec())
//                .build();
//
//        goToParkingPose = drive.trajectoryBuilder(goToPlaceSecondGoalPart2.end())
//                .lineToConstantHeading(parkPose.vec())
//                .build();
//    }
//
//    @Override
//    public void followPathAsync(Telemetry telemetry) {
//
//        switch (currentState) {
//            case DRIVE_TO_SHOOT:
//                // Check if the drive class isn't busy
//                // `isBusy() == true` while it's following the trajectory
//                // Once `isBusy() == false`, the trajectory follower signals that it is finished
//                // We move on to the next state
//                // Make sure we use the async follow function
//                if (!drive.isBusy()) {
//                    currentState = State.SHOOT;
//                    flywheel.setRPM(shootingPoseRPM);
//                    drive.followTrajectoryAsync(goToShootingPosePt1);
//                    timer.reset();
//                }
//                break;
//            case SHOOT:
//                if (!drive.isBusy()) {
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
//                        currentState = State.DRIVE_TO_PLACE_GOAL;
//                        drive.followTrajectoryAsync(goToPlaceGoalPose);
//                    }
//                }
//                break;
//
//            case DRIVE_TO_PLACE_GOAL:
//                if (!drive.isBusy()) {
//                    currentState = State.PLACE_GOAL;
//                    timer.reset();
//                }
//                break;
//
//            case PLACE_GOAL:
//                //Trigger action depending on timer using else if logic
//                if (timer.seconds() > 2) {
//                    currentState = State.DRIVE_TO_RING;
//                    wobbleArm.liftArm();
//                    //will drive to pose 1 and pose 2 using displacement marker
//                    drive.followTrajectoryAsync(goToPickupRingPose1);
//                    timer.reset();
//                } else if (timer.seconds() > 1.5) {
//                    wobbleArm.openClaw();
//                } else if (timer.seconds() > 0.5) {
//                    wobbleArm.placeGoal();
//                }
//                break;
//
//            case DRIVE_TO_RING:
//                if (!drive.isBusy()) {
//                    if(timer.seconds() > 2) {
//                        hopper.setLiftUpPos();
//                        currentState = State.DRIVE_TO_SECOND_GOAL;
//                        drive.followTrajectoryAsync(goToPickUpGoalPose1);
//                        timer.reset();
//                    }
//                }
//                break;
//
//
//            case DRIVE_TO_SECOND_GOAL:
//                if (!drive.isBusy()) {
//                    currentState = State.PICKUP_SECOND_GOAL;
//                    timer.reset();
//                }
//                if (timer.seconds() > 1){
//                    wobbleArm.pickUpSecondGoal();
//                }
//                break;
//
//
//            case PICKUP_SECOND_GOAL:
//                //Trigger action depending on timer using else if logic
//                if (timer.seconds() > 2) {
//                    currentState = State.DRIVE_TO_PLACE_SECOND_GOAL;
//                } else if (timer.seconds() > 1) {
//                    wobbleArm.closeClaw();
//                } else if (timer.seconds() > 0.5) {
//                    wobbleArm.pickUpSecondGoal();
//                }
//                break;
//
//            case DRIVE_TO_PLACE_SECOND_GOAL:
//                if(!wobbleArm.isBusy()) {
//                    wobbleArm.liftArm();
//                    //will drive to pose 1 and pose 2 using displacement marker
//                    drive.followTrajectoryAsync(goToPlaceSecondGoalPart1);
//                    currentState = State.CHECK_DRIVE;
//                }
//                break;
//
//            case CHECK_DRIVE:
//                if (!drive.isBusy()) {
//                    currentState = State.PLACE_SECOND_GOAL;
//                    timer.reset();
//                }
//                break;
//            case PLACE_SECOND_GOAL:
//                if(!drive.isBusy()) {
//                    //Trigger action depending on timer using else if logic
//                    if (timer.seconds() > 2) {
//                        currentState = State.SHOOT_SECOND;
       //                 rings   = 1;
//                        wobbleArm.liftArm();
//                        flywheel.setRPM(shootingPoseRPM);
//                        drive.followTrajectoryAsync(goToShootingPosePt1);
//                        timer.reset();
//                        //will drive to pose 1 and pose 2 using displacement marker
//                    } else if (timer.seconds() > 1.5) {
//                        wobbleArm.openClaw();
//                    } else if (timer.seconds() > 0.5) {
//                        wobbleArm.placeGoal();
//                    }
//                }
//                break;
//
//            case SHOOT_SECOND:
//                if (!drive.isBusy()) {
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
//                        currentState = State.PARK;
//                        drive.followTrajectoryAsync(goToParkingPose);
//                    }
//                }
//                break;
//
//            case PARK:
//                if (!drive.isBusy()) {
//                    wobbleArm.liftArm();
//                    currentState = State.IDLE;
//                }
//                break;
//            case IDLE:
//                wobbleArm.setArmPos(1);
//                flywheel.setRPM(0);
//                break;
//        }
//
//        // Anything outside of the switch statement will run independent of the currentState
//
//        // We update drive continuously in the background, regardless of state
//        drive.update();
//
//        // Read pose
//        Pose2d poseEstimate = drive.getPoseEstimate();
//
//        // Continually write pose to `PoseStorage`
//        PoseLibrary.AUTO_ENDING_POSE = poseEstimate;
//
//        // Print pose to telemetry
//        telemetry.addData("x", poseEstimate.getX());
//        telemetry.addData("y", poseEstimate.getY());
//        telemetry.addData("heading", poseEstimate.getHeading());
//        telemetry.addData("Current State", currentState);
//        telemetry.addData("Rings", rings);
//        telemetry.addData("In Range", UtilMethods.inRange(flywheel.getRPM(), shootingPoseRPM - RPM_FORGIVENESS, shootingPoseRPM + RPM_FORGIVENESS));
//        telemetry.update();
//
//    }
//
//
//    // This enum defines our "state"
//    // This is essentially just defines the possible steps our program will take
//    enum State {
//        DRIVE_TO_SHOOT,
//        SHOOT,   // First, follow a splineTo() trajectory
//        DRIVE_TO_PLACE_GOAL,   // Then, follow a lineTo() trajectory
//        PLACE_GOAL,
//        DRIVE_TO_RING,// Then we want to do a point turn
//        DRIVE_TO_SECOND_GOAL,   // Then, we follow another lineTo() trajectory
//        PICKUP_SECOND_GOAL,         // Then we're gonna wait a second
//        DRIVE_TO_PLACE_SECOND_GOAL,         // Finally, we're gonna turn again
//        CHECK_DRIVE,
//        PLACE_SECOND_GOAL,
//        SHOOT_SECOND,
//        PARK,
//        IDLE// Our bot will enter the IDLE state when done
//    }
//
//}
