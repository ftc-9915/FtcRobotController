//package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
//import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
//import com.noahbres.jotai.StateMachine;
//import com.noahbres.jotai.StateMachineBuilder;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Common.UtilMethods;
//import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpTest;
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
//import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousPathBAsync_FourRing.AutoState.*;
//import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.*;
//
//@Config
//public class AutonomousPathBAsync_FourRing extends AutonomousPathAsync {
//    //Treakable values for tuning
//
//    private static final double RPM_FORGIVENESS = 125;
//    public static int goalX = -26;
//    public static int goalY = 36;
//    public static double goalAngle = 150.0;
//    public static double shootingPoseAngle = -5;
//    public static double shootingPoseRPM = 3400;
//
//    private StateMachine autoMachine;
//
//    // This enum defines our "state"
//    // This is essentially just defines the possible steps our program will take
//    public  enum AutoState {
//        GO_TO_POWERSHOT_1,
//        SHOOT_POWERSHOT_1,
//        GO_TO_POWERSHOT_2,
//        SHOOT_POWERSHOT_2,
//        GO_TO_POWERSHOT_3,
//        SHOOT_POWERSHOT_3,
//        PLACE_WOBBLE,
//        GO_TO_PICKUP_RING,
//        GO_TO_SECOND_WOBBLE,
//        PICKUP_SECOND_WOBBLE,
//        GO_TO_SHOOT,
//        SHOOT,
//        GO_TO_PLACE_SECOND_WOBBLE,
//        PLACE_SECOND_WOBBLE,
//        PARK,
//        IDLE// Our bot will enter the IDLE state when done
//    }
//
//
//    //Pre declare trajectories
//    Trajectory goToPowershotPose1;
//    Trajectory goToPowershotPose2;
//    Trajectory goToPowershotPose3;
//    Trajectory goToPlaceGoalPose;
//    Trajectory goToPickupRingPose1;
//    Trajectory goToPickupRingPose2;
//    Trajectory goToPickUpGoalPose1;
//    Trajectory goToPlaceSecondGoalPart1;
//    Trajectory goToPlaceSecondGoalPart2;
//    Trajectory goToParkingPose;
//
//    //Set starting state and rings to shoot and timer
//    int rings = 3;
//    ElapsedTime timer = new ElapsedTime();
//    boolean hopperPositionIn = false;
//
//    //Poses
//    Pose2d shootingPose = new Pose2d(6.8066, 26.37388, Math.toRadians(shootingPoseAngle));
//    Pose2d placeGoalPose = new Pose2d(22, 25, Math.toRadians(0.0));
//    Pose2d pickUpRingPosePt1 = new Pose2d(-22, 36, Math.toRadians(180.0));
//    Pose2d pickUpRingPosePt2 = new Pose2d(-25, 36, Math.toRadians(180.0));
//    Pose2d pickUpGoalPose = new Pose2d(goalX, goalY, Math.toRadians(goalAngle));
//    Pose2d placeSecondGoalPose1 = new Pose2d(27, 57, Math.toRadians(0.0));
//    Pose2d placeSecondGoalPose2 = new Pose2d(23, 33, Math.toRadians(0.0));
//    Pose2d parkPose = new Pose2d(17, 27, Math.toRadians(0.0));
//
//    //build trajectories on construction
//    public AutonomousPathBAsync_FourRing(MecanumDrivebase drive, WobbleArm wobbleArm, Flywheel flywheel, Collector collector, Hopper hopper) {
//        super(drive, wobbleArm, flywheel, collector, hopper);
//
//        //Trajectories
//        goToPowershotPose1 = drive.trajectoryBuilder(PoseLibrary.START_POS_BLUE_2)
//                .splineTo(POWER_SHOT_POSE_1.getPose2d().vec(), POWER_SHOT_POSE_1.getPose2d().getHeading())
//                .build();
//
//        goToPowershotPose2 = drive.trajectoryBuilder(goToPowershotPose1.end())
//                .splineTo(POWER_SHOT_POSE_2.getPose2d().vec(), POWER_SHOT_POSE_2.getPose2d().getHeading())
//                .build();
//
//        goToPowershotPose3 = drive.trajectoryBuilder(goToPowershotPose2.end())
//                .splineTo(POWER_SHOT_POSE_3.getPose2d().vec(), POWER_SHOT_POSE_3.getPose2d().getHeading())
//                .build();
//
//        goToPlaceGoalPose = drive.trajectoryBuilder(goToPowershotPose1.end())
//                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
//                .build();
//
//        goToPickupRingPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
//                .lineToLinearHeading(pickUpRingPosePt1)
//                .addDisplacementMarker(collector::turnCollectorOn)
//                .addDisplacementMarker(()->drive.followTrajectoryAsync(goToPickupRingPose2))
//                .build();
//
//        goToPickupRingPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
//                .lineToConstantHeading(pickUpRingPosePt2.vec(), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
//                                )
//                        ),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addDisplacementMarker(collector::turnCollectorOff)
//                .build();
//
//
//        goToPickUpGoalPose1 = drive.trajectoryBuilder(goToPickupRingPose2.end())
//                .splineTo(pickUpGoalPose.vec(), pickUpGoalPose.getHeading(), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(8, DriveConstants.TRACK_WIDTH)
//                                )
//                        ),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();
//
//
//        goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
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
//
//        autoMachine = new StateMachineBuilder<AutoState>()
//                .state(GO_TO_POWERSHOT_1)
//                .onEnter(() -> drive.followTrajectoryAsync(goToPowershotPose1)) // followTrajectory upon entering state
//                .transition(() -> !drive.isBusy()) //leave state when trajectory finishs
//
//                .state(SHOOT_POWERSHOT_1)
//                .loop()
//
//                .state(GO_TO_POWERSHOT_2)
//                .onEnter(() -> drive.followTrajectoryAsync(traj1)) // followTrajectory upon entering state
//                .transition(() -> !drive.isBusy())                 // Transition when trajectory is finished
//
//                .state(GO_TO_POWERSHOT_3)
//                .onEnter(() -> drive.followTrajectoryAsync(traj2)) // followTrajectory upon entering state
//                .transition(() -> !drive.isBusy())                 // Transition when trajectory is finished
//
//                .state(GO_TO_PICKUP_RING)
//                .onEnter(() -> wobbleMachine.start())               // Start the wobble machine upon entering state
//                .transition(() -> wobbleMachine.getState() == HOLD) // We wait until wobble machine is on HOLD
//
//                .state(GO_TO_SECOND_WOBBLE)
//                .onEnter(() -> drive.followTrajectoryAsync(traj3)) // followTrajectory upon entering state
//                .transition(() -> !drive.isBusy())                 // Transition when trajectory is finished
//
//                .state(PICKUP_SECOND_WOBBLE)
//                .transition(() -> wobbleMachine.transition()) // Force wobble machine to transition to next state
//
//
//                .exit(IDLE) // Exit to an IDLE state
//
//                .build();
//    }
//
//    @Override
//    public void followPathAsync(Telemetry telemetry) {
//
//
//
//
//    }
//\
//
//
//
//
//}
