package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;

import java.util.Arrays;

public class MeepMeepTesting {

    //Drive Constants
    public static double MAX_VEL = 58.75; // Value given by MaxVelocityTuner: 62.6
    public static double MAX_ACCEL = 75;
    public static double MAX_ANG_VEL = 5;
    public static double MAX_ANG_ACCEL = 8;

    public static double TRACK_WIDTH = 10.25; // in


    public static int goalX = -33;
    public static int goalY = 54;

    public static Pose2d shootingPosePt1 = new Pose2d(-24, 21);
    public static Pose2d shootingPosePt2 = new Pose2d(6.8066, 26.37388, Math.toRadians(-5));
    public static Pose2d shootingPosePt3 = new Pose2d(2.8066, 26.37388, Math.toRadians(-6));
    public static Pose2d placeGoalPose = new Pose2d(48, 52, Math.toRadians(-0.1));

    public static Pose2d prepareToPushRingStack = new Pose2d(-5, 34, Math.toRadians(180.0));
    public static Pose2d pushRingStack = new Pose2d(-12, 34, Math.toRadians(180.0));

    public static Pose2d pickUpRingPose1 = new Pose2d(-16, 34.5, Math.toRadians(180.0));
    public static Pose2d pickUpRingPose2 = new Pose2d(-19, 34, Math.toRadians(180.0));


    public static Pose2d pickUpGoalPose1 = new Pose2d(-24, goalY, Math.toRadians(180.0));
    public static Pose2d pickUpGoalPose2 = new Pose2d(goalX, goalY, Math.toRadians(180.0));
    public static Pose2d placeSecondGoalPose1 = new Pose2d(45, 53, Math.toRadians(0.0));
    public static Pose2d parkPose = new Pose2d(15, 50, Math.toRadians(0.0));

    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels


        MeepMeep mm = new MeepMeep(1200)
                // Set field image
                .setBackground(MeepMeep.Background.FIELD_ULTIMATE_GOAL_DARK)
                // Set theme
                .setTheme(new ColorSchemeRedDark())
                // Background opacity from 0-1
                .setBackgroundAlpha(1f)
                // Set constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 10.25)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-54.75, 26.5, Math.toRadians(0.0)))
                                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
                                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
                                //shoot here
                                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                                //place goal here
//                            .addDisplacementMarker(() -> hopper.setLiftDownPos())
//                            .addDisplacementMarker(() -> hopper.setPushOutPos())
                                .lineToSplineHeading(prepareToPushRingStack)
//                                .addTemporalMarker(0.2, () -> collector.turnCollectorOnWithRingGuard())
                                .lineToConstantHeading(pickUpRingPose1.vec(), new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(MAX_ANG_VEL),
                                                        new MecanumVelocityConstraint(8, TRACK_WIDTH)
                                                )
                                        ),
                                        new ProfileAccelerationConstraint(MAX_ACCEL))
//                                .addDisplacementMarker(() -> timer.reset())
                                .lineToSplineHeading(shootingPosePt3)
//                                .addTemporalMarker(1.25, () -> {
//                                    collector.raiseRingGuard();
//                                    hopper.setLiftUpPos();
//                                })
                                //shoot here
                                .lineToSplineHeading(pickUpGoalPose1)
                                .lineToConstantHeading(pickUpGoalPose2.vec(), new MinVelocityConstraint(
                                                Arrays.asList(
                                                        new AngularVelocityConstraint(MAX_ANG_VEL),
                                                        new MecanumVelocityConstraint(20, TRACK_WIDTH)
                                                )
                                        ),
                                        new ProfileAccelerationConstraint(MAX_ACCEL))
                                //pickup goal here
                                .lineToSplineHeading(placeSecondGoalPose1)
                                //place goal here
                                .lineToConstantHeading(parkPose.vec())
                                .build())
                .start();


                                //goToShootingPosePt1
//                .followTrajectorySequence(drive ->
//                    drive.trajectorySequenceBuilder(new Pose2d(-54.75, 26.5, Math.toRadians(0.0)))
//                                .splineTo(shootingPosePt1.vec(), shootingPosePt1.getHeading())
//                                .splineTo(shootingPosePt2.vec(), shootingPosePt2.getHeading())
//                                .build()
//                )
//                //goToPlaceGoalPose
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(shootingPosePt2)
//                                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
//                                .build()
//                )
//                //goToPrepareToAccessRingStackPose1
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(placeGoalPose)
//                                .lineToSplineHeading(prepareToPushRingStack)
//                                .build()
//                )
//                //goToPickUpRingPose1
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(prepareToPushRingStack)
//                                .lineToConstantHeading(pickUpRingPose1.vec(), new MinVelocityConstraint(
//                                                Arrays.asList(
//                                                        new AngularVelocityConstraint(MAX_ANG_VEL),
//                                                        new MecanumVelocityConstraint(8, TRACK_WIDTH)
//                                                )
//                                        ),
//                                        new ProfileAccelerationConstraint(MAX_ACCEL))
//                        .build()
//                )
//                //goToShootingPose2
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(pickUpRingPose1)
//                                .lineToSplineHeading(shootingPosePt3)
//                                .build()
//                )
//                //goToPickUpGoalPose1
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(shootingPosePt3)
//                                .lineToSplineHeading(pickUpGoalPose1)
//                                .build()
//                )
//                //goToPickUpGoalPose2
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(pickUpGoalPose1)
//                                .lineToConstantHeading(pickUpGoalPose2.vec(), new MinVelocityConstraint(
//                                                Arrays.asList(
//                                                        new AngularVelocityConstraint(MAX_ANG_VEL),
//                                                        new MecanumVelocityConstraint(20, TRACK_WIDTH)
//                                                )
//                                        ),
//                                        new ProfileAccelerationConstraint(MAX_ACCEL))
//                                .build()
//                )
//                //goToPlaceSecondGoalPart1
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(pickUpGoalPose2)
//                                .lineToSplineHeading(placeSecondGoalPose1)
//                                .build()
//                )
//                //goToParkingPose
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(placeSecondGoalPose1)
//                                .lineToConstantHeading(parkPose.vec())
//                                .build()
//                )

    }
}