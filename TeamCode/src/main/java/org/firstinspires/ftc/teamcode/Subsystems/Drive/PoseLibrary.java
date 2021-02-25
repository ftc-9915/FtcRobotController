package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseLibrary {

    // Starting positions
    public static Pose2d START_POS_BLUE_2 = new Pose2d(-55, 26.5, Math.toRadians(0.0));
    public static Pose2d START_POS_BLUE_1 = new Pose2d(-55, 48, Math.toRadians(0.0));

    //transfer pose between opmodes
    public static Pose2d AUTO_ENDING_POSE = new Pose2d(72, 60, 0);

    //common shooting poses
    public static Pose2d_RPM SHOOTING_POSE_A = new Pose2d_RPM(-5, 55, Math.toRadians(-27.5), 3150); //used in auto path A
    public static Pose2d_RPM SHOOTING_POSE_BC = new Pose2d_RPM(6.8066, 26.37388, Math.toRadians(-5), 3400); //used in auto path B and C


    //automatic power shot poses
    public static Pose2d_RPM POWER_SHOT_START_POSE = new Pose2d_RPM(0, 64, Math.toRadians(0.0), 3300);
    public static Pose2d_RPM POWER_SHOT_POSE_1 = new Pose2d_RPM(-6, 24, Math.toRadians(-45.0), 3300);
    public static Pose2d_RPM POWER_SHOT_POSE_2 = new Pose2d_RPM(-6, 24, Math.toRadians(-40.0), 3300);
    public static Pose2d_RPM POWER_SHOT_POSE_3 = new Pose2d_RPM(-6, 24, Math.toRadians(-35.0), 3300);

    public static Pose2d_RPM[] POWER_SHOT_POSES = {POWER_SHOT_START_POSE, POWER_SHOT_POSE_1, POWER_SHOT_POSE_2, POWER_SHOT_POSE_3};


}
