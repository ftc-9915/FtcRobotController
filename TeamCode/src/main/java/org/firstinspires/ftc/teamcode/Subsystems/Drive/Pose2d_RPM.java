package org.firstinspires.ftc.teamcode.Subsystems.Drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;


//Wrapper Class for Pose2d that also contains RPM
public class Pose2d_RPM  {
    private  Pose2d pose2d;
    private  double RPM;


    public Pose2d_RPM(Pose2d pose2d, double RPM) {
        this.pose2d = pose2d;
        this.RPM = RPM;
    }

    public Pose2d_RPM(double x, double y, double heading, double RPM) {
        pose2d = new Pose2d(x, y, heading);
        this.RPM = RPM;
    }

    public double getRPM() {
        return RPM;
    }

    public void setRPM(double rpm) {
        this.RPM = rpm;
    }

    public void setPose2d(Pose2d pose2d) {
        this.pose2d = pose2d;
    }


    public Pose2d getPose2d() {
        return pose2d;
    }
}
