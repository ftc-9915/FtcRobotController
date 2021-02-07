package org.firstinspires.ftc.teamcode.Subsystems.Drive.opmode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class AutonomousPath {

//    double desiredDistance = 16.0;
//
//    public Vector2d getSecondGoalPose(SampleMecanumDrive drive, Pose2d currentPose) {
//        double distanceToWall = drive.rangeSensor.getDistance(DistanceUnit.INCH);
//        double distanceError = desiredDistance - distanceToWall;
//        return new Vector2d(currentPose.getX(), currentPose.getY() + distanceError);
//    }

    public void placeGoal(DcMotor armMotor, Servo clawServo, int ARM_POS_PLACE_GOAL, double CLAW_OPEN_POS) {
        armMotor.setTargetPosition(ARM_POS_PLACE_GOAL);
        armMotor.setPower(0.3);
        sleep(600);
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    // sleep method from LinearOpMode
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
