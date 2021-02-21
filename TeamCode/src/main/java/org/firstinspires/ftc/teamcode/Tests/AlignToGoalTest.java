package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.PDController;
import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Camera;

@Config
@TeleOp(name = "AlignToGoalTest", group = "test")
public class AlignToGoalTest extends OpMode {

    enum Mode {
        DRIVER_MODE,
        ALIGN_TO_GOAL_VISION_MODE,
        ALIGN_TO_GOAL_ENCODER_MODE,
    }

    Camera camera;
    BlueGoalVisionPipeline pipeline;
    MecanumDrivebase drive;

    private PIDFController headingController = new PIDFController(MecanumDrivebase.HEADING_PID);
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(72, 36);


    Mode mode;


    @Override
    public void init() {
        drive = new MecanumDrivebase(hardwareMap);
        drive.setPoseEstimate(PoseLibrary.autoEndingPose);

        pipeline = new BlueGoalVisionPipeline(telemetry);
        camera = new Camera(hardwareMap, pipeline);

        mode = Mode.DRIVER_MODE;

    }

    @Override
    public void loop() {

        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        Pose2d driveDirection = new Pose2d();

        double headingInput = 0;

        telemetry.addData("Mode:", mode);

        switch (mode) {
            case DRIVER_MODE:
                // Standard teleop control
                // Convert gamepad input into desired pose velocity
                //negative values for pose go clockwise
                driveDirection = new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                );
                if(gamepad1.a) {
                    mode = Mode.ALIGN_TO_GOAL_VISION_MODE;
                }
                if(gamepad1.b) {
                    mode = Mode.ALIGN_TO_GOAL_ENCODER_MODE;
                }
                break;

            case ALIGN_TO_GOAL_VISION_MODE:

                if(gamepad1.a) {
                    mode = Mode.DRIVER_MODE;
                }

                double current_heading = poseEstimate.getHeading();

                if(pipeline.isGoalVisible()){

                    double heading_error = Math.toRadians(pipeline.getYaw());
                    headingController.setTargetPosition(current_heading - heading_error);
                } else {
                    headingController.setTargetPosition(0);
                }

                headingInput = (headingController.update(current_heading)
                        * DriveConstants.kV)
                        * DriveConstants.TRACK_WIDTH;

                driveDirection = new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        headingInput
                );

                break;

            case ALIGN_TO_GOAL_ENCODER_MODE:

                // Switch back into normal driver control mode if `b` is pressed
                if (gamepad1.b) {
                    mode = Mode.DRIVER_MODE;
                }

                // Difference between the target vector and the bot's position
                Vector2d difference = targetPosition.minus(poseEstimate.vec());
                // Obtain the target angle for feedback and derivative for feedforward
                double theta = difference.angle();
                // Set the target heading for the heading controller to our desired angle
                headingController.setTargetPosition(theta);

                // Set desired angular velocity to the heading controller output + angular
                // velocity feedforward
                headingInput = (headingController.update(poseEstimate.getHeading())
                        * DriveConstants.kV)
                        * DriveConstants.TRACK_WIDTH;

                driveDirection = new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        headingInput
                );

                break;
        }


        drive.setWeightedDrivePower(driveDirection);
        headingController.update(poseEstimate.getHeading());
        drive.getLocalizer().update();

        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addData("heading input", headingInput);
        telemetry.update();


    }


}
