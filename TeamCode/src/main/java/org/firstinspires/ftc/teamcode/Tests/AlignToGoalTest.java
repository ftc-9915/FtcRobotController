package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp.TeleOpTest;
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
        drive.setPoseEstimate(PoseLibrary.AUTO_ENDING_POSE);

        pipeline = new BlueGoalVisionPipeline(telemetry);
        camera = new Camera(hardwareMap, pipeline);
        camera.setHighGoalPosition();
        mode = Mode.DRIVER_MODE;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    }

    @Override
    public void loop() {



        Pose2d driveDirection = new Pose2d();

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

                if(gamepad1.right_bumper) {
                    //turn to zero degrees
                    drive.turnAsync(Math.toRadians(0.0));

                    if (!drive.isBusy())
                    {
                        mode = Mode.ALIGN_TO_GOAL_VISION_MODE;
                    }
                }

                break;

            case ALIGN_TO_GOAL_VISION_MODE:



                //won't auto exit for debug purposes
                if (gamepad1.left_bumper)
                    telemetry.addLine("Left Bumper Pushed");
                    mode = Mode.DRIVER_MODE;

                if(pipeline.isGoalVisible() && !pipeline.isGoalCentered()) {
                    //returns posiptve if robot needs to turn counterclockwise
                    double motorPower = pipeline.getMotorPower(BlueGoalVisionPipeline.HIGH_GOAL_SETPOINT);

                    drive.leftFront.setPower(-motorPower);
                    drive.leftRear.setPower(-motorPower);
                    drive.rightFront.setPower(motorPower);
                    drive.rightRear.setPower(motorPower);
                }

                //for logging and tuning purposes
                if(pipeline.isGoalVisible()) {
                    telemetry.addData("Motor Power", pipeline.getMotorPower(BlueGoalVisionPipeline.HIGH_GOAL_SETPOINT));
                    telemetry.addData("Error", pipeline.headingController.getPositionError());
                    telemetry.addData("Yaw", pipeline.getYaw());
                }

                break;

//            case ALIGN_TO_GOAL_ENCODER_MODE:
//
//                // Switch back into normal driver control mode if `b` is pressed
//                if (gamepad1.b) {
//                    mode = Mode.DRIVER_MODE;
//                }
//
//                // Difference between the target vector and the bot's position
//                Vector2d difference = targetPosition.minus(poseEstimate.vec());
//                // Obtain the target angle for feedback and derivative for feedforward
//                double theta = Angle.normDelta(difference.angle());
//
//
//                // Set the target heading for the heading controller to our desired angle
//                headingController.setTargetPosition(theta);
//
//                // Set desired angular velocity to the heading controller output + angular
//                // velocity feedforward
//                headingInput = (headingController.update(poseEstimate.getHeading())
//                        * DriveConstants.kV)
//                        * DriveConstants.TRACK_WIDTH;
//
//                driveDirection = new Pose2d(
//                        -gamepad1.left_stick_y,
//                        -gamepad1.left_stick_x,
//                        headingInput
//                );
//
//                break;
        }

        //Press a to toggle between mask and output
        if (gamepad1.a) {
            pipeline.onViewportTapped();
        }

        drive.setWeightedDrivePower(driveDirection);
        drive.update();

        telemetry.update();


    }


}
