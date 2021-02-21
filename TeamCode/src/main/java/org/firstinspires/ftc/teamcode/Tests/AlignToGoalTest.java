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

    public static double kP = 0.02;
    public static double kD = 0;

    //in degrees
    public static double setPoint = 0;
    public static double tolerance = 5;

    PDController pd;
    private PIDFController headingController = new PIDFController(MecanumDrivebase.HEADING_PID);
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(72, 36);


    Mode mode;





    @Override
    public void init() {
        drive = new MecanumDrivebase(hardwareMap);
        drive.setPoseEstimate(PoseLibrary.START_POS_BLUE_2);

        pipeline = new BlueGoalVisionPipeline(telemetry);
        camera = new Camera(hardwareMap, pipeline);

        pd = new PDController(kP, kD);
        pd.setSetPoint(setPoint);
        pd.setTolerance(tolerance);

        mode = Mode.DRIVER_MODE;

    }

    @Override
    public void loop() {

        Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

        Pose2d driveDirection = new Pose2d();


        //for tuning purposes
        pd.setP(kP);
        pd.setD(kD);
        pd.setSetPoint(setPoint);
        pd.setTolerance(tolerance);

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

                if(pipeline.isGoalVisible()){
                    double heading_error = pipeline.getYaw();
                    double steering_adjust;

                    //when yaw is positive, turn clockwise
                    if (heading_error > 0){
                        steering_adjust = -pd.calculate();
                    //when yaw is negative, turn counter-clockwise
                    } else {
                        steering_adjust = pd.calculate();
                    }

                    telemetry.addData("Heading error", heading_error);
                    telemetry.addData("Steering adjust", steering_adjust);

                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            steering_adjust
                    );
                    //need to turn clockwisse to see goal
                } else if (UtilMethods.inRange(Math.toDegrees(poseEstimate.getHeading()),1, 179)){
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -0.5
                    );
                } else { //need to turn counter clockwise to see goal
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            0.5
                    );
                }

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
                double headingInput = (headingController.update(poseEstimate.getHeading())
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
        telemetry.update();


    }


}
