package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Drive.CoordinateConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_OPEN_POS;


@Autonomous(name = "Autonomous Test B", group = "test")
public class AutonomousTestB extends LinearOpMode {

    public DcMotor armMotor;
    public Servo clawServo;
    public static int armPos = -500;
    public static int armPos2 = -400;

    public static int testX = -35;
    public static int testY = 59;

    Pose2d shootingPose = new Pose2d(0, 24, Math.toRadians(0.0));
    Pose2d placeGoalPose = new Pose2d(22, 24, Math.toRadians(0.0));
    Pose2d pickUpGoalPose1 = new Pose2d(-24, testY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(testX, testY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose1 = new Pose2d(27, 57, Math.toRadians(0.0));
    Pose2d placeSecondGoalPose2 = new Pose2d(20, 27, Math.toRadians(0.0));




    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        armMotor = hardwareMap.dcMotor.get("armMotor"); // this stuff is going to be replaced by robot class later
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawServo = hardwareMap.servo.get("clawServo");
        clawServo.setPosition(CLAW_CLOSE_POS);

        // Starting Position
        drive.setPoseEstimate(CoordinateConstants.START_POS_BLUE_2);


        waitForStart();

        if (isStopRequested()) return;



        Trajectory goToShootingPose = drive.trajectoryBuilder(CoordinateConstants.START_POS_BLUE_2)
                .splineTo(shootingPose.vec(), shootingPose.getHeading())
                .build();

        Trajectory goToPlaceGoalPose = drive.trajectoryBuilder(goToShootingPose.end())
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armPos);
                    armMotor.setPower(0.3);
                    sleep(500);
                })
                .build();

        Trajectory goToPickUpGoalPose1 = drive.trajectoryBuilder(goToPlaceGoalPose.end())
                .addDisplacementMarker(()->{
                    clawServo.setPosition(CLAW_OPEN_POS);
                })
                .lineToLinearHeading(pickUpGoalPose1)
                .build();

        Trajectory goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
                .lineToConstantHeading(pickUpGoalPose2.vec())
                .addDisplacementMarker(() -> {
                    clawServo.setPosition(CLAW_CLOSE_POS);
                })
                .build();


        Trajectory goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armPos2);
                    armMotor.setPower(0.3);
                })
                .lineToSplineHeading(placeSecondGoalPose1)
                .build();

        Trajectory goToPlaceSecondGoalPart2 = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(placeSecondGoalPose2.vec())
                .addDisplacementMarker(()->{
                    clawServo.setPosition(CLAW_OPEN_POS);
                })
                .build();


        drive.followTrajectory(goToShootingPose);

        drive.followTrajectory(goToPlaceGoalPose);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);
        drive.followTrajectory(goToPickUpGoalPose2);

        sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);
        drive.followTrajectory(goToPlaceSecondGoalPart2);

        sleep(1000);

        armMotor.setTargetPosition(-200);
        armMotor.setPower(0.3);



        while(!isStopRequested() && opModeIsActive());

    }
}


