package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.CoordinateConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_CLOSE_POS;
import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.AutonomousFramework.CLAW_OPEN_POS;


@Autonomous(name = "Autonomous Test A", group = "test")
public class AutonomousTestA extends LinearOpMode {

    public DcMotor armMotor;
    public Servo clawServo;
    public static int armPos = -525;
    public static int armPos2 = -500;

    public static int testX = -37;
    public static int testY = 55;

    Pose2d shootingPose = new Pose2d(0, 24, Math.toRadians(0.0));
    Pose2d placeGoalPose = new Pose2d(0, 48, Math.toRadians(-10.0));

    Pose2d pickUpGoalPose1 = new Pose2d(-24, testY, Math.toRadians(180.0));
    Pose2d pickUpGoalPose2 = new Pose2d(testX, testY, Math.toRadians(180.0));
    Pose2d placeSecondGoalPose = new Pose2d(0, 57, Math.toRadians(0.0));
    Pose2d parkingPose = new Pose2d(10, 30, Math.toRadians(0.0));




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



        Trajectory goToShootingandPlaceGoalPose = drive.trajectoryBuilder(CoordinateConstants.START_POS_BLUE_2)
                .splineTo(placeGoalPose.vec(), placeGoalPose.getHeading())
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armPos);
                    armMotor.setPower(0.3);
                    sleep(500);
                })
                .build();

        Trajectory goToPickUpGoalPose1 = drive.trajectoryBuilder(goToShootingandPlaceGoalPose.end())
                .addDisplacementMarker(()->{
                    clawServo.setPosition(CLAW_OPEN_POS);
                    armMotor.setTargetPosition(-100);
                    armMotor.setPower(0.3);
                    sleep(500);
                })
                .lineToLinearHeading(pickUpGoalPose1)
                .build();

        Trajectory goToPickUpGoalPose2 = drive.trajectoryBuilder(goToPickUpGoalPose1.end())
                .addDisplacementMarker(() -> {
                    armMotor.setTargetPosition(armPos2);
                    armMotor.setPower(0.3);
                    sleep(500);
                })
                .lineToConstantHeading(pickUpGoalPose2.vec())
                .build();


        Trajectory goToPlaceSecondGoalPart1 = drive.trajectoryBuilder(goToPickUpGoalPose2.end())
                .addDisplacementMarker(() -> {
//                    armMotor.setTargetPosition(armPos2);
//                    armMotor.setPower(0.3);
//                    sleep(500);
                })
                .lineToSplineHeading(placeSecondGoalPose)
                .build();

        Trajectory goToParking = drive.trajectoryBuilder(goToPlaceSecondGoalPart1.end())
                .lineToConstantHeading(parkingPose.vec())
                .build();


        drive.followTrajectory(goToShootingandPlaceGoalPose);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose1);

        sleep(1000);

        drive.followTrajectory(goToPickUpGoalPose2);

        clawServo.setPosition(CLAW_CLOSE_POS);

        sleep(1000);

        drive.followTrajectory(goToPlaceSecondGoalPart1);

        clawServo.setPosition(CLAW_OPEN_POS);

        drive.followTrajectory(goToParking);

        sleep(1000);

        armMotor.setTargetPosition(armPos);
        armMotor.setPower(0.3);



        while(!isStopRequested() && opModeIsActive());

    }
}


