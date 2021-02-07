package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.PoseStorage;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Shooter;


@Config
@TeleOp(name="TeleOp Test", group="test")
public class TeleOpTest extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;



    //DcMotor launcherMotor;

    DcMotor collectorMotor;

    DcMotor armMotor;

    Servo liftServo;
    Servo pushServo;
    Servo clawServo;

    ElapsedTime timer = new ElapsedTime();

    double launcherPower;
    double launcherRPM;
    boolean launcherOn;
    double collectorPower;
    int armPos;

    // Ensures that the adjustments are made each time the gamepad buttons are pressed rather than each time through loop
    boolean buttonReleased1;
    boolean buttonReleased2;
    boolean triggerReleased;

    static final double LIFT_UP_POS = 0.50;
    static final double LIFT_DOWN_POS = 0.75;
    static final double NOT_PUSH_POS = 0.70;
    static final double PUSH_POS = 0.52;

    // TODO: test these and edit with accurate values
    static final int ARM_SPEED = 2;
    static final int ARM_UPPER_LIMIT = 10000;
    static final int ARM_LOWER_LIMIT = -10000;
    static final double CLAW_OPEN_POS = 0.7;
    static final double CLAW_CLOSE_POS = 0.15;


    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;
    double strafePower = 1.0;
    boolean slowmodeOn = false;

    Shooter shooter;

    SampleMecanumDrive drive;
    public static int shootingPositionX;
    public static int shootingPositionY;
    public static double shootingPositionHeading;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    @Override
    public void init() {
        // Chassis Motors

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        // Attachment Motors
        //launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        //launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor = hardwareMap.dcMotor.get("collectorMotor");
        //launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftServo = hardwareMap.servo.get("liftServo");
        pushServo = hardwareMap.servo.get("pushServo");


        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        clawServo = hardwareMap.servo.get("clawServo");


        // Initialization values
        launcherPower = 0.0;
        launcherRPM = -60.0;
        launcherOn = false;
        collectorPower = 1.0;
        armPos = 0;
        buttonReleased1 = true;
        buttonReleased2 = true;
        triggerReleased = true;

        // Starting position
        liftServo.setPosition(LIFT_DOWN_POS);
        pushServo.setPosition(NOT_PUSH_POS);

        //set localization
        drive = new SampleMecanumDrive(hardwareMap);
//        robotLocalizer.setPoseEstimate(PoseStorage.autoEndingPose);


    }

    @Override
    public void loop() {
        drive.update();

        // Retrieve pose
        Pose2d currentPose = drive.getPoseEstimate();

        // Controls and Information
        telemetry.addData("Drive Mode: ", currentMode);
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("heading", currentPose.getHeading());

        telemetry.addData("Launcher RPM", shooter.getRPM());
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Slowmode On", slowmodeOn);
        telemetry.addLine("--- Controls (Gamepad 1) ---");
        telemetry.addData("Turn collector on", "Button A");
        telemetry.addData("Turn collector off", "Button B");
        telemetry.addData("Reverse collector direction", "Button X");
        telemetry.addData("Slowmode", "Button Y");
        telemetry.addData("Dpad Up", "Drive to shoot");
        telemetry.addData("Dpad Down", "Cancel Auto");

        telemetry.addLine();
        telemetry.addLine("--- Controls (Gamepad 2) ---");
        telemetry.addData("Open Claw", "Button A");
        telemetry.addData("Close Claw", "Button B");
        telemetry.addData("Move Arm", "Left Stick Up/Down");
        telemetry.addData("Turn launcher on/off", "Button X");
        telemetry.addData("Push/retract collector servo", "Button Y");
        telemetry.addData("Lower collector platform", "Left Bumper");
        telemetry.addData("Lift collector platform", "Right Bumper");
        telemetry.addData("Decrease Launcher Speed", "Left Trigger");
        telemetry.addData("Increase Launcher Speed", "Right Trigger");

        switch (currentMode){
            case DRIVER_CONTROL:

                // Chassis code
                speed = -gamepad1.left_stick_y * strafePower;
                strafe = gamepad1.left_stick_x * strafePower;
                rotation = gamepad1.right_stick_x * strafePower;


                leftFront.setPower(speed + strafe + rotation);
                leftBack.setPower(speed - strafe + rotation);
                rightBack.setPower(speed + strafe - rotation);
                rightFront.setPower(speed - strafe - rotation);

                // Slowmode

                if (gamepad1.y && buttonReleased1) {
                    if (slowmodeOn) {
                        strafePower = 1.0;
                        slowmodeOn = false;
                    } else {
                        strafePower = 0.5;
                        slowmodeOn = true;
                    }
                    buttonReleased1 = false;
                }

                // Turns collector on/off
                if (gamepad1.a && buttonReleased1) {
                    collectorPower = 1.0;
                    pushServo.setPosition(NOT_PUSH_POS);
                    liftServo.setPosition(LIFT_DOWN_POS);
                    buttonReleased1 = false;
                }

                if (gamepad1.b && buttonReleased1) {
                    collectorPower = 0.0;
                    buttonReleased1 = false;
                }

                // Reverses collector

                if (gamepad1.x && buttonReleased1) {
                    collectorPower = -1.0;
                    buttonReleased1 = false;
                }

                // Turns launcher on/off
                if (gamepad2.x && buttonReleased2) {
                    /*
                    if (launcherPower == 0.0) {
                        launcherPower = -0.68;
                    } else {
                        launcherPower = 0.0;
                    }
                     */
                    if (launcherOn) {
                        shooter.setRPM(0);
                    } else {
                        shooter.setRPM(launcherRPM);
                    }
                    buttonReleased2 = false;
                }

                // Adjusts launcher speed every time trigger goes below 0.4
                if (gamepad2.left_trigger > 0.4 && triggerReleased && launcherOn) {
                    //launcherPower += 0.05;
                    launcherRPM += 5;
                    shooter.setRPM(launcherRPM);
                    triggerReleased = false;
                }

                if (gamepad2.right_trigger > 0.4 && triggerReleased && launcherOn) {
                    //launcherPower -= 0.05;
                    launcherRPM -= 5;
                    shooter.setRPM(launcherRPM);
                    triggerReleased = false;
                }

                // Pushes/retracts collector servo
                if (gamepad2.y && buttonReleased2) {
                    pushServo.setPosition(PUSH_POS);
                    timer.reset();
                    buttonReleased2 = false;
                }

                if (timer.seconds() > 1) {
                    pushServo.setPosition(NOT_PUSH_POS);
                }

                // Lifts/Lowers the collecting platform
                if (gamepad2.left_bumper && buttonReleased2) {
                    liftServo.setPosition(LIFT_DOWN_POS);
                    buttonReleased2 = false;
                }

                if (gamepad2.right_bumper && buttonReleased2) {
                    liftServo.setPosition(LIFT_UP_POS);
                    buttonReleased2 = false;
                }

                // Lifts/Lowers Wobble Goal Arm

                armPos += ARM_SPEED * gamepad2.left_stick_y;
                if (armPos > ARM_UPPER_LIMIT) {
                    armPos = ARM_UPPER_LIMIT;
                }
                if (armPos < ARM_LOWER_LIMIT) {
                    armPos = ARM_LOWER_LIMIT;
                }
                armMotor.setTargetPosition(armPos);
                armMotor.setPower(1.0);


                // Opens/Closes Wobble Goal Claw

                if (gamepad2.a && buttonReleased2) {
                    clawServo.setPosition(CLAW_OPEN_POS);
                    buttonReleased2 = false;
                }
                if (gamepad2.b && buttonReleased2) {
                    clawServo.setPosition(CLAW_CLOSE_POS);
                    buttonReleased2 = false;
                }



                // Do not adjust values again until after buttons are released (and pressed again) so the
                // adjustments are made each time the gamepad buttons are pressed rather than each time through loop
                if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                    buttonReleased1 = true;
                }

                if(!gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y) {
                    buttonReleased2 = true;
                }

                if (gamepad2.left_trigger < 0.4 && gamepad2.right_trigger < 0.4) {
                    triggerReleased = true;
                }

                //launcherMotor.setPower(launcherPower);
                collectorMotor.setPower(collectorPower);


                //create trajectory to shooting position on the fly
                if (gamepad1.dpad_up) {
                    // If the D-pad up button is pressed on gamepad1, we generate a splineTo()
                    // trajectory on the fly and follow it
                    // We switch the state to AUTOMATIC_CONTROL

                    liftServo.setPosition(LIFT_UP_POS);

                    Trajectory driveToShootPositionPath = drive.trajectoryBuilder(currentPose)
                            .lineToLinearHeading(new Pose2d(shootingPositionX, shootingPositionY,  Math.toRadians(shootingPositionHeading)))
                            .build();

                    drive.followTrajectoryAsync(driveToShootPositionPath);

                    currentMode = Mode.AUTOMATIC_CONTROL;
                }

            case AUTOMATIC_CONTROL:
                // If x is pressed, we break out of the automatic following
                if (gamepad1.dpad_down) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                // If drive finishes its task, cede control to the driver
                if (!drive.isBusy()) {
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;


        }

        }


}
