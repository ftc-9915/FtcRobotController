package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.UtilMethods;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.Pose2d_RPM;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.Camera;

import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.POWER_SHOT_POSES_LEFT;
import static org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary.POWER_SHOT_POSES_RIGHT;


@TeleOp(name="TeleOp Test", group="test")
@Config
public class TeleOpTest extends OpMode {

    public static double aimTimer = 1;

    //Subsystems
    MecanumDrivebase drive;
    Flywheel flywheel;
    WobbleArm wobbleArm;
    Collector collector;
    Hopper hopper;

    Camera camera;
    BlueGoalVisionPipeline pipeline;



    //DcMotor launcherMotor;
//    DcMotor collectorMotor;
//    DcMotor armMotor;
//    Servo liftServo;
//    Servo pushServo;
//    Servo clawServo1;
//    Servo clawServo2;

    ElapsedTime timer = new ElapsedTime();

    double launcherRPM;
    boolean launcherOn;
    int armPos;

    // Ensures that the adjustments are made each time the gamepad buttons are pressed rather than each time through loop
    boolean buttonReleased1;
    boolean buttonReleased2;
    boolean triggerReleased;


    int rings = 0;
    int powerShotState = 1; // *** changed from 0 to 1 ***
    double[] powerShotAngles;
    double initialAngle;

    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;
    double strafePower = 1.0;
    boolean slowmodeOn = false;
    double powershotHeadingOffset = 0;



    Pose2d_RPM[] POWER_SHOT_POSES;

    //target angle
    double angle = 0.0;

    enum Mode {
        DRIVER_CONTROL,
        LINE_TO_POINT,
        SHOOT_RINGS,
        GENERATE_NEXT_POWERSHOT_PATH,
        PREPARE_TO_SHOOT_POWERSHOTS,
        SHOOT_RINGS_POWERSHOT,
        ALIGN_TO_ANGLE,
        ALIGN_TO_GOAL;
    }

    Mode currentMode = Mode.DRIVER_CONTROL;


    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Init Drive and set estimate
        drive = new MecanumDrivebase(hardwareMap, 1);
        drive.setPoseEstimate(PoseLibrary.AUTO_ENDING_POSE);



        //Init Hopper
        hopper = new Hopper(hardwareMap);
//        liftServo = hardwareMap.servo.get("liftServo");
//        pushServo = hardwareMap.servo.get("pushServo");
//        // Starting position
//        liftServo.setPosition(LIFT_DOWN_POS);
//        pushServo.setPosition(NOT_PUSH_POS);


        //Init Collector
        collector = new Collector(hardwareMap);
//      collectorMotor = hardwareMap.dcMotor.get("collectorMotor");



        //Init Wobble Arm
        wobbleArm = new WobbleArm(hardwareMap);


        flywheel = new Flywheel(hardwareMap);

        //Init Camera
        pipeline = new BlueGoalVisionPipeline(telemetry);
        camera = new Camera(hardwareMap, pipeline);
        camera.setHighGoalPosition();

        // Initialization values
        launcherRPM = 3200;
        launcherOn = false;

        armPos = 0;
        buttonReleased1 = true;
        buttonReleased2 = true;
        triggerReleased = true;


        //set read mode to manual
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void loop() {

        //Clear Bulk Cache at beginning of loop
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();



        drive.update();

        // Retrieve pose
        Pose2d currentPose = drive.getPoseEstimate();

        telemetry.addData("Powershot State", powerShotState);

        telemetry.addData("Goal Visibility", pipeline.isGoalVisible());
        telemetry.addData("RPM", launcherRPM);
        telemetry.addData("Drive Mode: ", currentMode);
        telemetry.addData("x", currentPose.getX());
        telemetry.addData("y", currentPose.getY());
        telemetry.addData("Angle To Goal", pipeline.getYaw());
        telemetry.addData("Aligned To Goal", pipeline.isGoalCentered());
        telemetry.addData("raw heading", Math.toDegrees(drive.getRawExternalHeading()));
        telemetry.addData("At Setpoint Angle",UtilMethods.inRange(Math.toDegrees(drive.getRawExternalHeading()), angle - 1, angle + 1));
        telemetry.addData("Tele Shooting Pose", PoseLibrary.TELE_SHOOTING_POSE.getPose2d());


        switch (currentMode){
            case DRIVER_CONTROL:

                // Chassis code
                speed = -gamepad1.left_stick_y * strafePower;
                strafe = gamepad1.left_stick_x * strafePower;
                rotation = gamepad1.right_stick_x * strafePower;


                drive.setMotorPowers(speed + strafe + rotation, speed - strafe + rotation, speed + strafe - rotation, speed - strafe - rotation);

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
                    collector.turnCollectorOn();
                    hopper.setPushOutPos();
                    hopper.setLiftDownPos();
                    buttonReleased1 = false;
                }

                if (gamepad1.b && buttonReleased1) {
                    collector.turnCollectorOff();
                    buttonReleased1 = false;
                }

                // Reverses collector
                if (gamepad1.x && buttonReleased1) {
                    collector.turnCollectorReverse();
                    buttonReleased1 = false;
                }

                // Turns launcher on/off
                if (gamepad2.x && buttonReleased2) {
                    if (launcherOn) {
                        flywheel.setRPM(0);
                        launcherOn = false;
                    } else {
                        flywheel.setRPM(launcherRPM);
                        launcherOn = true;
                    }
                    buttonReleased2 = false;
                }

                // Adjusts launcher speed every time trigger goes below 0.4
                if (gamepad2.left_trigger > 0.4 && triggerReleased && launcherOn) {
                    launcherRPM -= 50;
                    flywheel.setRPM(launcherRPM);
                    triggerReleased = false;
                }

                if (gamepad2.right_trigger > 0.4 && triggerReleased && launcherOn) {
                    launcherRPM += 50;
                    flywheel.setRPM(launcherRPM);
                    triggerReleased = false;
                }

                // Pushes/retracts collector servo
                if (gamepad2.y && buttonReleased2) {
                    hopper.setPushInPos();
                    timer.reset();
                    buttonReleased2 = false;
                }

                if (timer.seconds() > 0.75) {
                    hopper.setPushOutPos();
                }

                // Lifts/Lowers the collecting platform
                if (gamepad2.left_bumper && buttonReleased2) {
                    hopper.setLiftDownPos();
                    buttonReleased2 = false;
                }

                if (gamepad2.right_bumper && buttonReleased2) {
                    collector.turnCollectorOff();
                    hopper.setLiftUpPos();
                    buttonReleased2 = false;
                }

                // Lifts/Lowers Wobble Goal Arm
                armPos -= 10 * gamepad2.left_stick_y;
                if (armPos < wobbleArm.ARM_LOWER_LIMIT) {
                    armPos = wobbleArm.ARM_LOWER_LIMIT;
                }
                if (armPos > wobbleArm.ARM_UPPER_LIMIT) {
                    armPos = wobbleArm.ARM_UPPER_LIMIT;
                }

                // Arm Presets
                if (gamepad2.dpad_up && buttonReleased2) {
                    armPos = wobbleArm.ARM_POS_LIFT_ARM;
                    buttonReleased2 = false;
                }
                if (gamepad2.dpad_down && buttonReleased2) {
                    armPos = wobbleArm.ARM_POS_PICKUP_GOAL;
                    wobbleArm.openClaw();
                    buttonReleased2 = false;
                }
                if (gamepad2.dpad_right && buttonReleased2) {
                    armPos = wobbleArm.ARM_POS_OVER_WALL;
                    buttonReleased2 = false;
                }
                if (gamepad2.dpad_left && buttonReleased2) {
                    armPos = 0;
                    buttonReleased2 = false;
                }
                wobbleArm.setArmPos(armPos);

                // Opens/Closes Wobble Goal Claw
                if (gamepad2.a && buttonReleased2) {
                    wobbleArm.openClaw();
                    buttonReleased2 = false;
                }
                if (gamepad2.b && buttonReleased2) {
                    wobbleArm.closeClaw();
                    buttonReleased2 = false;
                }



                // Do not adjust values again until after buttons are released (and pressed again) so the
                // adjustments are made each time the gamepad buttons are pressed rather than each time through loop
                if (!gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                    buttonReleased1 = true;
                }

                if(!gamepad2.left_bumper && !gamepad2.right_bumper && !gamepad2.a && !gamepad2.b && !gamepad2.x && !gamepad2.y && !gamepad2.dpad_up && !gamepad2.dpad_right && !gamepad2.dpad_down && !gamepad2.dpad_left) {
                    buttonReleased2 = true;
                }

                if (gamepad2.left_trigger < 0.4 && gamepad2.right_trigger < 0.4) {
                    triggerReleased = true;
                }


                //DPAD-UP - auto drive to general shooting position
                if (gamepad1.dpad_up) {
                    collector.turnCollectorOff();
                    Trajectory driveToShootPositionPath = drive.trajectoryBuilder(currentPose)
                            .lineToLinearHeading(PoseLibrary.TELE_SHOOTING_POSE.getPose2d())
                            .build();

                    drive.followTrajectoryAsync(driveToShootPositionPath);
                    flywheel.setRPM(PoseLibrary.TELE_SHOOTING_POSE.getRPM());

                    currentMode = Mode.LINE_TO_POINT;
                }


                //DPAD DOWN - go to 0 degrees heading
                if (gamepad1.dpad_down) {
                    angle = 0;
                    currentMode = Mode.ALIGN_TO_ANGLE;
                    timer.reset();
                }

                //DPAD LEFT - Shoot Powershots From Right To Left
                if (gamepad1.dpad_left) {
/*                    POWER_SHOT_POSES = POWER_SHOT_POSES_RIGHT;
                    flywheel.setRPM(POWER_SHOT_POSES[0].getRPM());
                    drive.setPoseEstimate(POWER_SHOT_POSES[0].getPose2d());
                    powerShotState = 0;
                    currentMode = Mode.GENERATE_NEXT_POWERSHOT_PATH;
                }

                //DPAD RIGHT - Shoot Powershots From Left To Right
                if (gamepad1.dpad_right) {
                    POWER_SHOT_POSES = POWER_SHOT_POSES_LEFT;
                    flywheel.setRPM(POWER_SHOT_POSES[0].getRPM());
                    drive.setPoseEstimate(POWER_SHOT_POSES[0].getPose2d());
                    powerShotState = 0;
                    currentMode = Mode.GENERATE_NEXT_POWERSHOT_PATH;
                }

                //RIGHT BUMPER - Auto Aim
                if(gamepad1.right_bumper && pipeline.isGoalVisible()) {
                    collector.turnCollectorOff();
                    flywheel.setRPM(launcherRPM);
                    currentMode = Mode.ALIGN_TO_GOAL;
                    timer.reset();
                }


                //GAMEPAD 2 LEFT STICK BUTTON - Set shooting position
                if(gamepad2.left_stick_button) {
                    PoseLibrary.TELE_SHOOTING_POSE.setPose2d(currentPose);
                }


                //GAMEPAD 2 RIGHT STICK BUTTON - Reset odometry to white line to starting line to maintain good distance
                if(gamepad2.right_stick_button) {
                    drive.setPoseEstimate(new Pose2d(12, currentPose.getY(), currentPose.getHeading()));
                }

                break;



            case ALIGN_TO_ANGLE:
                //pass angle in degrees
                drive.turnTo(angle);

                if (UtilMethods.inRange(Math.toDegrees(drive.getRawExternalHeading()), angle - 1, angle + 1) && timer.seconds() > 0.02){
                    currentMode = Mode.DRIVER_CONTROL;
                    timer.reset();
                } else{
                    timer.reset();
                }

                if (gamepad1.left_bumper)
                    currentMode = Mode.DRIVER_CONTROL;

                break;

            case ALIGN_TO_GOAL:
                //if goal is centered for 1 second shoot rings, else reset timer
                if (timer.seconds() > aimTimer) {
                    if(pipeline.isGoalVisible()){
                        rings = 3;
                        timer.reset();
                        launcherOn = true;
                        currentMode = Mode.SHOOT_RINGS;
                    } else {
                        timer.reset();
                    }
                }

                //emergency exit
                if (gamepad1.left_bumper)
                    currentMode = Mode.DRIVER_CONTROL;

                if(pipeline.isGoalVisible()) {
                    //returns positive if robot needs to turn counterclockwise
                    double motorPower = pipeline.getMotorPower();

                    drive.leftFront.setPower(-motorPower);
                    drive.leftRear.setPower(-motorPower);
                    drive.rightFront.setPower(motorPower);
                    drive.rightRear.setPower(motorPower);
                }
                break;

            // generate a trajectory based on powershot state and move to stop and aim state
            case GENERATE_NEXT_POWERSHOT_PATH:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                    // 0 1 2
                } else if (powerShotState < POWER_SHOT_POSES.length - 1) {
                    //0 1 2
                    Trajectory driveToPowerShotPose = drive.trajectoryBuilder(POWER_SHOT_POSES[powerShotState].getPose2d())
                            //1 2 3
                            .lineToSplineHeading(POWER_SHOT_POSES[powerShotState + 1].getPose2d())
                            .build();

                    drive.followTrajectoryAsync(driveToPowerShotPose);

                    powerShotState++;
                    currentMode = Mode.PREPARE_TO_SHOOT_POWERSHOTS;
                } else {
                    flywheel.setRPM(0);
                    currentMode = Mode.DRIVER_CONTROL;
                }
                break;

            //when robot has reached the end of it's generated trajectory, reset timer and rings to 1, then move to shoot rings state
            case PREPARE_TO_SHOOT_POWERSHOTS:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                } else if (!drive.isBusy()) {
                    rings = 1;
                    timer.reset();
                    currentMode = Mode.SHOOT_RINGS_POWERSHOT;
                }
                break;



            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS_POWERSHOT:
                //emergency exit
                if (gamepad1.left_bumper) {
                    rings = 0;
                    currentMode = Mode.DRIVER_CONTROL;
                }

                flywheel.setRPM(POWER_SHOT_POSES_LEFT[powerShotState].getRPM());
                hopper.setLiftUpPos();
                if (rings > 0) {
                    if (flywheel.atTargetRPM()
                            && hopper.getPushMode() == Hopper.PushMode.PUSH_OUT && timer.seconds() > 0.5) {
                        hopper.setPushInPos();
                        timer.reset();
                    }
                    if (hopper.getPushMode() == Hopper.PushMode.PUSH_IN  && timer.seconds() > 0.5) {
                        hopper.setPushOutPos();
                        timer.reset();
                        rings--;
                    }
                } else {
                    timer.reset();
                    currentMode = Mode.GENERATE_NEXT_POWERSHOT_PATH;
                }

                break;


            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS:
                drive.setMotorPowers(0,0,0,0);
                //emergency exit
                if (gamepad1.left_bumper || rings == 0) {
                    rings = 0;
                    currentMode = Mode.DRIVER_CONTROL;
                    flywheel.setRPM(0);
                } else {
                    flywheel.setRPM(launcherRPM);
                    hopper.setLiftUpPos();
                }
                if (rings > 0) {
                    if (flywheel.atTargetRPM() && hopper.getPushMode() == Hopper.PushMode.PUSH_OUT && timer.seconds() > 0.5) {
                        hopper.setPushInPos();
                        timer.reset();
                    }
                    if (hopper.getPushMode() == Hopper.PushMode.PUSH_IN  && timer.seconds() > 0.5) {
                        hopper.setPushOutPos();
                        timer.reset();
                        rings--;
                    }
                }
                break;




            case LINE_TO_POINT:
                // If left bumper is pressed, we break out of the automatic following
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }

                // If drive finishes its task, we break out of the automatic following
                if (!drive.isBusy()) {
                    timer.reset();
                    currentMode = Mode.ALIGN_TO_GOAL;
                }
                break;

        }

        telemetry.update();


    }


}