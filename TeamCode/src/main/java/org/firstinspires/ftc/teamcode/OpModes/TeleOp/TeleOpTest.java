package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
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

    //Predeclared Trajectories
    Trajectory driveToPowershotPosition;


    ElapsedTime timer = new ElapsedTime();

    double launcherRPM;
    boolean launcherOn;
    int armPos;
    boolean buttonReleased1;
    boolean buttonReleased2;
    boolean triggerReleased;

    //initialization variables
    int rings = 0;
    int powerShotState = 1; // *** changed from 0 to 1 ***
    double[] powerShotAngles;
    double initialAngle;

    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;
    double strafePower = 1.0;
    double powershotHeadingOffset = 0;
    boolean exitToAutoAim = false;



    Pose2d_RPM[] POWER_SHOT_POSES;

    //target angle
    double angle = -6.5;

    enum Mode {
        DRIVER_CONTROL,
        LINE_TO_POINT,
        SHOOT_RINGS,
        STAFE_TO_POWERSHOT_POSITION,
        PREPARE_TO_SHOOT_POWERSHOTS,
        SHOOT_RINGS_POWERSHOT,
        ALIGN_TO_ANGLE,
        TURN_TO,
        WAIT_FOR_TURN_TO_FINISH,
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

        //Init Collector
        collector = new Collector(hardwareMap);

        //Init Wobble Arm
        wobbleArm = new WobbleArm(hardwareMap);

        //Init flywheel
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


        //create trajectories
         driveToPowershotPosition = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(0.0)))
                .lineToConstantHeading(new Vector2d(-9.9, 15))
                .build();

        //set read mode to manual
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    @Override
    public void loop() {

        //Clear Bulk Cache at beginning of loop
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();


        // Retrieve pose
        Pose2d currentPose = drive.getPoseEstimate();

        //log telemetry data for drivers
        telemetry.addData("Goal Visibility", pipeline.isGoalVisible());
        telemetry.addData("RPM", launcherRPM);
        telemetry.addData("Drive Mode: ", currentMode);

        //Universal drive controls (works in all states)

        // Adjusts launcher speed every time trigger goes below 0.4
        if (gamepad2.left_trigger > 0.4 && triggerReleased) {
            launcherRPM -= 50;
            triggerReleased = false;
        }

        if (gamepad2.right_trigger > 0.4 && triggerReleased) {
            launcherRPM += 50;
            triggerReleased = false;
        }

        //Ring Block Servo
        if (gamepad2.right_stick_y < -0.5) {
            collector.raiseRingBlock();
        }
        if (gamepad2.right_stick_y > 0.5) {
            collector.lowerRingBlock();
        }

        switch (currentMode){

            case DRIVER_CONTROL:
                // ---------------  driver control specific subsystem updates ---------------

                //ensure hopper arm returns to out position
                if (timer.seconds() > 0.75 || hopper.pushMode != Hopper.PushMode.PUSH_OUT) {
                    hopper.setPushOutPos();
                }


                // --------------- driver control mode specific  controls -----------------

                // Sets robot up for pickup
                if (gamepad1.a && buttonReleased1) {
                    collector.turnCollectorOn();
                    hopper.setPushOutPos();
                    hopper.setLiftDownPos();
                    buttonReleased1 = false;
                }

                //Turns off collector
                if (gamepad1.b && buttonReleased1) {
                    collector.turnCollectorOff();
                    buttonReleased1 = false;
                }

                // Reverses collector
                if (gamepad1.x && buttonReleased1) {
                    collector.turnCollectorReverse();
                    buttonReleased1 = false;
                }

                // Turns launcher on/off and sets to predetermined RPM speed
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


                // Pushes/retracts collector servo
                if (gamepad2.y && buttonReleased2) {
                    hopper.setPushInPos();
                    timer.reset();
                    buttonReleased2 = false;
                }

                // Lifts/Lowers the collecting platform
                if (gamepad2.left_bumper && buttonReleased2) {
                    hopper.setLiftDownPos();
                    collector.turnCollectorOn();
                    buttonReleased2 = false;
                }

                if (gamepad2.right_bumper && buttonReleased2) {
                    collector.turnCollectorOff();
                    hopper.setLiftUpPos();
                    collector.raiseRingBlock();
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
                    wobbleArm.openClaw();
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

                // Ring guard controls
                if(gamepad2.back) {
                    collector.raiseRingGuard();
                }
                if(gamepad2.start) {
                    collector.lowerRingGuard();
                }

                // ---------- state switching driver controls ------------------


                //DPAD DOWN - go to 0 degrees heading
                if (gamepad1.dpad_down) {
                    drive.turnAsync(Angle.normDelta(0 - currentPose.getHeading()));
                    currentMode = Mode.ALIGN_TO_ANGLE;
                }

                //DPAD LEFT - Shoot Powershots From Right To Left
                if (gamepad1.dpad_left) {
                    launcherRPM = 2900;
                    flywheel.setRPM(launcherRPM);
                    powerShotState = 0;

                    drive.setPoseEstimate(new Pose2d(0,0,Math.toRadians(0.0)));


                    drive.followTrajectoryAsync(driveToPowershotPosition);

                    currentMode = Mode.STAFE_TO_POWERSHOT_POSITION;

                }


                //DPAD RIGHT - Shoot Powershots From Left To Right
                if (gamepad1.dpad_right) {
                    launcherRPM = 2900;
                    flywheel.setRPM(launcherRPM);
                    powerShotState = 0;

                    Trajectory driveToPowershotPosition = drive.trajectoryBuilder(currentPose)
                            .strafeRight(63)
                            .build();

                    drive.followTrajectoryAsync(driveToPowershotPosition);

                    currentMode = Mode.STAFE_TO_POWERSHOT_POSITION;
                }

                //RIGHT BUMPER - Auto Aim
                if(gamepad1.right_bumper && pipeline.isGoalVisible()) {
                    collector.turnCollectorOff();
                    collector.raiseRingBlock();;
                    flywheel.setRPM(launcherRPM);
                    currentMode = Mode.ALIGN_TO_GOAL;
                    timer.reset();
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

                //drive input
                speed = -gamepad1.left_stick_y * strafePower;
                strafe = gamepad1.left_stick_x * strafePower;
                rotation = gamepad1.right_stick_x * strafePower;

                drive.setMotorPowers(speed + strafe + rotation, speed - strafe + rotation, speed + strafe - rotation, speed - strafe - rotation);


                break;



            case ALIGN_TO_ANGLE:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }
                if (!drive.isBusy() && !exitToAutoAim)  {
                    currentMode = Mode.DRIVER_CONTROL;
                }  else if (!drive.isBusy() && exitToAutoAim && pipeline.isGoalVisible()) {
                    exitToAutoAim = false;
                    currentMode = Mode.ALIGN_TO_GOAL;
                    collector.turnCollectorOff();
                    collector.raiseRingBlock();;
                    flywheel.setRPM(launcherRPM);
                    timer.reset();
                }
                //switch to auto aim  after align to angle sequence ends
                if(gamepad1.right_bumper) {
                    exitToAutoAim = true;
                }
                break;

            case ALIGN_TO_GOAL:
                //if goal is centered for 1 second shoot rings, else reset timer
                flywheel.setRPM(launcherRPM);
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
                if (gamepad1.left_bumper) {
                    rings = 0;
                    currentMode = Mode.DRIVER_CONTROL;
                    collector.raiseRingBlock();
                    flywheel.setRPM(0);
                }


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
            case STAFE_TO_POWERSHOT_POSITION:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                    // 0 1 2
                } else if (!drive.isBusy()) {
                    launcherRPM -= 50;
                    currentMode = Mode.PREPARE_TO_SHOOT_POWERSHOTS;
                    hopper.setPushOutPos();
                    powershotHeadingOffset = drive.getRawExternalHeading();
                    timer.reset();
                }
                flywheel.setRPM(launcherRPM);
                break;

            //when robot has reached the end of it's generated trajectory, reset timer and rings to 1, then move to shoot rings state
            case PREPARE_TO_SHOOT_POWERSHOTS:
                if (gamepad1.left_bumper) {
                    drive.cancelFollowing();
                    currentMode = Mode.DRIVER_CONTROL;
                }
                else if (!drive.isBusy()) {
                    rings = 1;
                    timer.reset();
                    currentMode = Mode.SHOOT_RINGS_POWERSHOT;
                }
                flywheel.setRPM(launcherRPM);

                break;



            //set rings to shoot and reset timer required before moving to this state
            case SHOOT_RINGS_POWERSHOT:
                //emergency exit
                if (gamepad1.left_bumper) {
                    rings = 0;
                    currentMode = Mode.DRIVER_CONTROL;
                }

                flywheel.setRPM(launcherRPM);
                hopper.setLiftUpPos();
                if (rings > 0) {
                    if (UtilMethods.inRange(flywheel.getRPM(), launcherRPM - 125, launcherRPM + 125)
                            && hopper.getPushMode() == Hopper.PushMode.PUSH_OUT && timer.seconds() > 0.25) {
                        hopper.setPushInPos();
                        timer.reset();
                    }
                    if (hopper.getPushMode() == Hopper.PushMode.PUSH_IN  && timer.seconds() > 0.25) {
                        hopper.setPushOutPos();
                        timer.reset();
                        rings--;
                    }
                } else {
                    currentMode = Mode.TURN_TO;
                    powerShotState++;
                    timer.reset();
                }


                break;


            case TURN_TO:
                if (powerShotState > 2)
                    currentMode = Mode.DRIVER_CONTROL;
                else if (powerShotState == 1) {
                    //turn to -6.5 degrees
                    drive.turnAsync(Math.toRadians(-5));
                    currentMode = Mode.WAIT_FOR_TURN_TO_FINISH;
                } else if (powerShotState == 2) {
                    //turn to -13 degrees
                    drive.turnAsync(Math.toRadians(-5));
                    currentMode = Mode.WAIT_FOR_TURN_TO_FINISH;
                }
                flywheel.setRPM(launcherRPM);

                break;


            case WAIT_FOR_TURN_TO_FINISH:
                if(!drive.isBusy()) {
                    currentMode = Mode.PREPARE_TO_SHOOT_POWERSHOTS;
                }
                flywheel.setRPM(launcherRPM);

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
                    if (flywheel.atTargetRPM() && hopper.getPushMode() == Hopper.PushMode.PUSH_OUT && timer.seconds() > 0.25) {
                        hopper.setPushInPos();
                        collector.lowerRingBlock();
                        timer.reset();
                    }
                    if (hopper.getPushMode() == Hopper.PushMode.PUSH_IN  && timer.seconds() > 0.25) {
                        hopper.setPushOutPos();
                        timer.reset();
                        rings--;
                    }
                }

                break;
        }

        //update robot
        drive.update();
        wobbleArm.update();
        telemetry.update();



    }


}