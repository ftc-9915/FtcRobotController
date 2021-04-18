package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.teamcode.Vision.VisionPipeline;

@Config
@TeleOp(group = "drive")
public class PositionTuner extends LinearOpMode {

    public static double flywheelRpm = 0;
    public static double liftServoPosition = Hopper.LIFT_UP_POS;
    public static double pushServoPosition = Hopper.PUSH_OUT_POS;
    public static double leftClawPosition = WobbleArm.LEFT_CLAW_OPEN_POS;
    public static double rightClawPosition = WobbleArm.RIGHT_CLAW_OPEN_POS;
    public static int wobbleArmPosition = WobbleArm.ARM_POS_PICKUP_GOAL;
    public static double ringBlockServoPosition = Collector.RAISE_RING_BLOCK;
    public static double ringGuardServoPosition = Collector.RAISE_RING_GUARD;
    public static double cameraServoPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrivebase drive = new MecanumDrivebase(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Hopper hopper = new Hopper(hardwareMap);
        Collector collector = new Collector(hardwareMap);
        WobbleArm wobbleArm = new WobbleArm(hardwareMap);
        Camera camera = new Camera (hardwareMap, new VisionPipeline());

        waitForStart();

        while (!isStopRequested()) {

            flywheel.setRPM(flywheelRpm);
            hopper.liftServo.setPosition(liftServoPosition);
            hopper.pushServo.setPosition(pushServoPosition);
            wobbleArm.setArmPos(wobbleArmPosition);
            wobbleArm.clawServoLeft.setPosition(leftClawPosition);
            wobbleArm.clawServoRight.setPosition(rightClawPosition);
            camera.cameraServo.setPosition(cameraServoPosition);
            collector.ringBlockServo.setPosition(ringBlockServoPosition);
            collector.ringGuardServo.setPosition(ringGuardServoPosition);


            drive.update();



        }
    }
}