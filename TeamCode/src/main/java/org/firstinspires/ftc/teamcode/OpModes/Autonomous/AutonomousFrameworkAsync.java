package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.RingPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.teamcode.Vision.VisionPipeline;


@Autonomous(name = "AutonomousFrameworkAsync", group = "test")
public class AutonomousFrameworkAsync extends OpMode {


    Camera camera;
    VisionPipeline ringDetectPipeline;

    RingPosition ringConfiguration = RingPosition.NONE;



    boolean pathIsFinished = false;


    AutonomousPathAsync path;

    MecanumDrivebase drive;
    WobbleArm wobbleArm;
    Flywheel flywheel;
    Collector collector;
    Hopper hopper;


    @Override
    public void init() {
        //Init Drive
        drive = new MecanumDrivebase(hardwareMap);
        drive.setPoseEstimate(PoseLibrary.START_POS_BLUE_2);

        //Initialize Wobble Arm
        wobbleArm = new WobbleArm(hardwareMap);

        //Initialize Flywheel
        flywheel = new Flywheel(hardwareMap);

        //Initialize collector
        collector = new Collector(hardwareMap);

        //Initialize hopper
        hopper = new Hopper(hardwareMap);


        //Initialize webcam
        ringDetectPipeline = new VisionPipeline();
        camera = new Camera(hardwareMap, ringDetectPipeline);
    }

    //Called every 25 seconds
    @Override
    public void init_loop() {
        //Check ring and set path
//        ringConfiguration = ringDetectPipeline.getRingPosition();
        //TODO Implement path NONE and FOUR
        ringConfiguration = ringConfiguration.ONE;
        switch (ringConfiguration) {
            case NONE:
                telemetry.addLine("Go Path NONE");
//                path = new AutonomousPathAAsync(drive, wobbleArm, flywheel, collector, hopper);
                break;

            case ONE:
                telemetry.addLine("Go Path ONE");
                path = new AutonomousPathBAsync(drive, wobbleArm, flywheel, collector, hopper);
                break;

            case FOUR:
                telemetry.addLine("Go Path FOUR");
//                path = new AutonomousPathCAsync(drive, wobbleArm, flywheel, collector, hopper);
                break;
        }


        telemetry.addData("Ring position", ringConfiguration);
        telemetry.update();
    }

    @Override
    public void loop() {
        path.followPathAsync(telemetry);
    }


}
