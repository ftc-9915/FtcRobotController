package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.RingPosition;
import org.firstinspires.ftc.teamcode.Common.UtilMethods;
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

    public static AutonomousPathAsync pathA;
    public static AutonomousPathAsync pathB;
    public static AutonomousPathAsync pathC;


    public static RingPosition ringConfiguration;
    public static AutonomousPathAsync path;


    MecanumDrivebase drive;
    WobbleArm wobbleArm;
    Flywheel flywheel;
    Collector collector;
    Hopper hopper;
    
    private FtcDashboard dashboard;



    @Override
    public void init() {
        //Init Drive
        drive = new MecanumDrivebase(hardwareMap, 1.5);
        drive.setPoseEstimate(PoseLibrary.START_POS_BLUE_2);

        //Initialize Wobble Arm
        wobbleArm = new WobbleArm(hardwareMap);

        //Initialize Flywheel
        flywheel = new Flywheel(hardwareMap);

        //Initialize collector
        collector = new Collector(hardwareMap);

        //Initialize hopper
        hopper = new Hopper(hardwareMap);


        //set read mode to manual
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }


        //initialize paths ahead of time
        pathA =  new AutonomousPathAAsync(drive, wobbleArm, flywheel, collector, hopper);
        pathB = new AutonomousPathBAsync(drive, wobbleArm, flywheel, collector, hopper);
        pathC = new AutonomousPathCAsync(drive, wobbleArm, flywheel, collector, hopper);

//
//        Initialize webcam
        ringDetectPipeline = new VisionPipeline();
        camera = new Camera(hardwareMap, ringDetectPipeline);


        //logging

        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Desired RPM", 0);
        packet.put("RPM", (0));
        packet.put("Upper Bound", 5000);
        packet.put("Lower Bound", 0);

        dashboard.sendTelemetryPacket(packet);

    }

    //Called every 25 seconds
    @Override
    public void init_loop() {
        //Check ring and set path
        ringConfiguration = ringDetectPipeline.getRingPosition();

        telemetry.addData("Cb Value", ringDetectPipeline.getAnalysis());
        telemetry.addData("Ring position", ringConfiguration);
        telemetry.update();
    }

    @Override
    public void start() {
        switch (ringConfiguration) {
            case NONE:
                path = pathA;
                telemetry.addLine("Go Path A");
                break;

            case ONE:
                path = pathB;
                telemetry.addLine("Go Path B");
                break;

            case FOUR:
                path = pathC;
                telemetry.addLine("Go Path C");
                break;

            default:
                path = pathB;
                telemetry.addLine("Path B");
                break;
        }
    }

    @Override
    public void loop() {

        //Clear Bulk Cache at beginning of loop
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.clearBulkCache();
        }

        path.followPathAsync(telemetry);

    }


}
