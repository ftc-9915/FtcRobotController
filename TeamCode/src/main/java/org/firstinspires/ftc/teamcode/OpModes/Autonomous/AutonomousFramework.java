package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Common.RingPosition;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Hopper;
import org.firstinspires.ftc.teamcode.Vision.Camera;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.PoseLibrary;
import org.firstinspires.ftc.teamcode.Subsystems.Collector;
import org.firstinspires.ftc.teamcode.Subsystems.Drive.MecanumDrivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter.Flywheel;
import org.firstinspires.ftc.teamcode.Subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.Vision.VisionPipeline;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



@Autonomous(name = "AutonomousFramework", group = "test")
public class AutonomousFramework extends LinearOpMode {

    int state = 1;
    boolean A = false;
    boolean B = false;
    Camera camera;

    RingPosition ringConfiguration;

    DcMotor armMotor; // this stuff is going to be replaced by robot class later

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    boolean pathIsFinished = false;

    Servo clawServo;


    AutonomousPath path;

    MecanumDrivebase drive;
    WobbleArm wobbleArm;
    Flywheel flywheel;
    Collector collector;
    Hopper hopper;


    @Override
    public void runOpMode() throws InterruptedException {

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
        VisionPipeline ringDetectPipeline = new VisionPipeline();
        camera = new Camera(hardwareMap, ringDetectPipeline);

//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        VisionPipeline pipeline = new VisionPipeline();
//        webcam.setPipeline(pipeline);
//
//        //opens connection to camera asynchronously
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
//                FtcDashboard.getInstance().startCameraStream(webcam, 0);
//            }
//        });

        ringConfiguration =  ringDetectPipeline.getRingPosition();
        telemetry.addLine("Waiting for start");
        telemetry.update();


        waitForStart();

        while (opModeIsActive() && !pathIsFinished) {
            switch (ringConfiguration) {
                case NONE:
                    telemetry.addLine("Go Path NONE");
                    path = new AutonomousPathA(drive, wobbleArm, flywheel, collector, hopper);
                    break;

                case ONE:
                    telemetry.addLine("Go Path ONE");
                    path = new AutonomousPathB(drive, wobbleArm, flywheel, collector, hopper);

                case FOUR:
                    telemetry.addLine("Go Path FOUR");
                    path = new AutonomousPathC(drive, wobbleArm, flywheel, collector, hopper);
                    break;
            }

            //Returns true when path is finished
            pathIsFinished = path.followPath();

            telemetry.update();

        }
    }

    public void goToNextState() { state++; }
    public void goToState(int newState) { state = newState; }
    public void finishRoutine() {
        pathIsFinished = true;
        PoseLibrary.autoEndingPose = drive.getPoseEstimate();
    }
}
