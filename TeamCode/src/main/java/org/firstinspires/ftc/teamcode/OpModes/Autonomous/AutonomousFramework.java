package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.RingPosition;
import org.firstinspires.ftc.teamcode.OpModes.Vision.VisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionPipelineDynamic;
import org.firstinspires.ftc.teamcode.drive.CoordinateConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = "AutonomousFramework", group = "test")
public class AutonomousFramework extends LinearOpMode {

    int state = 1;
    boolean A = false;
    boolean B = false;
    OpenCvCamera webcam;

    RingPosition ringConfiguration;

    DcMotor armMotor; // this stuff is going to be replaced by robot class later

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    boolean firstTime = true;

    Servo clawServo;

    static final int ARM_INCREMENT = 3;
    static final double CLAW_OPEN_POS = 0.7;
    static final double CLAW_CLOSE_POS = 0.15;

    AutonomousPathA pathA = new AutonomousPathA();
    AutonomousPathB pathB = new AutonomousPathB();
    AutonomousPathC pathC = new AutonomousPathC();

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

        //Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        VisionPipeline pipeline = new VisionPipeline();
        webcam.setPipeline(pipeline);

        //opens connection to camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }
        });

        ringConfiguration =  pipeline.position;
        telemetry.addLine("Waiting for start");
        telemetry.addData("Ring Config", ringConfiguration);
        telemetry.update();

//        armMotor = hardwareMap.dcMotor.get("armMotor"); // this stuff is going to be replaced by robot class later
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setTargetPosition(0);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        clawServo = hardwareMap.servo.get("clawServo");
//        clawServo.setPosition(CLAW_CLOSE_POS);


        waitForStart();

        while (opModeIsActive()) {
        // TODO: For each path, robot repeats the path after completing it rather than stopping.
            switch (ringConfiguration) {
                case NONE:
                    telemetry.addLine("Go Path NONE");
                    //Execute path A
                    pathA.followPath(drive, armMotor, clawServo);
                case ONE:
                    telemetry.addLine("Go Path ONE");
                    //Execute path B
                    pathB.followPath(drive, armMotor, clawServo);

                case FOUR:
                    telemetry.addLine("Go Path FOUR");
                    //Execute path C
                    pathC.followPath(drive, armMotor, clawServo);

                default:
                    ringConfiguration = pipeline.position;
            }

            telemetry.update();


        }
    }

    public void goToNextState() { state++; }
    public void goToState(int newState) { state = newState; }
}
