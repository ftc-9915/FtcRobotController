package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.RingPosition;
import org.firstinspires.ftc.teamcode.Vision.VisionPipelineDynamic;
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

    Servo clawServo;

    static final int ARM_INCREMENT = 3;
    static final double CLAW_OPEN_POS = 0.7;
    static final double CLAW_CLOSE_POS = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        VisionPipelineDynamic pipeline = new VisionPipelineDynamic();
        webcam.setPipeline(pipeline);

        //opens connection to camera asynchronously
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });

        telemetry.addLine("Waiting for start");
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

            switch (ringConfiguration) {
                case NONE:
                    //Execute path A
                case ONE:
                    //Execute path B
                case FOUR:
                    //Execute path C
                default:
                    ringConfiguration = pipeline.position;
            }

        }
    }

    public void goToNextState() { state++; }
    public void goToState(int newState) { state = newState; }
}
