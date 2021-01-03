package org.firstinspires.ftc.teamcode.OpModes.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
            switch (state) {
                case 1: // Grab wobble goal
                    /*
                    clawServo.setPosition(CLAW_OPEN_POS);
                    armMotor.setTargetPosition(-500); i totally didn't forget how to write autonomous code
                    armMotor.setPower(1.0);
                    clawServo.setPosition(CLAW_CLOSE_POS);
                    armMotor.setTargetPosition(0);
                    armMotor.setPower(1.0);
                    goToNextState();
                     */
                    break;

                case 2: // Drive forward to rings
                    break;

                case 3: // Detect number of rings, determine A, B, C

                case 4: // Straighten robot
                    if (B) {
                        goToNextState(); // path B
                    } else {
                        goToState(7); // path A/C
                    }
                    break;

                case 5: // Path B: Go straight forward until 2 lines have been detected
                    break;

                case 6: // Path B: Go forward 12 inches
                    goToState(11); // this is the drop wobble goal state
                    break;

                case 7: // Path A/C: Strafe towards wall
                    if (A) {
                        goToNextState(); // path A
                    } else {
                        goToState(9); // path C
                    }
                break;

                case 8: // Path A: Go straight forward until 2 lines have been detected
                    goToState(11); // this is the drop wobble goal state
                    break;

                case 9: // Path C: Go straight forward until 4 lines have been detected
                    break;

                case 10: // Path C: Go forward 12 inches
                    break;

                case 11: // Release wobble goal
                    // clawServo.setPosition(CLAW_OPEN_POS);
                    break;

                case 12: // Back up behind white line
                    break;

                case 13: // Strafe towards center until 36 inches away from wall
                    break;

                case 14: // Launch rings
                    break;

                case 15: // Park on white line
                    break;

            }

        }
    }

    public void goToNextState() { state++; }
    public void goToState(int newState) { state = newState; }
}
