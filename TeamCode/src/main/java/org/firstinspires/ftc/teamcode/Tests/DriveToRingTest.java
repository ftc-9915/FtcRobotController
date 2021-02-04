package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.PDController;
import org.firstinspires.ftc.teamcode.Examples.SkystoneDeterminationExample;
import org.firstinspires.ftc.teamcode.Vision.VisionPipelineDynamic;
import org.firstinspires.ftc.teamcode.drive.opmode.DriveVelocityPIDTuner;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Config
@TeleOp(name = "DriveToRingTest", group = "test")
public class DriveToRingTest extends OpMode {

    enum Mode {
        DRIVER_MODE,
        RING_FOLLOW_MODE
    }

    DcMotor armMotor;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    Servo clawServo;


    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;

    OpenCvCamera webcam;
    VisionPipelineDynamic pipeline;


    public static double kP = 5;
    public static double kD = 0;
    PDController pd;

    Mode mode = Mode.DRIVER_MODE;



    @Override
    public void init() {
        // Arm Motor
        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Chassis Motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        pd = new PDController(kP, kD);
        pd.setSetPoint(180);
        pd.setTolerance(10);

        // Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new VisionPipelineDynamic();
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

    }

    @Override
    public void loop() {

        telemetry.addData("Mode:", mode);

        switch (mode) {
            case DRIVER_MODE:
                // Chassis
                speed = -gamepad1.right_stick_y;
                strafe = gamepad1.right_stick_x;
                rotation = gamepad1.left_stick_x;

                leftFront.setPower(speed + strafe + rotation);
                leftBack.setPower(speed - strafe + rotation);
                rightBack.setPower(speed + strafe - rotation);
                rightFront.setPower(speed - strafe - rotation);

                if(gamepad1.a) {
                    mode = Mode.RING_FOLLOW_MODE;
                }
            case RING_FOLLOW_MODE:

                if(pipeline.maxRect != null){
                    double currentPosition = pipeline.maxRect.x;
                    double output = pd.calculate(currentPosition);

                    telemetry.addData("Current Position", currentPosition);
                    telemetry.addData("Output: ", output);

                    //when ring x value is too small / too far left, turn right
                    if (output > 0){
                        leftFront.setPower(output);
                        leftBack.setPower(output);
                        rightFront.setPower(-output);
                        rightBack.setPower(-output);
                    } else if (output < 0){
                        leftFront.setPower(-output);
                        leftBack.setPower(-output);
                        rightFront.setPower(output);
                        rightBack.setPower(output);
                    } else {
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }
                }

                if(gamepad1.a) {
                    mode = Mode.DRIVER_MODE;
                }
                if (gamepad2.b){
                    pd.reset();
                }
        }






    }


}
