package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Examples.SkystoneDeterminationExample;
import org.firstinspires.ftc.teamcode.Vision.VisionPipelineDynamic;
import org.firstinspires.ftc.teamcode.drive.opmode.DriveVelocityPIDTuner;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

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

    double k_p = 1;
    double p = 0;
    double current_error = 0;
    //max error is 360 - 180
    double MAX_ERROR = 180;



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

        // Vision
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new VisionPipelineDynamic();
        webcam.setPipeline(pipeline);

    }

    @Override
    public void loop() {
        Mode mode = Mode.DRIVER_MODE;

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
                 current_error = 180 - pipeline.maxRect.x;
                 p = k_p * current_error;

                 //if current error is negative move left
                if (current_error < -10){
                    rotation = p / MAX_ERROR;
                }
                //if current error is positive move right
                else if (current_error > 10){
                    rotation = p / MAX_ERROR;
                }

                leftFront.setPower(speed + strafe + rotation);
                leftBack.setPower(speed - strafe + rotation);
                rightBack.setPower(speed + strafe - rotation);
                rightFront.setPower(speed - strafe - rotation);

                if(gamepad1.a) {
                     k_p = 0;
                     p = 0;
                     current_error = 0;
                    mode = Mode.DRIVER_MODE;
                }
        }




    }


}
