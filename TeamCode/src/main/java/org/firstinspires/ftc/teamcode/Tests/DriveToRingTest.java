package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Examples.SkystoneDeterminationExample;
import org.firstinspires.ftc.teamcode.Vision.VisionPipelineDynamic;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@TeleOp(name = "DriveToRingTest", group = "test")
public class DriveToRingTest extends OpMode {
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

        // Chassis
        speed = -gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotation = gamepad1.left_stick_x;

        leftFront.setPower(speed + strafe + rotation);
        leftBack.setPower(speed - strafe + rotation);
        rightBack.setPower(speed + strafe - rotation);
        rightFront.setPower(speed - strafe - rotation);

        if(gamepad1.dpad_up) {
            driveToRing();
        }

    }

    public void driveToRing(){
        if (pipeline.position == VisionPipelineDynamic.RingPosition.NONE){
            telemetry.addLine("Didn't find ring");
        } else {
            rotateToRing();
            moveForwardToRing();
        }
    }

    public void rotateToRing(){
    //turn robot until ring is within 30 pixels of horizontal center (180px)
        //ring to the left of center, so turn right
        if(pipeline.maxRect.x < 150){
            telemetry.addLine("Ring to the left");
            leftBack.setPower(0.2);
            leftFront.setPower(0.2);
            rightFront.setPower(-0.2);
            rightBack.setPower(-0.2);
        }
        //ring to the right of center, so turn left
        else if (pipeline.maxRect.x >  210){
            telemetry.addLine("Ring to the right");
            leftBack.setPower(-0.2);
            leftFront.setPower(-0.2);
            rightFront.setPower(0.2);
            rightBack.setPower(0.2);
        }

        telemetry.update();

    }
    public void moveForwardToRing(){
        double distance =  pipeline.distanceToRing;
        //TODO: Implement move forward after Robot.java methods for distance are done
    }
}
