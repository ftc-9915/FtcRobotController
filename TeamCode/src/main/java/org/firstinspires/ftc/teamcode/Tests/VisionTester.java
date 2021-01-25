package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionPipelineDynamic;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.teamcode.OpModes.Vision.VisionPipeline;


@Autonomous(name="Vision Test", group="test")
public class VisionTester extends LinearOpMode {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        VisionPipelineDynamic pipeline = new VisionPipelineDynamic();
        VisionPipeline pipeline = new VisionPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(() -> {
            /*
             * Tell the webcam to start streaming images to us! Note that you must make sure
             * the resolution you specify is supported by the camera. If it is not, an exception
             * will be thrown.
             *
             * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
             * supports streaming from the webcam in the uncompressed YUV image format. This means
             * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
             * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
             *
             * Also, we specify the rotation that the webcam is used in. This is so that the image
             * from the camera sensor can be rotated such that it is always displayed with the image upright.
             * For a front facing camera, rotation is defined assuming the user is looking at the screen.
             * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
             * away from the user.
             */
            webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            FtcDashboard.getInstance().startCameraStream(webcam, 0);
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
//            if(pipeline.isGoalVisible()) {
//                telemetry.addData("Rectangle Width", pipeline.goalRect.width);
//                telemetry.addData("Rectangle Height", pipeline.goalRect.height);
//                telemetry.addData("Goal Height (Center line)", pipeline.getGoalHeight());
//              //  telemetry.addData("Goal Height", pipeline.getGoalHeight());
//                //telemetry.addData("Horizontal Distance To Goal", pipeline.getXDistance());
//            } else {
//                telemetry.addLine("Goal not visible");
//            }
//            telemetry.addData("Corners visible",pipeline.isCornersVisible());

            telemetry.addData("Ring Position", pipeline.position);
            telemetry.addData("Cb Value", pipeline.getAnalysis());
            telemetry.update();

        }


//        telemetry.addData("Rectangle Cb Value", pipeline.avgCbValue);
//
//        telemetry.addData("Four cb error", pipeline.fourRingCbError);
//        telemetry.addData("Four dimension error", pipeline.fourRingDimensionError);
//        telemetry.addData("Four confidence value", pipeline.fourRingConfidence);
//
//        telemetry.addData("One cb error", pipeline.oneRingCbError);
//        telemetry.addData("One dimension error", pipeline.oneRingDimensionError);
//        telemetry.addData("One confidence value", pipeline.oneRingConfidence);
//
//        telemetry.addData("Hesitance", pipeline.hesitance);
//        telemetry.addData("Ring Position", pipeline.position);
//
//        telemetry.addData("Distance To Ring (inches)", pipeline.distanceToRing);

        telemetry.update();

    }
}
