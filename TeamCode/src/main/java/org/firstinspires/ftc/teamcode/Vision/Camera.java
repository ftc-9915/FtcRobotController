package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Common.RingPosition;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.Vision.VisionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {

    //Editable Constants
    public static double RING_STACK_POSITION = 10;

    OpenCvCamera webcam;
    OpenCvPipeline pipeline;

    Servo cameraServo;

    public Camera(HardwareMap hardwareMap, OpenCvPipeline pipeline)
    {
//        cameraServo.setPosition(RING_STACK_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        this.pipeline = pipeline;
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



}
