package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Vision.BlueGoalVisionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="RR Test", group="test")
public class RoadrunnerAutoTest extends LinearOpMode {

    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize webcam
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        VisionPipelineDynamic pipeline = new VisionPipelineDynamic();
        BlueGoalVisionPipeline pipeline = new BlueGoalVisionPipeline();
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

       //Create RR paths
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPoseBlue2 = new Pose2d(-63.0, 24.0, Math.toRadians(0.0));
        Pose2d zoneCPoseBlue = new Pose2d(48.0, 48.0, Math.toRadians(20.0));
        Pose2d parkingPosition =  new Pose2d(12.0, 48.0, Math.toRadians(0.0));

        Trajectory myTrajectory = drive.trajectoryBuilder(startPoseBlue2)
                .splineTo(zoneCPoseBlue.vec(), zoneCPoseBlue.getHeading())
                .lineToSplineHeading(parkingPosition)
                .build();

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {
            if(pipeline.isGoalVisible()) {
                telemetry.addData("Rectangle Width", pipeline.goalRect.width);
                telemetry.addData("Rectangle Height", pipeline.goalRect.height);
                telemetry.addData("Pitch", pipeline.getGoalPitch());
                telemetry.addData("Yaw", pipeline.getGoalYaw());
                telemetry.addData("Diagonal Distance To Goal", pipeline.getGoalDistance());
                telemetry.addData("Horizontal Distance TO Goal", pipeline.getXDistance());
            } else {
                telemetry.addLine("Goal not visible");
            }

            drive.followTrajectory(myTrajectory);


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
