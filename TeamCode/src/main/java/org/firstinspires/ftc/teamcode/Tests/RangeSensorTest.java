package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous (name = "RangeSensorTest", group = "test")
public class RangeSensorTest extends OpMode {
    Rev2mDistanceSensor rangeSensor;

    @Override
    public void init() {
        rangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "rangeSensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Range Inches", rangeSensor.getDistance(DistanceUnit.INCH));
    }
}
