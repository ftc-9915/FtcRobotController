package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Arm Test", group = "test")
public class ArmTest extends OpMode {
    DcMotor armMotor;

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;

    Servo clawServo;

    static final int ARM_INCREMENT = 3;
    static final double CLAW_OPEN_POS = 0.7;
    static final double CLAW_CLOSE_POS = 0.15;

    int currentArmPosition = 0;
    double armMultiplier = 0.0;

    double speed = 0.0;
    double strafe = 0.0;
    double rotation = 0.0;

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

        // Claw Servo
        clawServo = hardwareMap.servo.get("clawServo");
        clawServo.setPosition(CLAW_CLOSE_POS);
    }

    @Override
    public void loop() {
        telemetry.addLine("Controls:");
        telemetry.addData("Open Claw","button A");
        telemetry.addData("Close Claw", "button B");
        telemetry.addData("Arm", "right trigger (forward), left trigger (backward)");
        telemetry.addData("Arm Position", currentArmPosition);


        // Claw
        if(gamepad1.a) {
            clawServo.setPosition(CLAW_OPEN_POS);
        }
        if(gamepad1.b) {
            clawServo.setPosition(CLAW_CLOSE_POS);
        }

        // Arm
        armMultiplier = gamepad1.left_trigger - gamepad1.right_trigger;

        currentArmPosition += ARM_INCREMENT * armMultiplier;
        armMotor.setTargetPosition(currentArmPosition);
        armMotor.setPower(1.0);

        // Chassis
        speed = -gamepad1.right_stick_y;
        strafe = gamepad1.right_stick_x;
        rotation = gamepad1.left_stick_x;

        leftFront.setPower(speed + strafe + rotation);
        leftBack.setPower(speed - strafe + rotation);
        rightBack.setPower(speed + strafe - rotation);
        rightFront.setPower(speed - strafe - rotation);

    }
}
