package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Collector Test", group="test")
public class CollectorTest extends OpMode {

    DcMotor launcherMotor;

    DcMotor collectorMotor;

    Servo liftServo;
    Servo pushServo;

    Servo feederServo;

    // TODO: test these and edit with accurate values
    static final double SERVO_RESET_POS = 1.0;
    static final double SERVO_PUSH_POS = 0.0;

    double launcherPower;
    double collectorPower;
    boolean collectorPush;

    // Ensures that the adjustments are made each time the gamepad buttons are pressed rather than each time through loop
    boolean buttonReleased;

    static final double LIFT_UP_POS = 0.32;
    static final double LIFT_DOWN_POS = 0.28;
    static final double NOT_PUSH_POS = 0.68;
    static final double PUSH_POS = 0.52;

    @Override
    public void init() {
        launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor = hardwareMap.dcMotor.get("collectorMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftServo = hardwareMap.servo.get("liftServo");
        pushServo = hardwareMap.servo.get("pushServo");

        feederServo = hardwareMap.servo.get("feederServo");
        feederServo.setPosition(SERVO_RESET_POS);

        // Initialization values
        launcherPower = 0.0;
        collectorPower = 1.0;
        collectorPush = false;
        buttonReleased = true;

        // Starting position
        liftServo.setPosition(LIFT_DOWN_POS);
        pushServo.setPosition(NOT_PUSH_POS);

    }

    @Override
    public void loop() {
        // Controls and Information
        telemetry.addLine("--- Controls (Gamepad 1) ---");
        telemetry.addData("Turn collector on", "Button A");
        telemetry.addData("Turn collector off", "Button B");
        telemetry.addLine();
        telemetry.addLine("--- Controls (Gamepad 2) ---");
        telemetry.addData("Turn launcher on/off", "Button X");
        telemetry.addData("Push/retract collector servo", "Button Y");
        telemetry.addData("Lower collector platform", "Left Bumper");
        telemetry.addData("Lift collector platform", "Right Bumper");

        // Placeholder for chassis code

        // Turns collector on/off
        if (gamepad1.a) {
            collectorPower = 1.0;
        }

        if (gamepad1.b) {
            collectorPower = 0.0;
        }

        // Turns launcher on/off
        if (gamepad2.x) {
            if (launcherPower == 0.0) {
                launcherPower = -1.0;
            } else {
                launcherPower = 0.0;
            }
        }

        // Pushes/retracts collector servo
        if (gamepad2.y) {
            if (collectorPush) {
                pushServo.setPosition(NOT_PUSH_POS);
                collectorPush = false;
            } else {
                pushServo.setPosition(PUSH_POS);
                collectorPush = true;
            }
        }

        // Lifts/Lowers the collecting platform
        if (gamepad2.left_bumper && buttonReleased) {
            liftServo.setPosition(LIFT_DOWN_POS);
            buttonReleased = false;
        }

        if (gamepad2.right_bumper && buttonReleased) {
            liftServo.setPosition(LIFT_UP_POS);
            buttonReleased = false;
        }


        // Do not adjust values again until after buttons are released (and pressed again) so the
        // adjustments are made each time the gamepad buttons are pressed rather than each time through loop
        if(!gamepad2.left_bumper && !gamepad2.right_bumper) {
            buttonReleased = true;
        }

        launcherMotor.setPower(launcherPower);
        collectorMotor.setPower(collectorPower);

    }
}
