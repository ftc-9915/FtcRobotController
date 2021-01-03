package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Launcher Test", group="test")
public class LauncherTest extends OpMode {

    DcMotor launcherMotor;

    DcMotor collectorMotor;

    Servo liftServo;
    Servo pushServo;

    ElapsedTime timer = new ElapsedTime();
    Servo feederServo;

    // TODO: test these and edit with accurate values
    static final double SERVO_RESET_POS = 1.0;
    static final double SERVO_PUSH_POS = 0.0;

    double power;
    // Amount of adjustment to be made to the motor power
    double increment;
    // Ensures that the adjustments are made each time the gamepad buttons are pressed rather than each time through loop
    boolean buttonReleased;

    int direction = 1;

    static final double COLLECTOR_MOTOR_POWER = 1.0;
    static final double LIFT_UP_POS = 0.32;
    static final double LIFT_DOWN_POS = 0.28;
    static final double NOT_PUSH_POS = 0.68;
    static final double PUSH_POS = 0.52;

    boolean collectorPush;

    @Override
    public void init() {
        launcherMotor = hardwareMap.dcMotor.get("launcherMotor");
        // Make the motor's speed constant using encoders
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorMotor = hardwareMap.dcMotor.get("collectorMotor");
        launcherMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftServo = hardwareMap.servo.get("liftServo");
        pushServo = hardwareMap.servo.get("pushServo");

        feederServo = hardwareMap.servo.get("feederServo");
        feederServo.setPosition(SERVO_RESET_POS);

        // Initialization values
        power = 0.0;
        increment = 0.01;
        buttonReleased = true;
        direction = 1;
        collectorPush = false;

        // Starting position
        liftServo.setPosition(LIFT_DOWN_POS);
        pushServo.setPosition(NOT_PUSH_POS);

    }

    @Override
    public void loop() {
        // Controls and Information
        telemetry.addData("Motor Power", power);
        telemetry.addData("Power Adjustment Increment", increment);
        telemetry.addData("Motor Direction", direction);
        telemetry.addLine();
        telemetry.addLine("--- Controls ---");
        telemetry.addData("Push Ring With Servo)", "button X");
        telemetry.addData("Reset Servo", "button Y");
        telemetry.addData("Reverse Motor Direction", "D-pad left");
        telemetry.addData("Lower collector platform", "button X");
        telemetry.addData("Lift collector platform", "button Y");
        telemetry.addData("Push/retract collector servo", "D-pad right");
        telemetry.addLine(" - Motor Power -");
        telemetry.addData("Set Motor Power to Full Power (1.0)", "button A");
        telemetry.addData("Stop Motor (power 0.0)", "button B");
        telemetry.addData("Add Increment to Motor Power", "D-pad up");
        telemetry.addData("Subtract Increment to Motor Power", "D-pad down");
        telemetry.addLine("- Power Increment -");
        telemetry.addData("Increase Increment by 0.01", "Right Bumper");
        telemetry.addData("Decrease Increment by 0.01", "Left Bumper");

        // Lifts/Lowers the collecting platform
        if (gamepad1.y) {
            liftServo.setPosition(LIFT_UP_POS);
        }

        if (gamepad1.x) {
            liftServo.setPosition(LIFT_DOWN_POS);
        }


        // Pushes the stack then returns to original position
        if (gamepad1.dpad_right && buttonReleased) {
            pushServo.setPosition(PUSH_POS);
            collectorPush = true;
            timer.reset();
            buttonReleased = false;
        }

        if (collectorPush && timer.seconds() > 1) {
            pushServo.setPosition(NOT_PUSH_POS);
            collectorPush = false;
        }


        // Reverse motor direction (each time the button is pressed)
        if(gamepad1.dpad_left && buttonReleased) {
            direction *= -1;
        }

        // Run at full power or stop
        if(gamepad1.a) {
            power = 1.0;
        }
        if(gamepad1.b) {
            power = 0.0;
        }

        // Adjusting motor power based on increment value
        if(gamepad1.dpad_up && buttonReleased) {
            power = Range.clip(power + increment, 0.0, 1.0);
            buttonReleased = false;
        }
        if(gamepad1.dpad_down && buttonReleased) {
            power = Range.clip(power - increment, 0.0, 1.0);
            buttonReleased = false;
        }

        // Adjusting increment value
        // (this does not affect the motor power until the increment is applied to the motor power)
        if(gamepad1.right_bumper && buttonReleased) {
            increment += 0.01;
            buttonReleased = false;
        }
        if(gamepad1.left_bumper && buttonReleased) {
            increment = Math.max(increment - 0.01, 0.0);
            buttonReleased = false;
        }

        // Do not adjust values again until after buttons are released (and pressed again) so the
        // adjustments are made each time the gamepad buttons are pressed rather than each time through loop
        if(!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.left_bumper && !gamepad1.right_bumper && !gamepad1.dpad_left && !gamepad1.dpad_right) {
            buttonReleased = true;
        }

        launcherMotor.setPower(power * direction);
        collectorMotor.setPower(COLLECTOR_MOTOR_POWER);

    }
}
