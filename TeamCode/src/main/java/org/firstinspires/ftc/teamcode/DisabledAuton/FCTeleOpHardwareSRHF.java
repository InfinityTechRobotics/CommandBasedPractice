package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Disabled.Drive2;

@Disabled
@TeleOp
public class FCTeleOpHardwareSRHF extends OpMode {
    Drive2 drive = new Drive2();
    DcMotor motorFlywheel, motorTransfer, motorIntake;
    Servo servoPaddleLeft;

    double servoPos = 0.3;

    double DRIVE_POWER_FACTOR = 0.8;
    double DRIVE_POWER_FACTOR_LOW = 0.5;
    double DRIVE_POWER_FACTOR_HIGH = 1;
    double motorPower = 0.;

    double powerFactor = DRIVE_POWER_FACTOR;
    boolean lastLeftBump, lastRightBump, intakeOn, transferOn, dPadLeftAlreadyPressed, dPadRightAlreadyPressed;






    public void init() {

        drive.init(hardwareMap);
        motorFlywheel = hardwareMap.dcMotor.get("motorFlywheel");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.left_stick_button) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        drive.moveRobotRC(y, x, rx, powerFactor);

        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                motorIntake.setPower(0.75);
            } else {
                motorIntake.setPower(0.);
            }
        }
        lastRightBump = gamepad2.right_bumper;

        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
            if (transferOn) {
                motorTransfer.setPower(0.75);
            } else {
                motorTransfer.setPower(0.);
            }
        }
        lastLeftBump = gamepad2.left_bumper;

        if (gamepad2.dpad_up) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
            motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        if (gamepad2.dpad_down) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        if (gamepad2.y) {
            motorFlywheel.setPower(0.8);
        } else if (gamepad2.b) {
            motorFlywheel.setPower(0.65);
        } else if (gamepad2.x) {
            motorFlywheel.setPower(0.4);
        } else if (gamepad2.a) {
            motorFlywheel.setPower(0.);
        }

       if (gamepad2.right_trigger > 0.25) {
           servoPaddleLeft.setPosition(0.5);
       } else {
            servoPaddleLeft.setPosition(0.7);
        }

        if (gamepad2.dpad_left && !dPadLeftAlreadyPressed) {
            servoPos -= 0.01;
        } else if (gamepad2.dpad_right && !dPadRightAlreadyPressed) {
            servoPos += 0.01;
        }

        servoPaddleLeft.setPosition(servoPos);


        telemetry.addData("Directions","Right bumper controls the intake press it once and then again to turn it on and off and same thing for transfer but with right bumper and a b x y control the shooter's power");
        telemetry.addData("Intake: ",intakeOn);
        telemetry.addData("Transfer: ",transferOn);
        telemetry.addData("Flywheel Power: ",motorFlywheel.getPower());
        telemetry.update();


    }
}