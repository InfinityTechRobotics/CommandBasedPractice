package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Disabled
@TeleOp
public class DriveTrialTeleOpSRHF extends OpMode {
    Drive2 drive = new Drive2();

    double DRIVE_POWER_FACTOR = 0.8;
    double DRIVE_POWER_FACTOR_LOW = 0.5;
    double DRIVE_POWER_FACTOR_HIGH = 1;
    double powerFactor = DRIVE_POWER_FACTOR;

    public DcMotor motorIntake, motorTransfer, motorFlywheel;
    public Servo servoPaddleLeft, servoPaddleRight;
    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;

    private VoltageSensor battery;


    public boolean intakeOn, transferOn;


    public void init() {

        drive.init(hardwareMap);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.dcMotor.get("motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");
        servoPaddleRight = hardwareMap.servo.get("servoPaddleRight");

    }
    public void loop () {

        // Driver Controls

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


        // Operator Controls

        // Set Flywheel Power
        if (gamepad2.a) {
            motorFlywheel.setPower(0.);
        } else if (gamepad2.b) {
            motorFlywheel.setPower(0.6);
        } else if (gamepad2.x) {
            motorFlywheel.setPower(0.75);
        } else if (gamepad2.y) {
            motorFlywheel.setPower(0.9);
        } else {
            // dynamically set flywheel speed based off of voltage
            motorFlywheel.setPower(1.5715 - (0.07 * (battery.getVoltage())));
        }


        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                motorIntake.setPower(0.8);
            } else {
                motorIntake.setPower(0.);
            }
        }

        lastRightBump = gamepad2.right_bumper;

        // Toggle transfer when left_bumper is pressed
        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
            if (transferOn) {
                motorTransfer.setPower(0.8);
            } else {
                motorTransfer.setPower(0.);
            }
        }
        lastLeftBump = gamepad2.left_bumper;

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpad_up && !lastDpadUp) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        lastDpadDown = gamepad2.dpad_down;

        // Control Paddle Servo
        if (gamepad2.right_trigger > 0.25) {
//            servoPaddleLeft.setPosition(0.45);
            servoPaddleRight.setPosition(0.22);
            motorTransfer.setPower(0.);
        } else {
            servoPaddleLeft.setPosition(0.65);
            servoPaddleRight.setPosition(0.08);
        }


        // Telemetry Data
        telemetry.addData("Drive Power Factor", powerFactor);
        telemetry.addData("Intake Motor Power", motorIntake.getPower());
        telemetry.addData("Transfer Motor Power", motorTransfer.getPower());
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("Paddle Servo Position", servoPaddleLeft.getPosition());

        telemetry.update();

    }
}