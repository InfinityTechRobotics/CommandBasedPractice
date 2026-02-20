package org.firstinspires.ftc.teamcode.SKR;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;

import java.util.List;

//@Configurable
@TeleOp
public class TeleOpBlueGoggles extends OpMode {

    public static double NEW_P = 20.;   // 10.
    public static double NEW_I = 0.5;    // 3.
    public static double NEW_D = 5.;    // 0.
    public static double NEW_F = 12;    // 0.

    double DRIVE_POWER_FACTOR = 0.7;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 0.9;

    double powerFactor = DRIVE_POWER_FACTOR;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx backLeftMotor;

    public DcMotorEx motorIntake, motorTransfer;
    public DcMotorEx motorFlywheelLeft, motorFlywheelRight;

    public Servo outputAngleServo;

    public boolean intakeOn, transferOn;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    public static double INTAKE_POWER = 0.4;
    public static double TRANSFER_POWER = 1.0;


    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorIntake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        motorTransfer = hardwareMap.get(DcMotorEx.class, "intakeBeltMotor");
        motorFlywheelLeft = hardwareMap.get(DcMotorEx.class, "leftShooter");
        motorFlywheelRight = hardwareMap.get(DcMotorEx.class, "rightShooter");

        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheelLeft.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheelRight.setDirection(DcMotorEx.Direction.REVERSE);

        motorFlywheelLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFlywheelRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFlywheelLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheelRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheelLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
        motorFlywheelRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        outputAngleServo = hardwareMap.get(Servo.class, "outputAngleServo");
        outputAngleServo.setDirection(Servo.Direction.FORWARD);
        outputAngleServo.setPosition(0.47);


    }

    public void loop() {

        // Driver Controls
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        moveRobotFC(y, x, rx, 0., powerFactor);


        // manually set RPM distance
        if (gamepad1.x) {
            targetRPM = 0.;
        } else if (gamepad1.b) {
            targetRPM = 2400.;
        } else if (gamepad1.aWasPressed()) {
            targetRPM -= 50.;
        } else if (gamepad1.yWasPressed()) {
            targetRPM += 50.;
        }

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheelLeft.setVelocity(TPS);
        motorFlywheelRight.setVelocity(TPS);

        // Control Direction of Intake and Transfer Motors
        if (gamepad1.dpadUpWasPressed()) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
            motorTransfer.setDirection(DcMotorEx.Direction.REVERSE);
            intakeOn = false;
            transferOn = false;
        }

        if (gamepad1.dpadDownWasPressed()) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);
            intakeOn = false;
            transferOn = false;
        }

        // Toggle intake when right_bumper is pressed
        if (gamepad1.rightBumperWasPressed()) {
            intakeOn = !intakeOn;
        }

        if (intakeOn) {
            motorIntake.setPower(INTAKE_POWER);
        } else {
            motorIntake.setPower(0.);
        }


        // Toggle transfer when left_bumper is pressed
        if (gamepad1.leftBumperWasPressed()) {
            transferOn = !transferOn;
        }

        if (transferOn) {
            motorTransfer.setPower(TRANSFER_POWER);
        } else {
            motorTransfer.setPower(0.);
        }


        // Telemetry Data
        telemetry.addData("Drive Power Factor", powerFactor);
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Transfer On", transferOn);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);

        telemetry.update();



    }


    public void moveRobotFC (double y, double x, double rx, double botHeading, double powerFactor) {

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(powerFactor * frontLeftPower);
        backLeftMotor.setPower(powerFactor * backLeftPower);
        frontRightMotor.setPower(powerFactor * frontRightPower);
        backRightMotor.setPower(powerFactor * backRightPower);
    }

}