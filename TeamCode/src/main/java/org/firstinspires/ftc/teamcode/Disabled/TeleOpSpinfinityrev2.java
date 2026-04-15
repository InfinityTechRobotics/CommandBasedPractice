package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class TeleOpSpinfinityrev2 extends OpMode {

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    double DRIVE_POWER_FACTOR = 0.7;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 0.9;

    double powerFactor = DRIVE_POWER_FACTOR;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx backLeftMotor;

    public DcMotorEx motorIntake;
    public DcMotorEx motorFlywheel;
    public DcMotorEx motorTurret;


    public boolean intakeOn, transferOn;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    public static double INTAKE_POWER = 0.8;

    public Servo servoStop;
    public Servo servoPaddleLeft;

    double SERVO_STOP_OPEN_POS = 0.63;
    double SERVO_STOP_CLOSE_POS = 0.37;

    double SERVO_PADDLE_SHOOT_POS = 0.85;
    double SERVO_PADDLE_DOWN_POS = 0.5;

    public void init() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "motorRearLeft");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "motorRearRight");

        frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        servoStop.setPosition(SERVO_STOP_CLOSE_POS);
        servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
       /* outputAngleServo = hardwareMap.get(Servo.class, "outputAngleServo");
        outputAngleServo.setDirection(Servo.Direction.FORWARD);
        outputAngleServo.setPosition(0.47); */

        targetRPM = 2400.;

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
        motorFlywheel.setVelocity(TPS);

        // Control Direction of Intake and Transfer Motors
        if (gamepad1.dpadUpWasPressed()) {
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
            intakeOn = false;
            transferOn = false;
        }

        if (gamepad1.dpadDownWasPressed()) {
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
            intakeOn = false;
            transferOn = false;
        }

        // Toggle intake when right_bumper is pressed
        if (gamepad2.rightBumperWasPressed()) {
            intakeOn = !intakeOn;
        }

        if (intakeOn) {
            motorIntake.setPower(INTAKE_POWER);
        } else {
            motorIntake.setPower(0.);
        }

        if (gamepad2.left_trigger > 0.25) {
            servoStop.setPosition(SERVO_STOP_OPEN_POS);
        } else {
            servoStop.setPosition(SERVO_STOP_CLOSE_POS);
        }

        if (gamepad2.right_trigger > 0.25) {
            servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS);
        } else {
            servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
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