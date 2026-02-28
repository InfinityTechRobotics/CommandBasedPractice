package org.firstinspires.ftc.teamcode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Shooter;

//@Configurable
@Disabled
@TeleOp
public class FlywheelTestMotorPID extends OpMode {
    Shooter shooter = new Shooter();

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    public boolean lastBPress;

    public DcMotorEx motorIntake, motorTransfer;
    public DcMotorEx motorFlywheel;

    public boolean lastRightBump, lastLeftBump;
    public boolean lastDpadUp, lastDpadDown;
    public boolean lastDpadLeft, lastDpadRight;

    public boolean intakeOn, transferOn;

    boolean stopAlreadyEngaged = false;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double TPS;

    double currentPos;

    PIDFCoefficients pidfModified;

    public static double INTAKE_POWER = 0.8;
    public static double INTAKE_CURRENT_ALERT = 6.;

    public static double TRANSFER_POWER = 0.8;
    public static double TRANSFER_CURRENT_ALERT = 6.;

    public static double FLYWHEEL_CURRENT_ALERT = 9.;

    double intakeCurrent, transferCurrent, flywheelCurrent, totalCurrent;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private static ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    private VoltageSensor battery;

    int i = 0;



    public void init() {

        shooter.init(hardwareMap);

        telemetry.setMsTransmissionInterval(11);

        shooter.centerServoTurret();
        currentPos = shooter.servoTurretGetPosition();

        shooter.closeServoStop();
        shooter.downServoPaddle();

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorTransfer = hardwareMap.get(DcMotorEx.class, "motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel.setCurrentAlert(FLYWHEEL_CURRENT_ALERT, CurrentUnit.AMPS);

        motorIntake.setCurrentAlert(INTAKE_CURRENT_ALERT, CurrentUnit.AMPS);

        motorTransfer.setCurrentAlert(TRANSFER_CURRENT_ALERT, CurrentUnit.AMPS);
        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    public void start() {
        timer.reset();
        prevTime = 0.;
    }

    public void loop() {

        // Control Direction of Intake and Transfer Motors
        if (gamepad2.dpad_up && !lastDpadUp) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);
            intakeOn = false;
            transferOn = false;
        }
        lastDpadUp = gamepad2.dpad_up;

        if (gamepad2.dpad_down && !lastDpadDown) {
            motorTransfer.setPower(0.);
            motorIntake.setPower(0.);
            motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
            motorTransfer.setDirection(DcMotorEx.Direction.REVERSE);
            intakeOn = false;
            transferOn = false;
        }
        lastDpadDown = gamepad2.dpad_down;

        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
        }

        lastRightBump = gamepad2.right_bumper;

        if (intakeOn) {
            motorIntake.setPower(INTAKE_POWER);
        } else {
            motorIntake.setPower(0.);
        }


        // Toggle transfer when left_bumper is pressed
        if (gamepad2.left_bumper && !lastLeftBump) {
            transferOn = !transferOn;
        }

        lastLeftBump = gamepad2.left_bumper;

        if (transferOn) {
            motorTransfer.setPower(TRANSFER_POWER);
        } else {
            motorTransfer.setPower(0.);
        }

        // Control Paddle Servo
        if (gamepad2.right_trigger > 0.25) {
            shooter.shootServoPaddle();
        } else {
            shooter.downServoPaddle();
        }

        // Control Servo Stop
        if (gamepad2.left_trigger > 0.25) {
            shooter.openServoStop();
            if (!stopAlreadyEngaged) {
            }
            stopAlreadyEngaged = true;
        } else {
            shooter.closeServoStop();
            stopAlreadyEngaged = false;
        }

        // Flywheel Speed Setting
        if (gamepad2.xWasPressed()) {
            targetRPM = 0.;
        } else if (gamepad2.bWasPressed()) {
            targetRPM = 2400.;
        } else if (gamepad2.aWasPressed()) {
            targetRPM -= 50.;
        } else if (gamepad2.yWasPressed()) {
            targetRPM += 50.;
        }

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);

        flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;


        // Intake Motor Current
        intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

        if (motorIntake.isOverCurrent()) {
            intakeOn = false;
        }

        // Transfer Motor Current
        transferCurrent = motorTransfer.getCurrent(CurrentUnit.AMPS);

        if (motorTransfer.isOverCurrent()) {
            transferOn = false;
        }

        // Flywheel Motor Current
        flywheelCurrent = motorFlywheel.getCurrent(CurrentUnit.AMPS);

        if (motorFlywheel.isOverCurrent()) {
            motorFlywheel.setVelocity(0.);
        }

        totalCurrent = intakeCurrent + transferCurrent + flywheelCurrent;

        i += 1;

        if (i % 100 == 0) {
            currentTime = timer.seconds();
            elapsedTime = currentTime - prevTime;
            prevTime = currentTime;
        }

        // Panels Telemetry Data
        panelsTelemetry.addData("Timer", timer.seconds());
        panelsTelemetry.addData("Elapsed Time (100 loops)", elapsedTime);
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("Intake Current", intakeCurrent);
        panelsTelemetry.addData("Transfer Current", transferCurrent);
        panelsTelemetry.addData("Flywheel Current", flywheelCurrent);
        panelsTelemetry.addData("Total Current", totalCurrent);
        panelsTelemetry.addData("Voltage", battery.getVoltage());

        panelsTelemetry.update(telemetry);

    }

}