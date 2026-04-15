package org.firstinspires.ftc.teamcode.Disabled;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Disabled
//@Configurable

@TeleOp
public class HoodedFlywheelOnlyTestMotorPIDPanels extends OpMode {

    private Servo servo;
    public DcMotorEx motorFlywheel1;
    public DcMotorEx motorFlywheel2;

    double servoPosition = 0.5;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double motorFlywheel2RPM = 0.0;
    double TPS;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private static ElapsedTime timer = new ElapsedTime();
    double currentTime, prevTime, elapsedTime;

    int i = 0;

    public static double NEW_P = 100.;   // 100.
    public static double NEW_I = 1.;    // 1.
    public static double NEW_D = 20.;    // 20.
    public static double NEW_F = 3.5;    // 3.5

    @Override
    public void init() {

        motorFlywheel1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motorFlywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFlywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        motorFlywheel2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motorFlywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFlywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        servo = hardwareMap.get(Servo.class, "servoHood");

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        // stuff for panels
        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    public void start() {
        timer.reset();
        prevTime = 0.;
    }

    @Override
    public void loop() {

        // Flywheel Speed Setting
        if (gamepad1.xWasPressed()) {
            targetRPM = 0.;
        } else if (gamepad1.bWasPressed()) {
            targetRPM = 2400.;
        } else if (gamepad1.aWasPressed()) {
            targetRPM -= 50.;
        } else if (gamepad1.yWasPressed()) {
            targetRPM += 50.;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            servoPosition -= 0.01;
        } else if (gamepad1.dpadRightWasPressed()) {
            servoPosition += 0.01 ;
        } else if (gamepad1.dpadUpWasPressed()) {
            servoPosition += 0.1;
        } else if (gamepad1.dpadDownWasPressed()) {
            servoPosition -= 0.1;
        }

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel1.setVelocity(TPS);
        motorFlywheel2.setVelocity(TPS);

        flywheelRPM = motorFlywheel1.getVelocity() / CPR * 60;
        motorFlywheel2RPM = motorFlywheel2.getVelocity() / CPR * 60;

        servo.setPosition(servoPosition);

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
        panelsTelemetry.addData("Flywheel1 RPM", flywheelRPM);
        panelsTelemetry.addData("Flywheel2 RPM", motorFlywheel2RPM);
        panelsTelemetry.addData("TPS", TPS);
        panelsTelemetry.addData("Servo Position", servoPosition);

//        panelsTelemetry.addData("Intake Current", intakeCurrent);
//        panelsTelemetry.addData("Transfer Current", transferCurrent);
//        panelsTelemetry.addData("Flywheel Current", flywheelCurrent);
//        panelsTelemetry.addData("Total Current", totalCurrent);
//        panelsTelemetry.addData("Voltage", battery.getVoltage());

        panelsTelemetry.update(telemetry);

//        // Telemetry Data
//        telemetry.addData("Timer", timer.seconds());
//        telemetry.addData("Elapsed Time (100 loops)", elapsedTime);
//        telemetry.addData("Target RPM", targetRPM);
//        telemetry.addData("Flywheel RPM", flywheelRPM);
//        telemetry.update();



    }


}


