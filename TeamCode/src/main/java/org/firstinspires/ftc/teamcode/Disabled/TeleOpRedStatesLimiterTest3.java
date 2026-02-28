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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
//@Configurable
@TeleOp
public class TeleOpRedStatesLimiterTest3 extends OpMode {

    double currentPowerL = 0.;
    double currentPowerR = 0.;

    double previousPowerL = 0.;
    double previousPowerR = 0.;
    private static double MAX_CHANGE = 0.2;

    double DRIVE_POWER_FACTOR = 0.95;
    double DRIVE_POWER_FACTOR_LOW = 0.6;
    double DRIVE_POWER_FACTOR_HIGH = 1;

    double powerFactor = DRIVE_POWER_FACTOR;

    public DcMotorEx motorFlywheel;

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx backLeftMotor;

    boolean autoRPM = true;

    double targetRPM = 0.;
    double flywheelRPM = 0.;

    double CPR = 28.;
    double TPS;

    PIDFCoefficients pidfModified;

    public static double FLYWHEEL_CURRENT_ALERT = 9.;

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private VoltageSensor battery;


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

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel.setCurrentAlert(FLYWHEEL_CURRENT_ALERT, CurrentUnit.AMPS);

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    public void loop() {

        double yl = -gamepad1.left_stick_y;

        double yr = -gamepad1.right_stick_y;

//        currentPower = y;
        currentPowerL = limiter(yl, previousPowerL);
        currentPowerR = limiter(yr, previousPowerR);

        if (gamepad1.left_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        tankMoveRobotL(currentPowerL, powerFactor);
        tankMoveRobotR(currentPowerR, powerFactor);

        // Calculate and set flywheel motor velocity
        targetRPM = 2400;
        TPS = targetRPM / 60. * CPR;

        motorFlywheel.setVelocity(TPS);

        previousPowerL = currentPowerL;
        previousPowerR = currentPowerR;

        telemetry.addData("Current Left Power", currentPowerL);
        telemetry.addData("Current Right Power", currentPowerR);
        telemetry.addData("Previous Right Power", previousPowerR);
        telemetry.addData("Previous Left Power", previousPowerL);


        //         Panels Telemetry Data
        panelsTelemetry.addData("Flywheel Current", motorFlywheel.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("FL Current", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("FR Current", frontRightMotor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("RL Current", backLeftMotor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("RR Current", backRightMotor.getCurrent(CurrentUnit.AMPS));
        panelsTelemetry.addData("Voltage", battery.getVoltage());
        panelsTelemetry.addData("FL Power", frontLeftMotor.getPower());
        panelsTelemetry.addData("RL Power", backLeftMotor.getPower());
        panelsTelemetry.addData("FR Power", frontRightMotor.getPower());
        panelsTelemetry.addData("RR Power", backRightMotor.getPower());
        panelsTelemetry.update(telemetry);


    }

    double limiter(double targetPower, double previousPower) {
        if (Math.abs(targetPower - previousPower) > MAX_CHANGE) { // Can slew
            if (targetPower < previousPower) {
                return previousPower - MAX_CHANGE;
            } else if (targetPower > previousPower) {
                return previousPower + MAX_CHANGE;
            }
        }
        return targetPower; // Close enough that you can just use input
    }



    public void tankMoveRobotL (double currentPowerL, double powerFactor) {

        double frontLeftPower = currentPowerL;
        double backLeftPower = currentPowerL;

        frontLeftMotor.setPower(powerFactor * frontLeftPower);
        backLeftMotor.setPower(powerFactor * backLeftPower);
    }

    public void tankMoveRobotR (double currentPowerR, double powerFactor) {

        double frontRightPower = currentPowerR;
        double backRightPower = currentPowerR;

        frontRightMotor.setPower(powerFactor * frontRightPower);
        backRightMotor.setPower(powerFactor * backRightPower);
    }


}





