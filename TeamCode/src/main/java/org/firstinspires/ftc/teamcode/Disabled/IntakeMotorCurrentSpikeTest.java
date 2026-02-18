package org.firstinspires.ftc.teamcode.Disabled;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled
@Configurable
@TeleOp
public class IntakeMotorCurrentSpikeTest extends OpMode {

    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public DcMotorEx motorIntake;

    double intakeRPM;
    double intakeCurrent;

    double CPR = 103.8;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

    public boolean lastRightBump;
    public boolean intakeOn;

    double INTAKE_POWER = 0.8;

    boolean aAlreadyPressed;
    boolean yAlreadyPressed;

    public static double CURRENT_ALERT = 6.;


    private static ElapsedTime timer = new ElapsedTime();

    public void init() {

        telemetry.setMsTransmissionInterval(11);

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer.reset();

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);


    }

    public void loop() {

        // Toggle intake when right_bumper is pressed
        if (gamepad2.right_bumper && !lastRightBump) {
            intakeOn = !intakeOn;
        }

        lastRightBump = gamepad2.right_bumper;


        if (gamepad2.x) {
            INTAKE_POWER = 0.;
        } else if (gamepad2.b) {
            INTAKE_POWER = 0.8;
        } else if (gamepad2.a && !aAlreadyPressed) {
            INTAKE_POWER -= 0.1;
        } else if (gamepad2.y && !yAlreadyPressed) {
            INTAKE_POWER += 0.1;
        }

        aAlreadyPressed = gamepad2.a;
        yAlreadyPressed = gamepad2.y;



        if (intakeOn) {
            motorIntake.setPower(INTAKE_POWER);
        } else {
            motorIntake.setPower(0.);
        }



        // Calculate Intake motor velocity
        intakeRPM = motorIntake.getVelocity() / CPR * 60;

        intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

        motorIntake.setCurrentAlert(CURRENT_ALERT, CurrentUnit.AMPS);

        if (motorIntake.isOverCurrent()) {
            intakeOn = false;
        }


        // Telemetry Data
        telemetry.addData("Timer", timer.seconds());
        telemetry.addData("Intake On", intakeOn);
        telemetry.addData("Intake RPM", intakeRPM);
        telemetry.addData("Intake Motor Power", motorIntake.getPower());
        telemetry.addData("Intake Motor Current", intakeCurrent);
        telemetry.addData("Intake Motor Current Alert Level", motorIntake.getCurrentAlert(CurrentUnit.AMPS));
        telemetry.addData("Intake Motor Current Alert", motorIntake.isOverCurrent());

        telemetry.update();


        // Panels Telemetry Data
        panelsTelemetry.debug("Intake RPM: $intakeRPM");
        panelsTelemetry.debug("Intake Current: $intakeCurrent");
        panelsTelemetry.addData("Intake RPM", intakeRPM);
        panelsTelemetry.addData("Intake Current", intakeCurrent);

        panelsTelemetry.update(telemetry);

    }

    public void stop() {

    }

}