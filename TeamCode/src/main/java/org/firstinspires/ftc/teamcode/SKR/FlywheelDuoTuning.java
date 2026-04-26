package org.firstinspires.ftc.teamcode.SKR;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.ShooterSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.SpintakeSpinfinity;

@Disabled
//@Configurable
@TeleOp
public class FlywheelDuoTuning extends OpMode {


    ShooterSpinfinityDuo shooter = new ShooterSpinfinityDuo();
    SpintakeSpinfinity spintake = new SpintakeSpinfinity();
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public static double NEW_P = 10.;   // 10.
    public static double NEW_I = 3.;    // 3.
    public static double NEW_D = 0.;    // 0.
    public static double NEW_F = 0.;    // 0.

    public DcMotorEx motorFlywheel;
    public DcMotorEx motorFlywheel2;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double targetRPM = 0.;
    double flywheelRPM = 0.;
    double flywheelRPM2 = 0.;
    double TPS;

    PIDFCoefficients pidfModified;
    PIDFCoefficients pidfModified2;

    public boolean intakeOn;

    public boolean prevIntake;

    double shootingTime = 0.;
    private Timer pathTimer, opmodeTimer;

    private int pathState;

    public static double FAR_ZONE_HOOD_POS = 0.5;

    boolean prevRightTrigger = false;
    boolean prevLeftTrigger = false;


    public void init() {

        shooter.init(hardwareMap);
        spintake.init(hardwareMap);

        telemetry.setMsTransmissionInterval(11);

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel2 = hardwareMap.get(DcMotorEx.class, "motorFlywheel2");

        motorFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel2.setDirection(DcMotorEx.Direction.FORWARD);
        motorFlywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        panelsTelemetry.debug("Init was ran!");
        panelsTelemetry.update(telemetry);

    }

    public void loop() {

        autonomousPathUpdate();

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
        motorFlywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);
        pidfModified2 = motorFlywheel2.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set Flywheel Velocity
        if (gamepad2.xWasPressed()) {
            targetRPM = 3250.;
        } else if (gamepad2.bWasPressed()) {
            targetRPM = 2400.;
        } else if (gamepad2.aWasPressed()) {
            targetRPM -= 100.;
        } else if (gamepad2.yWasPressed()) {
            targetRPM += 100.;
        }

        if (gamepad2.dpadLeftWasPressed()) {
            FAR_ZONE_HOOD_POS = 0.5;
        } else if (gamepad2.leftBumperWasPressed()) {
            FAR_ZONE_HOOD_POS = 0.4;
        } else if (gamepad2.dpadRightWasPressed()) {
            FAR_ZONE_HOOD_POS = 0.2;;
        }

        shooter.setServoHoodManual(FAR_ZONE_HOOD_POS);

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);

        flywheelRPM = motorFlywheel.getVelocity() / CPR * 60;
        flywheelRPM2 = motorFlywheel2.getVelocity() / CPR * 60;


        // Toggle intake when right_bumper is pressed
        if (gamepad2.rightBumperWasPressed()) {
            intakeOn = !intakeOn;
        }

        if (intakeOn != prevIntake) {
            if (intakeOn) {
                spintake.turnIntakeOn();
            } else {
                spintake.turnIntakeOff();
            }
        }




        boolean rightTriggerPressed = gamepad2.right_trigger > 0.25;
        boolean leftTriggerPressed  = gamepad2.left_trigger > 0.25;

        if (rightTriggerPressed && !prevRightTrigger) {
            setPathState(10);
        }

        if (leftTriggerPressed && !prevLeftTrigger) {
            setPathState(11000); //10100
        }

        prevRightTrigger = rightTriggerPressed;
        prevLeftTrigger = leftTriggerPressed;
//
//        //start shooting sequence
//        if (gamepad2.right_trigger > 0.25) {
//            setPathState(10);
//        }
//
//        //start sequence for shooting paddle
//        if (gamepad2.left_trigger > 0.25) {
//            setPathState(10100);
//        }

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Flywheel RPM", flywheelRPM);
        telemetry.addData("Flywheel 2 RPM", flywheelRPM2);
        telemetry.addData("P,I,D,F (modified)", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified.p, pidfModified.i, pidfModified.d, pidfModified.f);
        telemetry.addData("P,I,D,F (modified) 2", "P: %.4f, I: %.4f, D: %.4f, F: %.4f",
                pidfModified2.p, pidfModified2.i, pidfModified2.d, pidfModified2.f);
        telemetry.update();

        // Panels Telemetry Data
        panelsTelemetry.addData("Target RPM", targetRPM);
        panelsTelemetry.addData("Flywheel RPM", flywheelRPM);
        panelsTelemetry.addData("Flywheel 2 RPM", flywheelRPM2);
        panelsTelemetry.update(telemetry);

    }


    public void setPathState (int pState){

        pathState = pState;
        pathTimer.resetTimer();

    }

    public void autonomousPathUpdate () {
        switch (pathState) {
            case 10:
                shooter.closeServoStop();
                shooter.downServoPaddle();
                setPathState(1001);
                break;
            case 1001:
                spintake.turnIntakeOn();
                setPathState(10001);
                break;
            case 10001:
                if (pathTimer.getElapsedTimeSeconds() > 0.01) { // changed from 0.5 to 0.25
                    shooter.openServoStop();
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    gamepad1.rumble(0.5, 0.5, 200);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    shooter.shootServoPaddle();
                    setPathState(10007);
                }
                break;
            case 10007:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    setPathState(999);
                }
                break;
            case 10100:
                shooter.openServoStop();
                if (pathTimer.getElapsedTimeSeconds() > 0.05) {
                    setPathState(10101);
                }
                break;
            case 10101:
                shooter.shootServoPaddle();
                setPathState(10102);
                break;
            case 10102:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    setPathState(999);
                }
                break;
            case 11000:
                intakeOn = true;
                setPathState(11001);
                break;
            case 11001:
                if (pathTimer.getElapsedTimeSeconds() > 0.01) { // changed from 0.5 to 0.25
                    shooter.openServoStop();
                    setPathState(11002);
                }
                break;
            case 11002:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    shooter.closeServoStop();
                    setPathState(11003);
                }
                break;
            case 11003:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    shooter.openServoStop();
                    setPathState(11004);
                }
                break;
            case 11004:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                    shooter.closeServoStop();
                    setPathState(11005);
                }
                break;
            case 11005:
                if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                    shooter.openServoStop();
                    setPathState(11006);
                }
                break;
            case 11006:
                if (pathTimer.getElapsedTimeSeconds() > 0.2){
                    gamepad1.rumble(0.5, 0.5, 200);
                }
                if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                    shooter.shootServoPaddle();
                    setPathState(11007);
                }
                break;
            case 11007:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    shooter.downServoPaddle();
                    shooter.closeServoStop();
                    setPathState(999);
                }
                break;
        }
    }

}