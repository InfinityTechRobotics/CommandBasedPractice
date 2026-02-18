package org.firstinspires.ftc.teamcode.SKR;


import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Disabled
@Autonomous
public class updatedSequenceSKR extends OpMode {

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Servo servoStop;

    public DcMotor motorIntake, motorTransfer;
    public DcMotorEx motorFlywheel;

    Servo servoPaddleLeft;

    double transferOn = 0.8;
    double transferOff = 0.;

    double intakeOn = 0.8;
    double intakeOff = 0.0;

    double TARGET_AUTON_RPM = 2400.;
    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double TPS;
    double targetRPM;

    double SERVO_PADDLE_SHOOT_POS = 0.3;
    double SERVO_PADDLE_DOWN_POS = 0.5;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.33;

    private VoltageSensor battery;

    public static double NEW_P = 50.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.6;    // 0.


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds() > 4.) {
                    setPathState(10001);
                }
                break;
            case 10001:
                motorIntake.setPower(intakeOn);
                motorTransfer.setPower(transferOn);
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                    setPathState(10002);
                }
                break;
            case 10002:
                if (pathTimer.getElapsedTimeSeconds() > 0.25 ) {
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10003);
                }
                break;
            case 10003:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                    setPathState(10004);
                }
                break;
            case 10004:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoStop.setPosition(SERVO_STOP_CLOSE_POS);
                    setPathState(10005);
                }
                break;
            case 10005:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoStop.setPosition(SERVO_STOP_OPEN_POS);
                    setPathState(10006);
                }
                break;
            case 10006:
                if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                    servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS);
                    setPathState(10007);
                }
                break;
            case 10007:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    motorTransfer.setPower(transferOff);
                    servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
                    setPathState(10009);
                }
                break;
        }
    }
    private void setPathState(int pState) {

        pathState = pState;
        pathTimer.resetTimer();

    }

    @Override
    public void loop() {

        // Calculate and set flywheel motor velocity
        targetRPM = TARGET_AUTON_RPM;
        TPS = targetRPM / 60. * CPR;
        motorFlywheel.setVelocity(TPS);

        autonomousPathUpdate();

        telemetry.addData("path state", pathState); // the current path the code is running
        telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
        telemetry.addData("Voltage", battery.getVoltage());
        telemetry.addData("Flywheel Motor Power", motorFlywheel.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();

    }

    @Override
    public void init() {

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");
        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        servoStop.setPosition(SERVO_STOP_CLOSE_POS);
        servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);

    }




    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }



}