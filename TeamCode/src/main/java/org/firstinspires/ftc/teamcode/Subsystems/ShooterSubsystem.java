package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class ShooterSubsystem {

    private Servo servoStop;
    private Servo servoPaddleLeft;
    private Servo servoHood;
    private DcMotorEx motorTurret;


    // Turret Motor PID Values
    public static double NEW_P = 15.;
    public static double NEW_I = 0.;
    public static double NEW_D = 1.;
    public static double NEW_F = 1.25;

    double SERVO_HOOD_UP_POS = 0.5;
    double SERVO_HOOD_DOWN_POS = 0.2;
    double SERVO_HOOD_MID_POS = 0.3;

    double SERVO_STOP_OPEN_POS = 0.32; //0.44
    double SERVO_STOP_CLOSE_POS = 0.78; //0.88

    double SERVO_PADDLE_SHOOT_POS = 0.35; //0.85
    double SERVO_PADDLE_DOWN_POS = 0.9; //0.5

    int MOTOR_TURRET_CENTER_POS = 0;

    double TURRET_ADJUSTMENT_THRESHOLD = 1.0;

    int MOTOR_TURRET_MIN_POS = -525;    // -550
    int MOTOR_TURRET_MAX_POS = 525; // 550

    double RED_GOAL_X_POS = 130.35;
    double RED_GOAL_Y_POS = 127.65;
    double BLUE_GOAL_X_POS = 13.65;
    double BLUE_GOAL_Y_POS = 127.65;


    public void init(HardwareMap hardwareMap) {
        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        servoHood = hardwareMap.get(Servo.class, "servoHood");

        servoHood.setPosition(0.2);

        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setTargetPosition(0);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorTurret.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorTurret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        motorTurret.setPower(0.75);
    }

    public void openServoStop() {
        servoStop.setPosition(SERVO_STOP_OPEN_POS);
    }

    public void closeServoStop() {
        servoStop.setPosition(SERVO_STOP_CLOSE_POS);
    }

    public void shootServoPaddle() {
        servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS);
    }

    public void downServoPaddle() {
        servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
    }

    public double servoStopPosition() {
        return servoStop.getPosition();
    }

    public double servoPaddlePosition() {
        return servoPaddleLeft.getPosition();
    }

    public void centerMotorTurret() {
        motorTurret.setTargetPosition(MOTOR_TURRET_CENTER_POS);
    }

    public double newTurretPositionCalc(double currentPos, double error) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
//            return (currentPos + error * MOTOR_TURRET_PROPORTIONAL_TERM);
            return (currentPos + error * (550.0 / 90.0));
        } else {
            return currentPos;
        }
    }

    public void motorTurretSetPosition(int newPos) {
        motorTurret.setTargetPosition(newPos);
    }

    public int motorTurretGetPosition() {
        return motorTurret.getCurrentPosition();
    }

    public int newTurretPositionClampedCalc(int currentPos, double error) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
//            int value = (int) (currentPos + error * MOTOR_TURRET_PROPORTIONAL_TERM);
            int value = (int) (currentPos + error * (550.0 / 90.0));
            return clamp(value, MOTOR_TURRET_MIN_POS, MOTOR_TURRET_MAX_POS);
        } else {
            return currentPos;
        }
    }

    public int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public int newTurretPDCalc(double currentPos, double error, double prevError, double turretTimer, double kP, double kD) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
//            double proportional = error * SERVO_TURRET_PROPORTIONAL_TERM;
//            double derivative = ((error - prevError) / turretTimer) * SERVO_TURRET_DERIVATIVE_TERM;
            double proportional = error * kP;
            double derivative = ((error - prevError) / turretTimer) * kD;
            int value = (int) (currentPos + proportional + derivative);
            return clamp(value, MOTOR_TURRET_MIN_POS, MOTOR_TURRET_MAX_POS);
        } else {
            return (int) currentPos;
        }
    }

    public double newTurretPoseCalc(double robotXPos, double robotYPos, double robotHeading) {

        double deltaY = RED_GOAL_Y_POS - robotYPos;

        double deltaX = RED_GOAL_X_POS - robotXPos;

        double robotToGoalAngle = Math.atan2(deltaY, deltaX);

        double robotToGoalRelativeAngle = robotToGoalAngle - robotHeading;

        return Math.toDegrees(robotToGoalRelativeAngle);

    }

    public int turretPosEncoderCalc(double robotToGoalRelativeAngle) {
        int turretEncoderPosCalc = (int) ((550.0 / 90.0) * robotToGoalRelativeAngle);
        return clamp(turretEncoderPosCalc, MOTOR_TURRET_MIN_POS, MOTOR_TURRET_MAX_POS);
    }

    public double newTurretBluePoseCalc(double robotXPos, double robotYPos, double robotHeading) {

        double deltaY = BLUE_GOAL_Y_POS - robotYPos;

        double deltaX = BLUE_GOAL_X_POS - robotXPos;

        double robotToGoalAngle = Math.atan2(deltaY, deltaX);

        double robotToGoalRelativeAngle = robotToGoalAngle - robotHeading;

        return Math.toDegrees(robotToGoalRelativeAngle);

    }

    public double getMotorTurretCurrent() {
        return motorTurret.getCurrent(CurrentUnit.AMPS);
    }

    public double getMotorTurretPower() {
        return motorTurret.getPower();
    }

    public void setMotorTurretPIDF(double P, double I, double D, double F) {
        PIDFCoefficients pidfNew = new PIDFCoefficients(P, I, D, F);
        motorTurret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
    }

    public void setServoHoodUpPos() {
        servoHood.setPosition(SERVO_HOOD_UP_POS);
    }

    public void setServoHoodDownPos() {
        servoHood.setPosition(SERVO_HOOD_DOWN_POS);
    }

    public void setServoHoodMidPos() {
        servoHood.setPosition(SERVO_HOOD_MID_POS);
    }

    public void setServoHoodManual(double manual_hood_pos) {
        servoHood.setPosition(manual_hood_pos);
    }

    public void resetTurretPos() {
        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorTurret.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
        motorTurret.setPower(0.75);
    }

    public void stop(){
        motorTurret.setPower(0);
    }

}
