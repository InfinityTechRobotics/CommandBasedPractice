package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSpinfinityDuo {

    public Servo servoStop;
    public Servo servoPaddleLeft;
    public DcMotorEx motorTurret;

    double SERVO_STOP_OPEN_POS = 0.44; //0.63
    double SERVO_STOP_CLOSE_POS = 0.88; //0.37

    double SERVO_PADDLE_SHOOT_POS = 0.35; //0.85
    double SERVO_PADDLE_DOWN_POS = 0.9; //0.5

    int MOTOR_TURRET_CENTER_POS = 0;
    double MOTOR_TURRET_PROPORTIONAL_TERM = 5;

    double MOTOR_TURRET_DERIVATIVE_TERM = 0.0;

    double TURRET_ADJUSTMENT_THRESHOLD = 1.0;

    int MOTOR_TURRET_MIN_POS = -550;
    int MOTOR_TURRET_MAX_POS = 550;

    double RED_GOAL_X_POS = 130.35;

    double RED_GOAL_Y_POS = 127.65;


    public void init(HardwareMap hardwareMap) {
        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        motorTurret = hardwareMap.get(DcMotorEx.class, "motorTurret");
        motorTurret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        motorTurret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setTargetPosition(0);
        motorTurret.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motorTurret.setDirection(DcMotorEx.Direction.FORWARD);
        motorTurret.setPower(0.5);
    }

    public void openServoStop () {
        servoStop.setPosition(SERVO_STOP_OPEN_POS);
    }

    public void closeServoStop () {
        servoStop.setPosition(SERVO_STOP_CLOSE_POS);
    }

    public void shootServoPaddle () {
        servoPaddleLeft.setPosition(SERVO_PADDLE_SHOOT_POS);
    }

    public void downServoPaddle () {
        servoPaddleLeft.setPosition(SERVO_PADDLE_DOWN_POS);
    }

    public double servoStopPosition () {
        return servoStop.getPosition();
    }

    public double servoPaddlePosition () {
        return servoPaddleLeft.getPosition();
    }

    public void centerMotorTurret () {
        motorTurret.setTargetPosition(MOTOR_TURRET_CENTER_POS);
    }

    public double newTurretPositionCalc(double currentPos, double error) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
            return (currentPos + error * MOTOR_TURRET_PROPORTIONAL_TERM);
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

    public double newTurretPositionClampedCalc(double currentPos, double error) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
            int value = (int) (currentPos + error * MOTOR_TURRET_PROPORTIONAL_TERM);
            return clamp(value, MOTOR_TURRET_MIN_POS, MOTOR_TURRET_MAX_POS);
        } else {
            return currentPos;
        }
    }

    public int clamp (int value, int min, int max) {
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

    public double newTurretPoseCalc (double robotXPos, double robotYPos, double robotHeading) {

        double deltaY = RED_GOAL_Y_POS - robotYPos;

        double deltaX = RED_GOAL_X_POS - robotXPos;

        double robotToGoalAngle = Math.atan2(deltaY, deltaX);

        double robotToGoalRelativeAngle = robotToGoalAngle - robotHeading;

        return Math.toDegrees(robotToGoalRelativeAngle);
//        return Math.toDegrees(robotToGoalAngle);


//         return clamp(value, MOTOR_TURRET_MIN_POS, MOTOR_TURRET_MAX_POS);

    }

    public int turretPosEncoderCalc (double robotToGoalRelativeAngle) {
        int turretEncoderPosCalc = (int) ((535.0 / 90.0) * robotToGoalRelativeAngle);
        return clamp(turretEncoderPosCalc, MOTOR_TURRET_MIN_POS, MOTOR_TURRET_MAX_POS);
    }


}
