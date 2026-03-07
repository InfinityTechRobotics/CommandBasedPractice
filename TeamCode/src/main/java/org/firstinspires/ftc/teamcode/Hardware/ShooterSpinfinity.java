package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSpinfinity {

    public Servo servoStop;
    public Servo servoPaddleLeft;
    public Servo servoTurret;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.38;

    double SERVO_PADDLE_SHOOT_POS = 0.7;
    double SERVO_PADDLE_DOWN_POS = 0.25;

    double SERVO_TURRET_CENTER_POS = 0.56;
    double SERVO_TURRET_PROPORTIONAL_TERM = 0.0016;

    double SERVO_TURRET_DERIVATIVE_TERM = 0.0;

    double TURRET_ADJUSTMENT_THRESHOLD = 1.0;

    double SERVO_MIN_POS = 0;
    double SERVO_MAX_POS = 1;




    public void init(HardwareMap hardwareMap) {
        servoStop = hardwareMap.get(Servo.class, "servoStop");
        servoPaddleLeft = hardwareMap.servo.get("servoPaddleLeft");

        servoTurret = hardwareMap.get(Servo.class, "servoWebcam");
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

    public void centerServoTurret () {
        servoTurret.setPosition(SERVO_TURRET_CENTER_POS);
    }

    public double newTurretPositionCalc(double currentPos, double error) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
            return (currentPos + error * SERVO_TURRET_PROPORTIONAL_TERM);
        } else {
            return currentPos;
        }
    }

    public void servoTurretSetPosition(double newTurretPos) {
        servoTurret.setPosition(newTurretPos);
    }

    public double servoTurretGetPosition() {
        return servoTurret.getPosition();
    }

    public double newTurretPositionClampedCalc(double currentPos, double error) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
            double value = currentPos + error * SERVO_TURRET_PROPORTIONAL_TERM;
            return clamp(value, SERVO_MIN_POS, SERVO_MAX_POS);
        } else {
            return currentPos;
        }
    }

    public double clamp (double value, double min, double max) {
       return Math.max(min, Math.min(max, value));
    }

    public double newTurretPDCalc(double currentPos, double error, double prevError, double turretTimer, double kP, double kD) {
        if (Math.abs(error) > TURRET_ADJUSTMENT_THRESHOLD) {
//            double proportional = error * SERVO_TURRET_PROPORTIONAL_TERM;
//            double derivative = ((error - prevError) / turretTimer) * SERVO_TURRET_DERIVATIVE_TERM;
            double proportional = error * kP;
            double derivative = ((error - prevError) / turretTimer) * kD;
            double value = currentPos + proportional + derivative;
            return clamp(value, SERVO_MIN_POS, SERVO_MAX_POS);
        } else {
            return currentPos;
        }
    }

}
