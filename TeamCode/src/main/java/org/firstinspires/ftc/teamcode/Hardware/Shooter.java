package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    public Servo servoStop;
    public Servo servoPaddleLeft;
    public Servo servoTurret;

    double SERVO_STOP_OPEN_POS = 0.15;
    double SERVO_STOP_CLOSE_POS = 0.38;

    double SERVO_PADDLE_SHOOT_POS = 0.7;
    double SERVO_PADDLE_DOWN_POS = 0.25;

    double SERVO_TURRET_CENTER_POS = 0.56;
    double SERVO_TURRET_PROPORTIONAL_TERM = 0.0008;


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

    public double newTurretPosition (double currentPos, double error) {
        return currentPos + error * SERVO_TURRET_PROPORTIONAL_TERM;
    }
    public void servoTurretSetPosition(double newTurretPos) {
        servoTurret.setPosition(newTurretPos);
    }

    public double servoTurretGetPosition() {
        return servoTurret.getPosition();
    }

}
