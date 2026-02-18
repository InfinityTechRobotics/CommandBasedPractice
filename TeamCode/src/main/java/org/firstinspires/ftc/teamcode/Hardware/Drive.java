package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive {

    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;
    public DcMotorEx backLeftMotor;

    public void init (HardwareMap hardwareMap) {

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
    }

    public void moveRobot (double y, double x, double rx) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public void moveRobotRC (double y, double x, double rx, double powerFactor) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftMotor.setPower(powerFactor * frontLeftPower);
        backLeftMotor.setPower(powerFactor * backLeftPower);
        frontRightMotor.setPower(powerFactor * frontRightPower);
        backRightMotor.setPower(powerFactor * backRightPower);
    }

    public void moveRobotFC (double y, double x, double rx, double botHeading, double powerFactor) {

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(powerFactor * frontLeftPower);
        backLeftMotor.setPower(powerFactor * backLeftPower);
        frontRightMotor.setPower(powerFactor * frontRightPower);
        backRightMotor.setPower(powerFactor * backRightPower);
    }

    public double squareInputWithSign (double input) {
        double square = input * input;
        if (input < 0) {
            square = square * -1;
        }
        return square;
    }
}

