package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive2 {

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;

    public void init (HardwareMap hardwareMap) {

        frontLeftMotor = hardwareMap.dcMotor.get("motorFrontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("motorRearLeft");
        frontRightMotor = hardwareMap.dcMotor.get("motorFrontRight");
        backRightMotor = hardwareMap.dcMotor.get("motorRearRight");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
}

