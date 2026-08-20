package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DriveSubsystem {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    public DriveSubsystem(HardwareMap hardwareMap) {

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

    public void stop(){
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public double getMotorFLCurrent () {
        return frontLeftMotor.getCurrent(CurrentUnit.AMPS);
    }
    public double getMotorBLCurrent () {
        return backLeftMotor.getCurrent(CurrentUnit.AMPS);
    }
    public double getMotorFRCurrent () {
        return frontRightMotor.getCurrent(CurrentUnit.AMPS);
    }
    public double getMotorBRCurrent () {
        return backRightMotor.getCurrent(CurrentUnit.AMPS);
    }

    public double getMotorFLPower () {
        return frontLeftMotor.getPower();
    }
    public double getMotorBLPower () {
        return backLeftMotor.getPower();
    }
    public double getMotorFRPower () {
        return frontRightMotor.getPower();
    }
    public double getMotorBRPower () {
        return backRightMotor.getPower();
    }
}

