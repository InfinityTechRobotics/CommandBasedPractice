package org.firstinspires.ftc.teamcode.shadowday;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Disabled
@TeleOp

public class TankDriveAnswerKey extends OpMode {

    // Stuff to add for student varibles edition

    double powerFactor = 0.5;
    double leftPower;
    double rightPower;


    // --------------------------

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;

    public void init () {

        // hardware stuff

        frontLeftMotor = hardwareMap.dcMotor.get("motorFrontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("motorRearLeft");
        frontRightMotor = hardwareMap.dcMotor.get("motorFrontRight");
        backRightMotor = hardwareMap.dcMotor.get("motorRearRight");

        //motor directions

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // zero power behavior

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }

    public void loop() {

// Assign Joysticks to power
        leftPower  = -gamepad1.left_stick_y;
        rightPower = -gamepad1.right_stick_y;

// Send power to motors
        frontLeftMotor.setPower(leftPower * powerFactor);
        backLeftMotor.setPower(leftPower * powerFactor);

        frontRightMotor.setPower(rightPower  * powerFactor);
        backRightMotor.setPower(rightPower * powerFactor);

        // telemtry stuff

        telemetry.addData("Left Motor Power", leftPower);
        telemetry.addData("Right Motor Power", rightPower);

        telemetry.update();



    }

}
