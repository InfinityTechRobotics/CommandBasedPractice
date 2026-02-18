package org.firstinspires.ftc.teamcode.shadowday;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp

public class TankDriveShadowDayEdition2 extends OpMode {

    // Main drive Variables




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


        // motor directions

        // zero power behavior :) 67

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void loop() { // the loop loop

        // Assign Joysticks to power

        // Send power to motors

        // telemtry stuff


        telemetry.update();

    }

}
