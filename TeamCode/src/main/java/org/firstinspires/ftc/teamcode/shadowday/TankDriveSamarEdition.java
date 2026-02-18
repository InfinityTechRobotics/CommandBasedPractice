package org.firstinspires.ftc.teamcode.shadowday;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@TeleOp

public class TankDriveSamarEdition extends OpMode {

    // Main drive Variables


    // --------------------------

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public DcMotor backLeftMotor;

    public void init () {

        // hardware stuff

        // motor directions

        // zero power behavior :) 67

    }

    public void loop() { // the loop loop

        // Assign Joysticks to power

        // Send power to motors

        // telemtry stuff


        telemetry.update();

    }

}
