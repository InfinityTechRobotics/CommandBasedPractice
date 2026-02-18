package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spintake2 {

    public DcMotor motorIntake, motorTransfer;

    double INTAKE_MAX_POWER = 0.8;
    double INTAKE_NO_POWER = 0.;
    double TRANSFER_MAX_POWER = 0.8;
    double TRANSFER_NO_POWER = 0.;

    public void init (HardwareMap hardwareMap) {

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorTransfer = hardwareMap.dcMotor.get("motorTransfer");

        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        }


    public void turnIntakeOn () {
        motorIntake.setPower(INTAKE_MAX_POWER);
    }

    public void turnIntakeOff () {
        motorIntake.setPower(INTAKE_NO_POWER);
    }

    public void turnTransferOn () {
        motorTransfer.setPower(TRANSFER_MAX_POWER);
    }

    public void turnTransferOff () {
        motorTransfer.setPower(TRANSFER_NO_POWER);
    }

    public void reverseSpintakes () {
       motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);
       motorTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void forwardSpintakes () {
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorSimple.Direction.FORWARD);
    }



}

