package org.firstinspires.ftc.teamcode.DisabledHardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Disabled

//@Configurable
public class Spintake {

    public DcMotorEx motorIntake, motorTransfer;

    public DigitalChannel ledTransferGreen;
    public DigitalChannel ledTransferRed;

    double INTAKE_MAX_POWER = 0.8;
    public static double INTAKE_NO_POWER = 0.2;
    double TRANSFER_MAX_POWER = 0.8; // 0.8
    public static double TRANSFER_NO_POWER = 0.;

    public static double INTAKE_LOW_POWER = 0.2;

    public static double TRANSFER_LOW_POWER = 0.2;

//    public static double INTAKE_CURRENT_ALERT = 6.;
//
//    public static double TRANSFER_CURRENT_ALERT = 6.;

    double intakeCurrent, transferCurrent;

    int counter = 0;

    private Servo artifactIndicator3, artifactIndicator2, artifactIndicator1;

    public void init (HardwareMap hardwareMap) {

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
        motorTransfer = hardwareMap.get(DcMotorEx.class, "motorTransfer");

        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorTransfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        ledTransferGreen = hardwareMap.get(DigitalChannel.class, "ledTransferGreen");
        ledTransferRed = hardwareMap.get(DigitalChannel.class, "ledTransferRed");

        ledTransferGreen.setMode(DigitalChannel.Mode.OUTPUT);
        ledTransferRed.setMode(DigitalChannel.Mode.OUTPUT);

        artifactIndicator3 = hardwareMap.get(Servo.class, "artifactIndicator3");
        artifactIndicator2 = hardwareMap.get(Servo.class, "artifactIndicator2");
        artifactIndicator1 = hardwareMap.get(Servo.class, "artifactIndicator1");

        //motorIntake.setCurrentAlert(INTAKE_CURRENT_ALERT, CurrentUnit.AMPS);

        //motorTransfer.setCurrentAlert(TRANSFER_CURRENT_ALERT, CurrentUnit.AMPS);
        
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

    public void intakeLowPower () {
        motorIntake.setPower(INTAKE_LOW_POWER);
    }
    public void transferLowPower () {
        motorTransfer.setPower(TRANSFER_LOW_POWER);
    }

    public void reverseSpintakes () {
       motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
       motorTransfer.setDirection(DcMotorEx.Direction.REVERSE);
    }
    public void forwardSpintakes () {
        motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
        motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void setTransferLEDOn () {
        ledTransferRed.setState(false);
        ledTransferGreen.setState(true);
    }

    public void setTransferLEDOff () {
        ledTransferRed.setState(true);
        ledTransferGreen.setState(false);
    }

//    public void getIntakeCurrent () {
//        intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);
//    }
//
//    public void getTransferCurrent () {
//        transferCurrent = motorTransfer.getCurrent(CurrentUnit.AMPS);
//    }

    public void checkIntakeCurrent () {
        if (motorIntake.isOverCurrent()) {
            turnIntakeOff();
        }
    }

    public void checkTransferCurrent () {
        if (motorTransfer.isOverCurrent()) {
            turnTransferOff();
        }
    }

    public void setArtifactIndicator (double counter) {
        if (counter >= 3) {
            artifactIndicator3.setPosition(0.555);
        } else {
            artifactIndicator3.setPosition(0.);
        }

        if (counter >= 2) {
            artifactIndicator2.setPosition(0.555);
        } else {
            artifactIndicator2.setPosition(0.);
        }

        if (counter >= 1) {
            artifactIndicator1.setPosition(0.555);
        }else {
            artifactIndicator1.setPosition(0.);
        }
    }

        public double getIntakeMotorCurrent() {
            return motorIntake.getCurrent(CurrentUnit.AMPS);
        }
        public double getTransferMotorCurrent() {
            return motorTransfer.getCurrent(CurrentUnit.AMPS);
        }
        public double getIntakeMotorPower() {
        return motorIntake.getPower();
        }
        public double getTransferMotorPower() {
        return motorTransfer.getPower();
        }


}

