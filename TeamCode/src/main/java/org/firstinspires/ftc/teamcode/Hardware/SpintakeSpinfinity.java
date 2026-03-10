package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.teamcode.Hardware.Spintake.TRANSFER_LOW_POWER;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//@Configurable
public class SpintakeSpinfinity {

    public DcMotorEx motorIntake;

    double INTAKE_MAX_POWER = 0.9;
    public static double INTAKE_NO_POWER = 0.2;

    public static double INTAKE_LOW_POWER = 0.2;

    int counter = 0;

    private Servo artifactIndicator3, artifactIndicator2, artifactIndicator1;

    public void init (HardwareMap hardwareMap) {

        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");

        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);

        motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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

    public void intakeLowPower () {
        motorIntake.setPower(INTAKE_LOW_POWER);
    }

    public void reverseSpintakes () {
       motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void forwardSpintakes () {
        motorIntake.setDirection(DcMotorEx.Direction.REVERSE);
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

        public double getIntakeMotorPower() {
        return motorIntake.getPower();
        }
}

