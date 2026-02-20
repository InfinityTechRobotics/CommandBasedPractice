package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Flywheel {

    public DcMotorEx motorFlywheel;

    public Servo RPMIndicatorLeft, RPMIndicatorRight;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    PIDFCoefficients pidfModified;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

    double h1 = 13.3;  // Height of camera lens from ground. 14.25
    double h2 = 29.5; // Height of AprilTag
    double a1 = 11.5; // Angle of camera relative to ground.
    // double a2 = 0.; // Limelight angle measurement between camera and AprilTag.
    double x1 = -1.0;    // Distance between camera and ramp


    public void init (HardwareMap hardwareMap) {

        motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

        motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        //pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RPMIndicatorLeft = hardwareMap.get(Servo.class, "RPMIndicatorLeft");

        RPMIndicatorRight = hardwareMap.get(Servo.class, "RPMIndicatorRight");

    }

    public double distanceToGoalCalc (double a2) {
        double angleToGoalDegrees = a1 + a2;
        return ((h2 - h1) / Math.tan(Math.toRadians(angleToGoalDegrees)) + x1);
    }

    public double targetRPMCalc (double distanceToGoalInches) {
        return (11.7 * distanceToGoalInches + 1743);
    }

    // Calculate and set flywheel motor velocity
    public void setFlywheelVel (double targetRPM) {
        motorFlywheel.setVelocity(targetRPM / 60. * CPR);
    }

    // Calculate current flywheel motor RPM
    public double getFlywheelVel () {
        return (motorFlywheel.getVelocity() / CPR * 60);
    }

    public void setFlywheelRGB (double flywheelRPM, double targetRPM) {

        if (flywheelRPM < (targetRPM - 150)) { // turns the RGB lights blue if the flywheel speed is too low
            RPMIndicatorLeft.setPosition(0.611);
            RPMIndicatorRight.setPosition(0.611);
        } else if (flywheelRPM > (targetRPM + 150)) { // turns the RGB lights orange if the flywheel speed is too high
            RPMIndicatorLeft.setPosition(0.3);
            RPMIndicatorRight.setPosition(0.3);
        } else { // turns the RGB indicator green if the flywheel speed is correct
            RPMIndicatorLeft.setPosition(0.5);
            RPMIndicatorRight.setPosition(0.5);
        }

    }

    public double getMotorFlywheelCurrent () {
        return motorFlywheel.getCurrent(CurrentUnit.AMPS);
    }
    public double getMotorFlywheelPower () {
        return motorFlywheel.getPower();
    }

}
