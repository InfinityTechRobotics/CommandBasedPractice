package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class FlywheelSpinfinityDuo {

    public DcMotorEx motorFlywheel1;
    public DcMotorEx motorFlywheel2;

    public Servo RPMIndicatorLeft, RPMIndicatorRight;

    public static double NEW_P = 150.;   // 150.
    public static double NEW_I = 5.;    // 2.5
    public static double NEW_D = 40.;    // 50.
    public static double NEW_F = 1.25;    // 2.5

    PIDFCoefficients pidfModified;

    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

//    double h1 = 13.25;  // Height of camera lens from ground. 14.25
//    double h2 = 29.5; // Height of AprilTag
//    double a1 = 12; // Angle of camera relative to ground.
//    // double a2 = 0.; // Limelight angle measurement between camera and AprilTag.
//    double x1 = -3.5;    // Distance between camera and ramp

    double h1 = 15.375;  // Height of camera lens from ground. 14.25
    double h2 = 29.5; // Height of AprilTag
    double a1 = 21.8; // Angle of camera relative to ground.
    // double a2 = 0.; // Limelight angle measurement between camera and AprilTag.
    double x1 = -1.5;    // Distance between camera and ramp


    public void init (HardwareMap hardwareMap) {

        motorFlywheel1 = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motorFlywheel2 = hardwareMap.get(DcMotorEx.class, "motorFlywheel2");

        motorFlywheel1.setDirection(DcMotorEx.Direction.REVERSE);
        motorFlywheel2.setDirection(DcMotorEx.Direction.FORWARD);

        motorFlywheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorFlywheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        motorFlywheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        motorFlywheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Set Flywheel Motor PIDF coefficients
        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motorFlywheel1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);
        motorFlywheel2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        //pidfModified = motorFlywheel.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER);

        RPMIndicatorLeft = hardwareMap.get(Servo.class, "RPMIndicatorLeft");

        RPMIndicatorRight = hardwareMap.get(Servo.class, "RPMIndicatorRight");

    }

    public double distanceToGoalCalc (double a2) {
        double angleToGoalDegrees = a1 + a2;
        return ((h2 - h1) / Math.tan(Math.toRadians(angleToGoalDegrees)) + x1);
    }

    public double targetRPMCalc (double distanceToGoalInches) {
        return (11.4 * distanceToGoalInches + 1832 + 25); // Old Equation: 10 * distanceToGoalInches + 1914 + 25
    }

    // Calculate and set flywheel motor velocity
    public void setFlywheelVel (double targetRPM) {
        motorFlywheel1.setVelocity(targetRPM / 60. * CPR);
        motorFlywheel2.setVelocity(targetRPM / 60. * CPR);
    }

    // Calculate current flywheel motor RPM
    public double getFlywheelVel () {
        return (motorFlywheel1.getVelocity() / CPR * 60);
    }
    public double getFlywheelVel2 () {
        return (motorFlywheel2.getVelocity() / CPR * 60);
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
        return motorFlywheel1.getCurrent(CurrentUnit.AMPS);
    }
    public double getMotorFlywheelPower () {
        return motorFlywheel1.getPower();
    }

}
