package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Hardware.Drive;
import org.firstinspires.ftc.teamcode.Hardware.Pinpoint;

@Disabled
@TeleOp
public class FCTeleOpHardwareSR extends OpMode {
    Drive drive = new Drive();
    Pinpoint pinpoint = new Pinpoint();
    Pose2D pose2D;

    double DRIVE_POWER_FACTOR = 0.8;
    double DRIVE_POWER_FACTOR_LOW = 0.5;
    double DRIVE_POWER_FACTOR_HIGH = 1;

    double powerFactor = DRIVE_POWER_FACTOR;

    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);

    }
    public void loop () {

        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        pose2D = pinpoint.getPinpointPose();

        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            powerFactor = DRIVE_POWER_FACTOR_LOW;
        } else if (gamepad1.left_stick_button) {
            powerFactor = DRIVE_POWER_FACTOR_HIGH;
        } else {
            powerFactor = DRIVE_POWER_FACTOR;
        }

        if (gamepad1.a) {
            pinpoint.pinpointReset();
        }

        if (gamepad1.left_bumper) {
            drive.moveRobotRC(y, x, rx, powerFactor);
        } else {
            double botHeading = pose2D.getHeading(AngleUnit.RADIANS);
            drive.moveRobotFC(y, x, rx, botHeading, powerFactor);
        }

        telemetry.addData("X coordinate", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }
}