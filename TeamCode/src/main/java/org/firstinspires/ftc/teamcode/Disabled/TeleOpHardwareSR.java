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
public class TeleOpHardwareSR extends OpMode {
    Drive drive = new Drive();
    Pinpoint pinpoint = new Pinpoint();
    Pose2D pose2D;

    public void init() {

        drive.init(hardwareMap);
        pinpoint.init(hardwareMap);

    }
    public void loop () {

        double y = -gamepad1.left_stick_y;
        double x =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        drive.moveRobot(y, x, rx);

        pose2D = pinpoint.getPinpointPose();

        telemetry.addData("X coordinate", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle", pose2D.getHeading(AngleUnit.DEGREES));
        telemetry.update();

    }
}
