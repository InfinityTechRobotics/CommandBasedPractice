package org.firstinspires.ftc.teamcode.Subsystems;


import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import android.transition.Scene;

@TeleOp
public class IntakeIvyTest extends LinearOpMode {
    @Override
    public void runOpMode () {
        Scheduler.reset();

        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "motorIntake");

        Command startIntake = Command.build()
                .setExecute(() -> intakeMotor.setPower(-0.75))
                .setDone(() -> gamepad1.a)
                .setEnd(endCondition -> intakeMotor.setPower(0))
                .requiring(intakeMotor);

//        Command sequence = sequential(
//                startIntake
//        );

        waitForStart();

//        gamepad1.y.onPress(startIntake);

        schedule(startIntake);

        while (opModeIsActive()) {
            Scheduler.execute();
        }
    }




}
