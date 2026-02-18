package org.firstinspires.ftc.teamcode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.io.FileWriter;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.IOException;


@Disabled
@TeleOp
public class MotorRPMLoggerSR extends OpMode {
    private DcMotor motor;

    double currentPos = 0;
    double previousPos = 0;
    double CPR = 28.;
    double revolutions = 0;
    double previousRev = 0;
    double deltaRev = 0;
    double previousTime = 0;
    double currentTime = 0;
    double deltaTime = 0;
    double RPM = 0;

    private FileWriter logWriter;

    ElapsedTime timer = new ElapsedTime();


    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        timer.reset();

        // Open the log file for appending (true flag)
        try {
            logWriter = new FileWriter("/sdcard/FIRST/java/src/Datalogs/testData.txt", true);
            logWriter.write("Timer, Current Ticks, Delta Rev, Delta Time, Motor Speed, Motor Power\n");  // CSV header
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to open log file: " + e.getMessage());
        }
    }

    @Override public void loop() {
        if (gamepad1.a) {
            motor.setPower(0.6);
        } else if(gamepad1.b) {
            motor.setPower(0);
        }

        currentPos = motor.getCurrentPosition();

        currentTime = timer.time();

        revolutions = currentPos / CPR;

        deltaRev = (currentPos - previousPos) / CPR;

        deltaTime = currentTime - previousTime;

        RPM = deltaRev / deltaTime * 60;

        previousPos = currentPos;

        previousTime = currentTime;


        telemetry.addData("Motor Position", motor.getCurrentPosition());
        telemetry.addData("Timer", currentTime);
        telemetry.addData("Revolutions", revolutions);
        telemetry.addData("RPM", RPM);
        telemetry.update();

        try {


            logWriter.write(currentTime + "," + currentPos + "," + deltaRev + "," + deltaTime + "," + RPM + "," + motor.getPower() + "\n");

            logWriter.flush();  // Ensure data is written immediately

            telemetry.addData("Logging", "Speed: " + RPM);  // Optional: Show on DS

        } catch (IOException e) {

            telemetry.addData("Error", "Failed to write log: " + e.getMessage());

        }

    }

    @Override

    public void stop() {

        // Close the file to prevent data loss

        try {

            if (logWriter != null) {

                logWriter.close();

            }

        } catch (IOException e) {

            telemetry.addData("Error", "Failed to close log file: " + e.getMessage());

        }
    }
}
