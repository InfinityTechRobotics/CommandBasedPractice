package org.firstinspires.ftc.teamcode.Disabled;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;

@Disabled
@TeleOp
public class OnbotJavaLoggerPID extends OpMode {

    private static ElapsedTime timer = new ElapsedTime();
    private FileWriter logWriter;

    public static double NEW_P = 100.;   // 10.
    public static double NEW_I = 1.;    // 3.
    public static double NEW_D = 20.;    // 0.
    public static double NEW_F = 3.5;    // 0.

    private VoltageSensor battery;
    double voltage;

    private DcMotorEx motor;
    double  motorPower = 0.;
    boolean aAlreadyPressed;
    boolean yAlreadyPressed;

    double currentPos = 0;
    double previousPos = 0;
    double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double deltaRev = 0;
    double previousTime = 0;
    double currentTime = 0;
    double deltaTime = 0;
    double RPM = 0;
    double targetRPM = 0.;
    double TPS;

    int i = 0;


    @Override public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motorFlywheel");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        timer.reset();

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        // Open the log file for appending (true flag)
        try {
//            logWriter = new FileWriter("/sdcard/FIRST/java/src/Datalogs/DataRPMModulus.txt", true);
            String logFilePath = String.format("%s/FIRST/DataLogs/DataRPMLogOBJ_Nov19_01.csv", Environment.getExternalStorageDirectory().getPath());
            logWriter = new FileWriter(logFilePath, true);
            logWriter.write("Timer, Current Ticks, Delta Rev, Delta Time, Motor Speed, TargetRPM, Motor Power, Battery Voltage\n");  // CSV header
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to open log file: " + e.getMessage());
        }

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            targetRPM = 0.;
        } else if (gamepad1.b) {
            targetRPM = 2500.;
        } else if (gamepad1.a && !aAlreadyPressed) {
            targetRPM -= 25.;
        } else if (gamepad1.y && !yAlreadyPressed) {
            targetRPM += 25.;
        }

        aAlreadyPressed = gamepad1.a;
        yAlreadyPressed = gamepad1.y;

        // Calculate and set flywheel motor velocity
        TPS = targetRPM / 60. * CPR;
        motor.setVelocity(TPS);

        if (i % 10 == 0) {
            currentPos = motor.getCurrentPosition();
            currentTime = timer.time();

            deltaRev = (currentPos - previousPos) / CPR;
            deltaTime = currentTime - previousTime;

            RPM = deltaRev / deltaTime * 60;

            previousPos = currentPos;
            previousTime = currentTime;

            voltage = battery.getVoltage();

            try {
                logWriter.write(currentTime + "," + currentPos + "," + deltaRev + "," + deltaTime + "," + RPM + "," + targetRPM + "," + motor.getPower() + "," + voltage + "\n");
                logWriter.flush();  // Ensure data is written immediately
            } catch (IOException e) {
                telemetry.addData("Error", "Failed to write log: " + e.getMessage());
            }

        }

        telemetry.addData("Motor Power", motor.getPower());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("RPM", RPM);
        telemetry.addData("Delta Rev", deltaRev);
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("Timer", timer.time());
        telemetry.addData("Voltage", voltage);
        telemetry.addData("i",i);
        telemetry.update();

        i += 1;

    }

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
