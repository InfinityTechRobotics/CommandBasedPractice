package org.firstinspires.ftc.teamcode.Disabled;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;

@Disabled
@TeleOp
public class OnbotJavaLogger extends OpMode {

    private static ElapsedTime timer = new ElapsedTime();
    private FileWriter logWriter;

    private VoltageSensor battery;
    double voltage;

    private DcMotor motor;
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

    int i = 0;


    @Override public void init() {
        motor = hardwareMap.get(DcMotor.class, "motorFlywheel");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        timer.reset();

        battery = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // Open the log file for appending (true flag)
        try {
//            logWriter = new FileWriter("/sdcard/FIRST/java/src/Datalogs/DataRPMModulus.txt", true);
            String logFilePath = String.format("%s/FIRST/DataLogs/DataRPMLogOBJ_OCT28_01.csv", Environment.getExternalStorageDirectory().getPath());
            logWriter = new FileWriter(logFilePath, true);
            logWriter.write("Timer, Current Ticks, Delta Rev, Delta Time, Motor Speed, Motor Power, Battery Voltage\n");  // CSV header
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to open log file: " + e.getMessage());
        }

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            motorPower = 0.;
        }
        else if (gamepad1.y && !yAlreadyPressed) {
            motorPower = motorPower + 0.025;
        }
        else if (gamepad1.a && !aAlreadyPressed) {
            motorPower = motorPower - 0.025;
        }

        motor.setPower(motorPower);

        aAlreadyPressed = gamepad1.a;
        yAlreadyPressed = gamepad1.y;

        if (i % 100 == 0) {
            currentPos = motor.getCurrentPosition();
            currentTime = timer.time();

            deltaRev = (currentPos - previousPos) / CPR;
            deltaTime = currentTime - previousTime;

            RPM = deltaRev / deltaTime * 60;

            previousPos = currentPos;
            previousTime = currentTime;

            voltage = battery.getVoltage();

            try {
                logWriter.write(currentTime + "," + currentPos + "," + deltaRev + "," + deltaTime + "," + RPM + "," + motor.getPower() + "," + voltage + "\n");
                logWriter.flush();  // Ensure data is written immediately
            } catch (IOException e) {
                telemetry.addData("Error", "Failed to write log: " + e.getMessage());
            }

        }

        telemetry.addData("Motor Power", motor.getPower());
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
