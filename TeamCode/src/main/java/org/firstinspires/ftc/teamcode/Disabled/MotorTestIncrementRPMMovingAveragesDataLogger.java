package org.firstinspires.ftc.teamcode.Disabled;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

@Disabled

@TeleOp
public class MotorTestIncrementRPMMovingAveragesDataLogger extends OpMode {

    private static ElapsedTime timer = new ElapsedTime();
    private FileWriter logWriter;

    private DcMotor motor;
    private DcMotor motor2;
    private Servo servoWebcam;
    double  motorPower = 0.;
    double motor2Power = 0.;
    boolean aAlreadyPressed;
    boolean yAlreadyPressed;
    boolean dpadDAlreadyPressed;
    boolean dpadUAlreadyPressed;

    double currentPos = 0;
    double previousPos = 0;
    double CPR = 145.1;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;
    double deltaRev = 0;
    double previousTime = 0;
    double currentTime = 0;
    double deltaTime = 0;
    double RPM = 0;

    int i = 0;

    List<Double> averageResult = new ArrayList<>();

    int currentIndex = 0;
    List<Double> rpmList = new ArrayList<>();

    @Override public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        timer.reset();

        motor2 = hardwareMap.get(DcMotor.class, "motor1");
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoWebcam = hardwareMap.get(Servo.class,"servoWebcam");

        // Open the log file for appending (true flag)
        try {
//            logWriter = new FileWriter("/sdcard/FIRST/java/src/Datalogs/DataRPMModulus.txt", true);
            String logFilePath = String.format("%s/FIRST/DataLogs/DataRPMMovingAvg.txt", Environment.getExternalStorageDirectory().getPath());
            logWriter = new FileWriter(logFilePath, true);
            logWriter.write("Timer, Current Ticks, Delta Rev, Delta Time, Motor Speed, Motor Power\n");  // CSV header
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
            motorPower = motorPower + 0.05;
        }
        else if (gamepad1.a && !aAlreadyPressed) {
            motorPower = motorPower - 0.05;
        }
        else if (gamepad1.dpad_down && !dpadDAlreadyPressed) {
            motor2Power = motor2Power - 0.05;
        }
        else if (gamepad1.dpad_up && !dpadUAlreadyPressed) {
            motor2Power = motor2Power + 0.05;
        }
        else if (gamepad1.dpad_left) {
            motor2Power = 0.;
        }

        motor2.setPower(motor2Power);
        motor.setPower(motorPower);

        aAlreadyPressed = gamepad1.a;
        yAlreadyPressed = gamepad1.y;

        dpadUAlreadyPressed = gamepad1.dpad_up;
        dpadDAlreadyPressed = gamepad1.dpad_down;

        currentPos = motor.getCurrentPosition();
        currentTime = timer.time();

        deltaRev = (currentPos - previousPos) / CPR;
        deltaTime = currentTime - previousTime;

        RPM = deltaRev / deltaTime * 60;

        previousPos = currentPos;
        previousTime = currentTime;

        rpmList.add(RPM);

        if (i > 10) {
            // Calculate the average of the 10 values and store in AverageList
            List<Double> currentList = rpmList.subList(currentIndex, currentIndex + 10);
            double sum = 0;
            for (Double num: currentList) {
                sum += num;
            }
            double average = sum / 10;
            averageResult.add(average);
            telemetry.addData("RPM Average ("+ currentIndex + ")", average);
            currentIndex += 1;
        }

        if (RPM > 1000){
            servoWebcam.setPosition(0.555);
        }
        else {
            servoWebcam.setPosition(0.333);
        }

        telemetry.addData("Motor 1 Power", motor.getPower());
        telemetry.addData("Motor 2 Power", motor2.getPower());
        telemetry.addData("RPM", RPM);
        telemetry.addData("Timer", timer.time());
        telemetry.addData("i",i);
        telemetry.update();

        i += 1;

        try {
            logWriter.write(currentTime + "," + currentPos + "," + deltaRev + "," + deltaTime + "," + RPM + "," + motor.getPower() + "\n");
            logWriter.flush();  // Ensure data is written immediately
        } catch (IOException e) {
            telemetry.addData("Error", "Failed to write log: " + e.getMessage());
        }

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
