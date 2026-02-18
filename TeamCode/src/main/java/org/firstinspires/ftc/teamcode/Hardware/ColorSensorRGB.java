package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@TeleOp
@Disabled
public class ColorSensorRGB extends LinearOpMode {
    Servo light;

    private NormalizedColorSensor test_color;
    double hue;

    @Override
    public void runOpMode () {
        test_color = hardwareMap.get (NormalizedColorSensor.class, "colorV3");
        test_color.setGain(5);
        light = hardwareMap.get(Servo.class,"light");
        waitForStart();

        while (opModeIsActive()) {


            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
            NormalizedRGBA colors = test_color.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());

            //Determining the amount of red, green, and blue
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f",colors.blue);

            //Determining HSV and alpha
            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            telemetry.update();

            if(hue > 150 && hue < 170) {
                telemetry.addData("Ball Color","Green");
                light.setPosition(0.500);
            }
            else if(hue > 210 && hue < 230) {
                telemetry.addData("Ball Color","Purple");
                light.setPosition(0.722);
            }

            else {
                telemetry.addData("Ball Color", "Unknown");
                light.setPosition(0.0);
            }





            //Using hue to detect color
            //  if(hue < 30){
            //          telemetry.addData("Color", "Red");
            //          }
            //   else if (hue < 60) {
            //          telemetry.addData("Color", "Orange");
            //          }
            //   else if (hue < 90){
            //          telemetry.addData("Color", "Yellow");
            //         }
            //   else if (hue < 150){
            //          telemetry.addData("Color", "Green");
            //         }
            //   else if (hue < 225){
            //          telemetry.addData("Color", "Blue");
            //         }
            //   else if (hue < 350){
            //          telemetry.addData("Color", "Purple");
            //         }
            //   else{
            //          telemetry.addData("Color", "Red");
            //         }



        }



    }
}