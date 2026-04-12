    package org.firstinspires.ftc.teamcode.DisabledAuton;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.Hardware.Shooter;

    @Disabled
    @Autonomous
        public class wRedGoalShootingSequenceRev1 extends OpMode {

        Shooter shooter = new Shooter();

        double START_DELAY_TIME = 2.;

        double shootingTime = 0.;

        private Timer pathTimer, opmodeTimer;
        private Timer shootTimer;
        private int pathState;

        public static double NEW_P = 100.;   // 10.
        public static double NEW_I = 1.;    // 3.
        public static double NEW_D = 20.;    // 0.
        public static double NEW_F = 3.5;    // 0.

        DcMotorEx motorIntake, motorTransfer;
        DcMotorEx motorFlywheel;

        double targetRPM = 0.;
        double flywheelRPM = 0.;
        double TPS;
        double CPR = 28.;   // 6000 RPM = 28.; 1620 RPM = 103.8; 1150 RPM = 145.1;

        double transferOn = 0.8;
        double transferOff = 0.;

        double intakeOn = 0.8;
        double intakeOff = 0.0;

        double TARGET_AUTON_RPM = 2250.;


        public void autonomousPathUpdate() {
            switch (pathState) {
                case 10:
                    shooter.closeServoStop();
                    shooter.downServoPaddle();
                    setPathState(11);
                    break;
                case 11:
                    if (pathTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                        setPathState(1000);
                    }
                    break;
                case 1000:
                    setPathState(1001);
                    break;
                case 1001:
                    motorIntake.setPower(intakeOn);
                    motorTransfer.setPower(transferOn);
                    setPathState(10001);
                    break;
                case 10001:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) { // changed from 0.5 to 0.25
                        shootTimer.resetTimer();
                        shooter.openServoStop();
                        setPathState(10006);
                    }
                    break;
                case 10002:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        shooter.closeServoStop();
                        setPathState(10003);
                    }
                    break;
                case 10003:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        shooter.openServoStop();
                        setPathState(10004);
                    }
                    break;
                case 10004:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        shooter.closeServoStop();
                        setPathState(10005);
                    }
                    break;
                case 10005:
                    if (pathTimer.getElapsedTimeSeconds() > 0.25) {
                        shooter.openServoStop();
                        setPathState(10006);
                    }
                    break;
                case 10006:
                    if (pathTimer.getElapsedTimeSeconds() > 0.6) {
                        shooter.shootServoPaddle();
                        setPathState(10007);
                    }
                    break;
                case 10007:
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        //motorTransfer.setPower(transferOff);
                        shootingTime = shootTimer.getElapsedTimeSeconds();
                        shooter.downServoPaddle();
                        shooter.closeServoStop();
                        motorIntake.setPower(intakeOff);
                        setPathState(10008);
                    }
                    break;
                case 10008:
                    setPathState(10009);
                    break;
                case 10009: // updates shooting sequence flag
                    if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        setPathState(10010);
                    }
                    break;
                case 10010:  // sends to next driving path
                    setPathState(999);
                    break;
                case 999: // last state, just stops and waits
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        TARGET_AUTON_RPM = 0.;
                        motorTransfer.setPower(transferOff);
                        motorIntake.setPower(intakeOff);
                        setPathState(912);
                    }
                    break;
            }
        }

        private void setPathState(int pState) {

            pathState = pState;
            pathTimer.resetTimer();

        }

        @Override
        public void loop() {

            autonomousPathUpdate();

            targetRPM = TARGET_AUTON_RPM;
            TPS = targetRPM / 60. * CPR;
            motorFlywheel.setVelocity(TPS);
            flywheelRPM = motorFlywheel.getVelocity() * 60 / CPR;

            telemetry.addData("Path State", pathState); // the current path the code is running
            telemetry.addData("Path Timer",pathTimer.getElapsedTimeSeconds());
            telemetry.addData("OpMode Timer", opmodeTimer.getElapsedTimeSeconds());
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Flywheel RPM", flywheelRPM);
            telemetry.addData("Shoot Timer", shootingTime);
            telemetry.update();

        }

        @Override
        public void init() {

            shooter.init(hardwareMap);

            motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");
            motorTransfer = hardwareMap.get(DcMotorEx.class, "motorTransfer");
            motorFlywheel = hardwareMap.get(DcMotorEx.class, "motorFlywheel");

            motorIntake.setDirection(DcMotorEx.Direction.FORWARD);
            motorTransfer.setDirection(DcMotorEx.Direction.FORWARD);
            motorFlywheel.setDirection(DcMotorEx.Direction.FORWARD);

            motorFlywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            motorIntake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorTransfer.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motorFlywheel.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

            PIDFCoefficients pidfNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, NEW_F);
            motorFlywheel.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

            shootTimer = new Timer();

            shooter.closeServoStop();
            shooter.downServoPaddle();

            shooter.centerServoTurret();

        }

        @Override
        public void init_loop() {
            // Nothing
        }

        @Override
        public void start() {
            opmodeTimer.resetTimer();
            setPathState(10);
        }


        @Override
        public void stop() {
            // Nothing
        }

    }