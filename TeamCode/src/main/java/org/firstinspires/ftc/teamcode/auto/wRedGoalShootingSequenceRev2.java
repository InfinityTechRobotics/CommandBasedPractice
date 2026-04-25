    package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Hardware.FlywheelSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.ShooterSpinfinityDuo;
import org.firstinspires.ftc.teamcode.Hardware.SpintakeSpinfinity;

//    @Disabled
    @Autonomous
        public class wRedGoalShootingSequenceRev2 extends OpMode {

        ShooterSpinfinityDuo shooter = new ShooterSpinfinityDuo();
        FlywheelSpinfinityDuo flywheel = new FlywheelSpinfinityDuo();
        SpintakeSpinfinity spintake = new SpintakeSpinfinity();

        double START_DELAY_TIME = 2.;

        double shootingTime = 0.;

        private Timer pathTimer, opmodeTimer;
        private Timer shootTimer;
        private int pathState;

        public static double NEW_P = 100.;   // 10.
        public static double NEW_I = 1.;    // 3.
        public static double NEW_D = 20.;    // 0.
        public static double NEW_F = 3.5;    // 0.

        double targetRPM = 0.;
        double flywheelRPM = 0.;
        double TARGET_AUTON_RPM = 2325.;


        public void autonomousPathUpdate() {
            switch (pathState) {
                case 10:
                    shooter.closeServoStop();
                    shooter.downServoPaddle();
                    setPathState(11);
                    break;
                case 11:
                    if (pathTimer.getElapsedTimeSeconds() > START_DELAY_TIME) {
                        setPathState(1001);
                    }
                    break;
                case 1001:
                    spintake.turnIntakeOn();
                    setPathState(10001);
                    break;
                case 10001:
                    if (pathTimer.getElapsedTimeSeconds() > 0.1) { // changed from 0.5 to 0.25
                        shootTimer.resetTimer();
                        shooter.openServoStop();
                        setPathState(10006);
                    }
                    break;
                case 10006:
                    if (pathTimer.getElapsedTimeSeconds() > 0.45) {
                        shooter.shootServoPaddle();
                        setPathState(10007);
                    }
                    break;
                case 10007:
                    if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                        shootingTime = shootTimer.getElapsedTimeSeconds();
                        shooter.downServoPaddle();
                        shooter.closeServoStop();
                        spintake.turnIntakeOff();
                        setPathState(10009);
                    }
                    break;
                case 10009: // updates shooting sequence flag
                    if (pathTimer.getElapsedTimeSeconds() > 0.1) {
                        setPathState(999);
                    }
                    break;
                case 999: // last state, just stops and waits
                    if(pathTimer.getElapsedTimeSeconds() > 1) {
                        TARGET_AUTON_RPM = 0.;
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
            flywheel.setFlywheelVel(targetRPM);

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
            flywheel.init(hardwareMap);
            spintake.init(hardwareMap);

            pathTimer = new Timer();
            opmodeTimer = new Timer();
            opmodeTimer.resetTimer();

            shootTimer = new Timer();

            shooter.closeServoStop();
            shooter.downServoPaddle();

            shooter.centerMotorTurret();

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