package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "FTC Decode Test", group = "TeleOp")
public class FTC_Decode_Test extends OpMode {

    private enum RobotState {
        IDLE,
        INTAKING,
        SHOOTING
    }

    private enum FeederState {
        IDLE,
        EXTENDING,
        RETRACTING
    }

    RobotState robotState;

    FeederState feederState;

    private Limelight3A limelight;


    // Fahrmotoren
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    // Intake Motor
    DcMotor intake;

    // Transfer Motor
    DcMotor transfer;

    // Shooter Motor
    DcMotor shooterLeft;
    DcMotor shooterRight;


    Servo hood;

    //Servo feeder_stop;


    Servo feeder_servo;

    private ElapsedTime runtime = new ElapsedTime();

    // Intake Toggle
    boolean intakeOn = false;
    boolean lastAState = false;

    boolean feederOn = false;
    boolean lastBState = false;

    boolean shooterOn = false;
    boolean lastXState = false;

    double feederRetracted = 1.0;
    double feederExtended = 0.7;
    double feederTime = 0.3; // in seconds
    double feederIdleTime = 0.7; // time the feeder has to wait for ball to come into position
    double feederStartTime = 0.0;

    boolean lastYState = false;
    boolean lastLeftState = false;

    double hoodPosition = 0.8;
    double hoodMin = 0.3501;
    double hoodMax = 1.0;

    boolean lastRBState = false;
    boolean lastLBState = false;

    double shooterPower = 0.5;
    double shooterMin = 0.3;
    double shooterMax = 1.0;

    double intakePower = 1.0;
    double transferPower = 1.0;

    double intakePowerFeeding = 0.7;
    double transferPowerFeeding = 1.0;

    boolean lastUpState = false;
    boolean lastDownState = false;

    boolean red = true;

    //Pose2D targetPos;
    double targetX = 1.83;
    double targetY = 1.83;

    boolean manualOverride = false;





    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        frontLeft  = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft   = hardwareMap.dcMotor.get("backLeft");
        backRight  = hardwareMap.dcMotor.get("backRight");

        hood = hardwareMap.get(Servo.class, "hood");
        feeder_servo = hardwareMap.get(Servo.class, "feeder_servo");
        //feeder_stop = hardwareMap.get(Servo.class, "feeder_stop");
        intake = hardwareMap.dcMotor.get("intake");

        transfer = hardwareMap.dcMotor.get("transfer");

        shooterLeft = hardwareMap.dcMotor.get("shooterLeft");
        shooterRight = hardwareMap.dcMotor.get("shooterRight");

        // Motor-Richtungen (typisch für Mecanum)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        transfer.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);

        hood.setPosition(hoodPosition);
        feeder_servo.setPosition(feederRetracted);

        robotState = RobotState.IDLE;

        feederState = FeederState.IDLE;

        if(red) {
            //targetPos = new Pose2D(DistanceUnit.METER,2,-2,UNIT);
            targetX *= -1;
        }

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        limelight.start();

    }

    @Override
    public void loop() {
        // Joystick Werte
        double y = gamepad1.left_stick_y;   // vor/zurück
        double x = -gamepad1.left_stick_x;    // strafe
        double rx = -gamepad1.right_stick_x;  // drehen

        LLResult result = limelight.getLatestResult();
        if(result != null) {
            if(result.isValid()) {
                Pose3D botpose = result.getBotpose();
                double robotX = botpose.getPosition().x;
                double robotY = botpose.getPosition().y;
                double robotYaw = botpose.getOrientation().getYaw();

                double dX = targetX - robotX;
                double dY = targetY - robotY;

                double distance = Math.sqrt(dX*dX + dY*dY);

                double targetHeading = Math.toDegrees(Math.atan2(dY,dX));

                double headingError = targetHeading - robotYaw;
                if(headingError >= 360.0) {
                    headingError -= 360;
                }

                boolean currentYState = gamepad1.y;

                if (currentYState && !lastYState) {
                    manualOverride = !manualOverride;
                }


                telemetry.addData("Robot x",robotX);
                telemetry.addData("Robot y",robotY);
                telemetry.addData("Robot yaw",robotYaw);
                telemetry.addData("Distance",distance);
                telemetry.addData("Target Heading",targetHeading);
                telemetry.addData("Heading Error",headingError);

                if (robotState == RobotState.SHOOTING){ // slow down rotation
                    rx = headingError * 0.025;
                }

                shooterPower = 0.645 + distance*0.091;
                hoodPosition = 0.782 + distance*0.027;

            }
        }




        // Mecanum Berechnung
        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        // Normalisieren
        double max = Math.max(Math.abs(fl),
                Math.max(Math.abs(bl),
                        Math.max(Math.abs(fr), Math.abs(br))));

        if (max > 1.0) {
            fl /= max;
            bl /= max;
            fr /= max;
            br /= max;
        }

        // Motoren setzen
        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);



        // ---- Intake Toggle mit Button A ----
        boolean currentAState = gamepad1.a;

        if (currentAState && !lastAState) {
            if (robotState == RobotState.INTAKING) {
                robotState = RobotState.IDLE;
            } else {
                robotState = RobotState.INTAKING;
            }
        }

        lastAState = currentAState;

        // ---- Feeder Toggle mit Button B ----
        // boolean currentBState = gamepad1.b;
//
        // if (currentBState && !lastBState) {
        //     feederOn = !feederOn;
        // }
//
        // lastBState = currentBState;
//
        // if (feederOn) {
        //     transfer.setPower(1.0);
        // } else {
        //     transfer.setPower(0);
        // }

        // ---- Feeder stop----
        //boolean currentLeftState = gamepad1.dpad_left;
//
        //if (currentLeftState && !lastLeftState) {
        //    feederUp = !feederUp;
        //}
//
        //lastLeftState = currentLeftState;
//
        //if (feederUp) {
        //    feeder_stop.setPosition(0.8);
        //} else {
        //    feeder_stop.setPosition(0.6);
        //}


        // ---- Feeder----
        switch (feederState) {
            case IDLE:
                if(robotState == RobotState.SHOOTING && gamepad1.b
                        && runtime.seconds() >= feederIdleTime + feederStartTime) { //TODO: add speed requirement for shooter wheels
                    feeder_servo.setPosition(feederExtended);
                    feederState = FeederState.EXTENDING;
                    feederStartTime = runtime.seconds();
                }
                break;
            case EXTENDING:
                if (runtime.seconds() >= feederTime + feederStartTime) {
                    feeder_servo.setPosition(feederRetracted);
                    feederState = FeederState.RETRACTING;
                    feederStartTime = runtime.seconds();
                }
                break;
            case RETRACTING:
                if (runtime.seconds() >= feederTime + feederStartTime) {
                    feederState = FeederState.IDLE;
                    feederStartTime = runtime.seconds();
                }
                break;
            default:
                break;
        }



        // ---- Shooter Toggle mit Button X ----
        boolean currentXState = gamepad1.x;

        if (currentXState && !lastXState) {
            if (robotState == RobotState.SHOOTING) {
                robotState = RobotState.IDLE;
            } else {
                robotState = RobotState.SHOOTING;
            }
        }

        lastXState = currentXState;

        // ---- Shooter Hood Adjustement
        double step_size = 0.025;
        if (gamepad1.right_bumper && !lastRBState && manualOverride) {
            if (hoodPosition >= hoodMax) {
                hoodPosition = hoodMax;
            } else if (hoodPosition == hoodMin){
                hoodPosition = hoodMin + step_size - (hoodMin % step_size);
            } else {
                hoodPosition += step_size;
            }
        }

        if (gamepad1.left_bumper && !lastLBState && manualOverride) {
            if (hoodPosition <= hoodMin) {
                hoodPosition = hoodMin;
            } else if (hoodPosition - step_size < hoodMin){
                hoodPosition = hoodMin;
            } else {
                hoodPosition -= step_size;
            }
        }

        lastRBState = gamepad1.right_bumper;
        lastLBState = gamepad1.left_bumper;

        hood.setPosition(hoodPosition);


        // ---- Shooter Power Adjustement
        double step_size_shooter = 0.05;
        if (gamepad1.dpad_up && !lastUpState && manualOverride) {
            if (shooterPower >= shooterMax) {
                shooterPower = shooterMax;
            } else {
                shooterPower += step_size_shooter;
            }
        }

        if (gamepad1.dpad_down && !lastDownState && manualOverride) {
            if (shooterPower <= shooterMin) {
                shooterPower = shooterMin;
            } else {
                shooterPower -= step_size_shooter;
            }
        }

        lastUpState = gamepad1.dpad_up;
        lastDownState = gamepad1.dpad_down;

        // ---- Robot State Machine
        if (feederState == FeederState.IDLE) {
            switch(robotState) {
                case IDLE:
                    intake.setPower(0);
                    transfer.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    break;
                case INTAKING:
                    intake.setPower(intakePower);
                    transfer.setPower(transferPower);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    break;
                case SHOOTING:
                    intake.setPower(intakePowerFeeding);
                    transfer.setPower(transferPowerFeeding);
                    shooterLeft.setPower(shooterPower);
                    shooterRight.setPower(shooterPower);
                    break;
                default:
                    intake.setPower(0);
                    transfer.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    break;
            }

        } else { // feeder doing something
            intake.setPower(0);
            transfer.setPower(0);
            shooterLeft.setPower(shooterPower);
            shooterRight.setPower(shooterPower);
        }


        // Telemetrie
        telemetry.addData("Robot State", robotState);
        telemetry.addData("Feeder State", feederState);
        telemetry.addData("Manual Override", manualOverride);
        telemetry.addData("hood Servo Position", hoodPosition);
        telemetry.addData("power shooter", shooterPower);
        telemetry.addData("elapsed time", runtime.seconds());
        telemetry.update();
    }
}
