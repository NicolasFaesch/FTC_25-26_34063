package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name = "FTC Decode Test", group = "TeleOp")
public class FTC_Decode_Test extends OpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

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

    InterpLUT shooterVelocityLUT = new InterpLUT();
    InterpLUT hoodPositionLUT = new InterpLUT();

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

    //private MotorGroup shooter;

    DcMotorEx shooterLeft;
    DcMotorEx shooterRight;


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

    double feederRetracted = 0.95; // 0.95
    double feederExtended = 0.6;
    double feederTime = 0.12; // in seconds              0.3
    double feederIdleTime = 0.15; // time the feeder has to wait for ball to come into position  0.7
    double feederStartTime = 0.0;

    boolean lastYState = false;
    boolean lastLeftState = false;

    double hoodPosition = 0.8;
    double hoodMin = 0.3501;
    double hoodMax = 1.0;

    boolean lastRBState = false;
    boolean lastLBState = false;

    double shooterVelocity = 3000;
    double shooterMin = 2000;
    double shooterMax = 4000;

    double velocityTreshold = 100;

    // shooter PID
    double shooter_kP = 20;
    double shooter_kF = 0.7;

    double shooterCPR = 28;

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

    Pose2D startPos = new Pose2D(DistanceUnit.METER, -2, 0, AngleUnit.RADIANS, 0.5*Math.PI);

    double llVelThreshold = 0.05; // Threshold for limelight Position update in m/s
    double llHeadingVelThreshold = 5.0; // Threshold for limelight Position update in deg/s

    double autoAim_kP = 0.025; // unit: /deg
    double autoAim_kD = 0.0005; // unit: s/deg

    double previousHeadingError = 0;
    double headingErrorThreshold = 3; // in deg
    double oldTime = 0;

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

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");

        odo.setOffsets(-48.0, -115.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //odo.resetPosAndIMU();

        odo.setPosition(startPos);

        // Motor-Richtungen (typisch für Mecanum)
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        transfer.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        PIDFCoefficients shooterPIDF = new PIDFCoefficients(shooter_kP, 0, 0, shooter_kF);



        hood.setPosition(hoodPosition);
        feeder_servo.setPosition(feederRetracted);

        robotState = RobotState.IDLE;

        feederState = FeederState.IDLE;

        if(red) {
            //targetPos = new Pose2D(DistanceUnit.METER,2,-2,UNIT);
            targetX *= -1;
        }

        // add LUT points
        hoodPositionLUT.add(0.0,0.6);
        shooterVelocityLUT.add(0.0,2750);
        hoodPositionLUT.add(0.75,0.425);
        shooterVelocityLUT.add(0.75,2500);
        hoodPositionLUT.add(0.9,0.6);
        shooterVelocityLUT.add(0.9,2750);
        hoodPositionLUT.add(1.2,0.6);
        shooterVelocityLUT.add(1.2,2750);
        hoodPositionLUT.add(1.3,0.675);
        shooterVelocityLUT.add(1.3,2750);
        hoodPositionLUT.add(1.4,0.725);
        shooterVelocityLUT.add(1.4,3000);
        hoodPositionLUT.add(1.5,0.75);
        shooterVelocityLUT.add(1.5,3000);
        hoodPositionLUT.add(1.6,0.75);
        shooterVelocityLUT.add(1.6,3000);
        hoodPositionLUT.add(1.7,0.775);
        shooterVelocityLUT.add(1.7,3000);
        hoodPositionLUT.add(1.8,0.775);
        shooterVelocityLUT.add(1.8,3100);
        hoodPositionLUT.add(1.9,0.775);
        shooterVelocityLUT.add(1.9,3150);
        hoodPositionLUT.add(2.0,0.775);
        shooterVelocityLUT.add(2.0,3250);
        hoodPositionLUT.add(2.1,0.8);
        shooterVelocityLUT.add(2.1,3250);
        hoodPositionLUT.add(2.2,0.8);
        shooterVelocityLUT.add(2.2,3250);
        hoodPositionLUT.add(2.3,0.825);
        shooterVelocityLUT.add(2.3,3300);
        hoodPositionLUT.add(2.4,0.85);
        shooterVelocityLUT.add(2.4,3350);
        hoodPositionLUT.add(2.5,0.85);
        shooterVelocityLUT.add(2.5,3500);
        hoodPositionLUT.add(2.6,0.85);
        shooterVelocityLUT.add(2.6,3500);
        hoodPositionLUT.add(2.7,0.85);
        shooterVelocityLUT.add(2.7,3500);
        hoodPositionLUT.add(3.7,0.875);
        shooterVelocityLUT.add(3.7,4000);
        hoodPositionLUT.add(10.0,0.875);
        shooterVelocityLUT.add(10.0,4000);

        //generating final equation
        hoodPositionLUT.createLUT();
        shooterVelocityLUT.createLUT();

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        limelight.start();

    }

    @Override
    public void loop() {
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;

        // Joystick Werte
        double y = gamepad1.left_stick_y;   // vor/zurück
        double x = -gamepad1.left_stick_x;    // strafe
        double rx = -gamepad1.right_stick_x;  // drehen


        odo.update();

        Pose2D botpose = odo.getPosition();
        double robotVelX = odo.getVelX(DistanceUnit.METER);
        double robotVelY = odo.getVelY(DistanceUnit.METER);
        double robotAbsVel = Math.sqrt(robotVelX*robotVelX+robotVelY*robotVelY);
        double robotHeadingVel = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);


        LLResult llresult = null;

        if(robotAbsVel < llVelThreshold && robotHeadingVel < llHeadingVelThreshold) {
            limelight.updateRobotOrientation(botpose.getHeading(AngleUnit.DEGREES));
            llresult = limelight.getLatestResult();
        }

        boolean usingLimelight = false;
        if(llresult != null) {
            if(llresult.isValid()) {
                usingLimelight = true;
                Pose3D botpose3D = llresult.getBotpose();
                botpose = new Pose2D(DistanceUnit.METER, botpose3D.getPosition().x, botpose3D.getPosition().y,
                        AngleUnit.DEGREES, botpose3D.getOrientation().getYaw());
                odo.setPosition(botpose);
            }
        }
        telemetry.addData("Using Limelight Localization", usingLimelight);

        /* Auto Aiming stuff */
        double robotX = botpose.getX(DistanceUnit.METER);
        double robotY = botpose.getY(DistanceUnit.METER);
        double robotYaw = botpose.getHeading(AngleUnit.DEGREES);

        double dX = targetX - robotX;
        double dY = targetY - robotY;

        double distance = Math.sqrt(dX*dX + dY*dY);

        double targetHeading = Math.toDegrees(Math.atan2(dY,dX));


        double headingError = targetHeading - robotYaw;
        if(headingError >= 180.0) {
            headingError -= 360;
        }


        //  telemetry.addData("Robot x",robotX);
        //   telemetry.addData("Robot y",robotY);
        // telemetry.addData("Robot yaw",robotYaw);
        telemetry.addData("Distance",distance);
        telemetry.addData("Target Heading",targetHeading);
        telemetry.addData("Heading Error",headingError);

        if (robotState == RobotState.SHOOTING){ // slow down rotation
            rx = headingError * autoAim_kP + (headingError-previousHeadingError)/loopTime*autoAim_kD;
        }

        previousHeadingError = headingError;

        if(!manualOverride) {
            shooterVelocity = shooterVelocityLUT.get(distance);
            hoodPosition = hoodPositionLUT.get(distance);
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
        double shooterLeftVelocity = shooterLeft.getVelocity() / shooterCPR * 60;
        double shooterRightVelocity = shooterRight.getVelocity() / shooterCPR * 60;

        // ---- Feeder----
        switch (feederState) {
            case IDLE:
                if(robotState == RobotState.SHOOTING && gamepad1.b
                        && runtime.seconds() >= feederIdleTime + feederStartTime
                        && Math.abs(shooterLeftVelocity - shooterVelocity) < velocityTreshold
                        && Math.abs(headingError) < headingErrorThreshold) {
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

        boolean currentYState = gamepad1.y;

        if (currentYState && !lastYState) {
            manualOverride = !manualOverride;
        }

        lastYState = currentYState;


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
        double step_size_shooter = 250;
        if (gamepad1.dpad_up && !lastUpState && manualOverride) {
            if (shooterVelocity >= shooterMax) {
                shooterVelocity = shooterMax;
            } else {
                shooterVelocity += step_size_shooter;
            }
        }

        if (gamepad1.dpad_down && !lastDownState && manualOverride) {
            if (shooterVelocity <= shooterMin) {
                shooterVelocity = shooterMin;
            } else {
                shooterVelocity -= step_size_shooter;
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
                    shooterLeft.setVelocity(shooterVelocity / 60 * shooterCPR); // in ticks per second
                    shooterRight.setVelocity(shooterVelocity / 60 * shooterCPR);
                    break;
                default:
                    intake.setPower(0);
                    transfer.setPower(0);
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    break;
            }

        } else { // feeder doing something
            intake.setPower(intakePowerFeeding);
            transfer.setPower(0);
            shooterLeft.setVelocity(shooterVelocity / 60 * shooterCPR); // in ticks per second
            shooterRight.setVelocity(shooterVelocity / 60 * shooterCPR);
        }


        // Telemetrie
        String position = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", botpose.getX(DistanceUnit.METER), botpose.getY(DistanceUnit.METER), botpose.getHeading(AngleUnit.DEGREES));
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", odo.getVelX(DistanceUnit.METER), odo.getVelY(DistanceUnit.METER), odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES));

        telemetry.addData("Position", position);
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Robot State", robotState);
        telemetry.addData("Feeder State", feederState);
        telemetry.addData("Manual Override", manualOverride);
        telemetry.addData("hood Servo Position", hoodPosition);
        telemetry.addData("shooter target velocity", shooterVelocity);
        telemetry.addData("shooter velocity left", shooterLeftVelocity);
        telemetry.addData("shooter velocity right", shooterRightVelocity);
        telemetry.addData("elapsed time", runtime.seconds());
        telemetry.update();
    }
}
