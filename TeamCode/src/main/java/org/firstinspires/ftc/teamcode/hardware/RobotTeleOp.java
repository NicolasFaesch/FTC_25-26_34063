package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.lib.Drawing.drawDebug;
import static org.firstinspires.ftc.teamcode.lib.Drawing.drawRobot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.DynamicAiming;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.PositionChecker;
import org.firstinspires.ftc.teamcode.lib.ShooterLUT;

import java.util.Locale;


public class RobotTeleOp extends Robot{

    private static final double AIMING_MAX_HEADING_ERROR = 3;

    //Possible adjustment per frame in radian.
    private static final double autoAimHeadingAdjustment = 0.03;

    public DrivetrainTeleOp drivetrainTeleOp;

    private Controller controller1, controller2;

    double previousTime = 0;


    public RobotTeleOp(HardwareMap hardwareMap, Alliance alliance, Pose2D startPose, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, alliance);
        drivetrainTeleOp = new DrivetrainTeleOp(hardwareMap,startPose);
        drivetrainTeleOp.setTargetPose((alliance==Alliance.RED) ? PoseStorage.targetPoseRed :PoseStorage.targetPoseBlue);

        if(alliance == Alliance.RED) {
            drivetrainTeleOp.setParkingPose(PoseStorage.parkingPoseRED);
            drivetrainTeleOp.setParkingHeading(PoseStorage.parkingHeadingRED);
        } else {
            drivetrainTeleOp.setParkingPose(PoseStorage.parkingPoseBLUE);
            drivetrainTeleOp.setParkingHeading(PoseStorage.parkingHeadingBLUE);
        }

        DynamicAiming.createLUTs();
        DynamicAiming.setTargetPose(drivetrainTeleOp.getTargetPose());

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    public void update(double currentTime) throws Exception {
        //Update pos.
        myPos = drivetrainTeleOp.getPose();


        double loopTime = currentTime-previousTime;
        previousTime = currentTime;

        //Slow driving speed while button hold.
        if(controller1.getLeftTrigger() == Controller.ButtonState.PRESSED) {
            drivetrainTeleOp.slowDrivetrain();
        } else {
            drivetrainTeleOp.normalDrivetrain();
        }

        if(controller1.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            setState(State.PARKING);
            drivetrainTeleOp.park();
        }

        if(getState() != State.PARKING) {
            drivetrainTeleOp.noParking();
        }

        //Gamepad 2 adjusting
        if(controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            //shooter.adjustShooterSpeed(false);
            //PoseStorage.parkingCorrectionX += 0.03;
        } else if(controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) {
            //shooter.adjustShooterSpeed(true);
            //PoseStorage.parkingCorrectionX -= 0.03;
        } else if(controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
            shooter.adjustHood(true,myPos);
            //PoseStorage.parkingCorrectionY += 0.03;
        } else if(controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
            shooter.adjustHood(false,myPos);
            //PoseStorage.parkingCorrectionY -= 0.03;
        }

        if(controller2.getLeftBumper() == Controller.ButtonState.ON_PRESS) {
            drivetrainTeleOp.adjustAutoAimHeading(autoAimHeadingAdjustment,myPos);
        } else if(controller2.getRightBumper() == Controller.ButtonState.ON_PRESS) {
            drivetrainTeleOp.adjustAutoAimHeading((autoAimHeadingAdjustment*-1),myPos);
        }


        //Reset adjustment
        if(controller2.getRightJoystickButton() == Controller.ButtonState.ON_PRESS || controller1.getRightJoystickButton() == Controller.ButtonState.ON_PRESS) {
            shooter.resetShooterSpeedAdjustment();
            shooter.resetHoodAdjustment();
            drivetrainTeleOp.resetAutoAimHeadingAdjustment();
        }



        //drivetrainTeleOp.update(controller1.getLeftJoystickXValue(), controller1.getLeftJoystickYValue(),
        //        controller1.getRightJoystickXValue(), loopTime, myPos);
        super.update(drivetrainTeleOp.getPose());

        controller1.update();
        controller2.update();

        /*
        Pose2D limelightPose;
        if(controller2.getLeftJoystickButton() == Controller.ButtonState.PRESSED) { // use MT1 for calib
            limelightPose = getLimelightPose(drivetrainTeleOp.getVelocityX(), drivetrainTeleOp.getVelocityY(), drivetrainTeleOp.getAngularVelocity(), false);
        } else { // otherwise MT2
            limelightPose = getLimelightPose(drivetrainTeleOp.getVelocityX(), drivetrainTeleOp.getVelocityY(), drivetrainTeleOp.getAngularVelocity(), true);
        }
        if (isUsingLimelight()) {
            drivetrainTeleOp.overridePose(limelightPose);
        }
        */

        boolean validShootingPose = drivetrainTeleOp.getDistance() >= ShooterLUT.minDistance;
        validShootingPose &= PositionChecker.checkInZones(drivetrainTeleOp.getPose());
        boolean validShootingState = true;//Math.abs(drivetrainTeleOp.getHeadingError()) <= AIMING_MAX_HEADING_ERROR;
        //TODO: fix state

        switch (state) {
            case AIMING:
            case SHOOTING:
                if(!validShootingPose) {
                    colorLED.setColor(ColorLED.Color.RED);
                } else if (!validShootingState) {
                    colorLED.setColor(ColorLED.Color.ORANGE);
                } else {
                    colorLED.setColor((state == State.AIMING) ? ColorLED.Color.GREEN : ColorLED.Color.PURPLE);
                }
                break;
            default:
                colorLED.setColor(ColorLED.Color.OFF);
        }

        if(controller1.getLeftTriggerValue() > 0.5) { // TODO: remove hack
            shooter.update(drivetrainTeleOp.getDistance(), validShootingPose && validShootingState,myPos);
            drivetrainTeleOp.update(controller1.getLeftJoystickXValue(), controller1.getLeftJoystickYValue(),
                    controller1.getRightJoystickXValue(), loopTime, myPos);
        } else {
            DynamicAiming.ShooterState targetingVals = DynamicAiming.calculateTargeting(
                    drivetrainTeleOp.getPose().getX(DistanceUnit.METER), drivetrainTeleOp.getPose().getY(DistanceUnit.METER),
                    drivetrainTeleOp.getPose().getHeading(AngleUnit.DEGREES), drivetrainTeleOp.getVelocityX(),
                    drivetrainTeleOp.getVelocityY(), drivetrainTeleOp.getAngularVelocity()
            );
            shooter.dynamicAimingUpdate(targetingVals.hoodAngle, targetingVals.flywheelRpm, validShootingPose);
            drivetrainTeleOp.updateDynamic(controller1.getLeftJoystickXValue(), controller1.getLeftJoystickYValue(),
                    controller1.getRightJoystickXValue(), loopTime, myPos, Math.toDegrees(targetingVals.turretAngle));
        }



        if (controller1.getyButton() == Controller.ButtonState.ON_PRESS) {
            setState(State.IDLE);
        }

        if(controller1.getRightBumper() == Controller.ButtonState.ON_PRESS) {
            if (state == State.INTAKING) {
                setState(State.STORING);
            } else {
                setState(State.INTAKING);
            }
        }

        if(controller1.getRightTrigger() == Controller.ButtonState.PRESSED) {
            setState(State.OUTTAKING);
        } else if (controller1.getRightTrigger() == Controller.ButtonState.ON_RELEASE) {
            setState(State.INTAKING);
        }

        if (controller1.getLeftBumper() == Controller.ButtonState.ON_PRESS) {
            if (state == State.AIMING || state == State.SHOOTING) {
                setState(State.STORING);
            } else {
                setState(State.AIMING);
            }
        }

        if(controller1.getaButton() == Controller.ButtonState.PRESSED || controller1.getxButton() == Controller.ButtonState.PRESSED) {
            setState(State.SHOOTING);
        } else if (controller1.getaButton() == Controller.ButtonState.ON_RELEASE || controller1.getxButton() == Controller.ButtonState.ON_RELEASE) {
            setState(State.AIMING);
        }

        // enable auto aim
        drivetrainTeleOp.setAimingMode(state == State.AIMING || state == State.SHOOTING);

        if(controller2.getdPadUp() == Controller.ButtonState.ON_PRESS) {
            shooter.increaseManualHoodPos();
        }

        if(controller2.getdPadDown() == Controller.ButtonState.ON_PRESS) {
            shooter.decreaseManualHoodPos();
        }

        if(controller2.getdPadRight() == Controller.ButtonState.ON_PRESS) {
            shooter.increaseManualShooterVel();
        }

        if(controller2.getdPadLeft() == Controller.ButtonState.ON_PRESS) {
            shooter.decreaseManualShooterVel();
        }

        if (controller2.getaButton() == Controller.ButtonState.ON_PRESS) {
            shooter.setManualOverride(!shooter.getManualOverride());
        }
    }


    public void updateTelemetry(TelemetryManager panelsTelemetry, Telemetry telemetry) {
        // Position
        Pose2D botPose = drivetrainTeleOp.getPose();
        String position = String.format(Locale.US, "X: %.2f, Y: %.2f, H: %.1f",
                botPose.getX(DistanceUnit.INCH),
                botPose.getY(DistanceUnit.INCH),
                botPose.getHeading(AngleUnit.DEGREES));


        // Robot States
        panelsTelemetry.addLine("=== ROBOT STATES ===");
        panelsTelemetry.addData("Robot State", getState());
        panelsTelemetry.addData("Intake State", intake.getState());
        panelsTelemetry.addData("Transfer State", transfer.getState());
        panelsTelemetry.addData("Shooter State", shooter.getState());

        telemetry.addLine("=== ROBOT STATES ===");
        telemetry.addData("Robot State", getState());
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Transfer State", transfer.getState());
        telemetry.addData("Shooter State", shooter.getState());

        // Pose
        panelsTelemetry.addLine("=== POSE ===");
        if (!isUsingLimelight()) {
            panelsTelemetry.addData("Odometry", position);
        } else {
            panelsTelemetry.addData("Limelight", position);
        }
        panelsTelemetry.addData("Distance", drivetrainTeleOp.getDistance());
        panelsTelemetry.addData("Velocity X", drivetrainTeleOp.getVelocityX());
        panelsTelemetry.addData("Velocity Y", drivetrainTeleOp.getVelocityY());

        telemetry.addLine("=== POSE ===");
        if (!isUsingLimelight()) {
            telemetry.addData("Odometry", position);
        } else {
            telemetry.addData("Limelight", position);
        }
        telemetry.addData("Distance", drivetrainTeleOp.getDistance());
        telemetry.addData("Velocity X", drivetrainTeleOp.getVelocityX());
        telemetry.addData("Velocity Y", drivetrainTeleOp.getVelocityY());

        // Shooter Info
        panelsTelemetry.addLine("=== SHOOTER ===");
        if (shooter.getManualOverride()) {
            panelsTelemetry.addLine("Shooter: MANUAL OVERRIDE");
            panelsTelemetry.addData("Hood Position (manual)", shooter.getHoodPositionManual());
            panelsTelemetry.addData("Target Velocity (manual)", shooter.getShooterTargetVelocityManual());
        } else {
            panelsTelemetry.addData("Hood Position", shooter.getHoodPosition());
            panelsTelemetry.addData("Target Velocity", shooter.getShooterTargetVelocity());
        }
        panelsTelemetry.addData("Current Velocity", shooter.getShooterVelocity());

        telemetry.addLine("=== SHOOTER ===");
        if (shooter.getManualOverride()) {
            telemetry.addLine("Shooter: MANUAL OVERRIDE");
            telemetry.addData("Hood Position (manual)", shooter.getHoodPositionManual());
            telemetry.addData("Target Velocity (manual)", shooter.getShooterTargetVelocityManual());
        } else {
            telemetry.addData("Hood Position", shooter.getHoodPosition());
            telemetry.addData("Target Velocity", shooter.getShooterTargetVelocity());
        }
        telemetry.addData("Current Velocity", shooter.getShooterVelocity());

        telemetry.addLine("=== ADJUSTMENT ===");
        telemetry.addData("HoodFarAdjustment", shooter.getHoodFarAdjustment());
        telemetry.addData("HoodNearAdjustment", shooter.getHoodNearAdjustment());



        // Update PanelsTelemetry
        //drawPath();
        drawDebug(drivetrainTeleOp.getFollower());
        panelsTelemetry.update();
        telemetry.update();
    }




}
