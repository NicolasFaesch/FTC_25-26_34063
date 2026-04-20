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


public class RobotTeleOp extends Robot {

    public DrivetrainTeleOp drivetrainTeleOp;

    double previousTime = 0;
    Gamepad gamepad1, gamepad2;


    public RobotTeleOp(HardwareMap hardwareMap, Alliance alliance, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, alliance);
        drivetrainTeleOp = new DrivetrainTeleOp(hardwareMap);
        setDrivetrain(drivetrainTeleOp);

        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // TODO: set up auto parking poses
        /*
        if (alliance == Alliance.RED) {
            drivetrainTeleOp.setParkingPose(PoseStorage.parkingPoseRED);
            drivetrainTeleOp.setParkingHeading(PoseStorage.parkingHeadingRED);
        } else {
            drivetrainTeleOp.setParkingPose(PoseStorage.parkingPoseBLUE);
            drivetrainTeleOp.setParkingHeading(PoseStorage.parkingHeadingBLUE);
        }

         */

    }

    public void update(double currentTime) throws Exception {
        // TODO: auto parking engagement
        /*
        if (gamepad1.dpadLeftWasPressed()) {
            setState(State.PARKING);
            drivetrainTeleOp.park();
        }

        if (getState() != State.PARKING) {
            drivetrainTeleOp.noParking();
        }
        */

        double drivingScalar = 1 - gamepad1.left_trigger;

        drivetrainTeleOp.update(gamepad1.left_stick_x, gamepad1.left_stick_y,
                gamepad1.right_stick_x, drivingScalar);
        super.update();

        /*
        //Gamepad 2 adjusting
        if (gamepad2.dpadLeftWasPressed()) {
            //shooter.adjustShooterSpeed(false);
            //PoseStorage.parkingCorrectionX += 0.03;
        } else if (gamepad2.dpadRightWasPressed()) {
            //shooter.adjustShooterSpeed(true);
            //PoseStorage.parkingCorrectionX -= 0.03;
        } else if (gamepad2.dpadUpWasPressed()) {
            shooter.adjustHood(true, myPos);
            //PoseStorage.parkingCorrectionY += 0.03;
        } else if (gamepad2.dpadDownWasPressed()) {
            shooter.adjustHood(false, myPos);
            //PoseStorage.parkingCorrectionY -= 0.03;
        }


        if (controller2.getLeftBumper() == Controller.ButtonState.ON_PRESS) {
            drivetrainTeleOp.adjustAutoAimHeading(autoAimHeadingAdjustment, myPos);
        } else if (controller2.getRightBumper() == Controller.ButtonState.ON_PRESS) {
            drivetrainTeleOp.adjustAutoAimHeading((autoAimHeadingAdjustment * -1), myPos);
        }

        //Reset adjustment
        if (controller2.getRightJoystickButton() == Controller.ButtonState.ON_PRESS || controller1.getRightJoystickButton() == Controller.ButtonState.ON_PRESS) {
            shooter.resetShooterSpeedAdjustment();
            shooter.resetHoodAdjustment();
            drivetrainTeleOp.resetAutoAimHeadingAdjustment();
        }
        */

        switch (state) {
            case AIMING:
            case SHOOTING:
                if (!validShootingPose) {
                    colorLED.setColor(ColorLED.Color.RED);
                } else if (!turretReady) {
                    colorLED.setColor(ColorLED.Color.ORANGE);
                } else {
                    colorLED.setColor(blockerDisengaged ? ColorLED.Color.PURPLE : ColorLED.Color.GREEN);
                }
                break;
            default:
                colorLED.setColor(ColorLED.Color.OFF);
        }

        if (gamepad1.yWasPressed()) {
            setState(State.IDLE);
        }

        if (gamepad1.rightBumperWasPressed()) {
            if (state == State.INTAKING) {
                setState(State.STORING);
            } else {
                setState(State.INTAKING);
            }
        }

        if (gamepad1.right_trigger > 0.1) {
            setState(State.OUTTAKING);
        } else if (getState() == State.OUTTAKING) {
            setState(State.INTAKING);
        }

        if (gamepad1.leftBumperWasPressed()) {
            if (state == State.AIMING || state == State.SHOOTING) {
                setState(State.STORING);
            } else {
                setState(State.AIMING);
            }
        }

        if (gamepad1.aWasPressed() || gamepad1.xWasPressed()) { //TODO: improve by splitting button roles
            if (state != State.SHOOTING) {
                setState(State.SHOOTING);
            } else {
                setState(State.AIMING);
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            shooter.increaseManualHoodPos();
        }

        if (gamepad2.dpadDownWasPressed()) {
            shooter.decreaseManualHoodPos();
        }

        if (gamepad2.dpadRightWasPressed()) {
            shooter.increaseManualShooterVel();
        }

        if (gamepad2.dpadLeftWasPressed()) {
            shooter.decreaseManualShooterVel();
        }

        if (gamepad2.aWasPressed()) {
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
        panelsTelemetry.addLine(position);
        panelsTelemetry.addData("Distance", DynamicAiming.getTargetDistance());

        telemetry.addLine("=== POSE ===");
        telemetry.addLine(position);
        telemetry.addData("Distance", DynamicAiming.getTargetDistance());

        // Shooter Info
        panelsTelemetry.addLine("=== SHOOTER ===");
        if (shooter.getManualOverride()) {
            panelsTelemetry.addLine("Shooter: MANUAL OVERRIDE");
            panelsTelemetry.addData("Hood Position (manual)", shooter.getHoodPositionManual());
            panelsTelemetry.addData("Target Velocity (manual)", shooter.getShooterTargetVelocityManual());
            panelsTelemetry.addData("Current Velocity", shooter.getShooterVelocity());
            panelsTelemetry.addData("Target Angle (manual)", turret.getTargetAngleManual());
            panelsTelemetry.addData("Turret Angle", turret.getTurretAngle());
        } else {
            panelsTelemetry.addData("Hood Position", shooter.getHoodPosition());
            panelsTelemetry.addData("Target Velocity", shooter.getShooterTargetVelocity());
            panelsTelemetry.addData("Current Velocity", shooter.getShooterVelocity());
            panelsTelemetry.addData("Target Angle", turret.getTargetAngle());
            panelsTelemetry.addData("Turret Angle", turret.getTurretAngle());
        }


        telemetry.addLine("=== SHOOTER ===");
        if (shooter.getManualOverride()) {
            telemetry.addLine("Shooter: MANUAL OVERRIDE");
            telemetry.addData("Hood Position (manual)", shooter.getHoodPositionManual());
            telemetry.addData("Target Velocity (manual)", shooter.getShooterTargetVelocityManual());
            telemetry.addData("Current Velocity", shooter.getShooterVelocity());
            telemetry.addData("Target Angle (manual)", turret.getTargetAngleManual());
            telemetry.addData("Turret Angle", turret.getTurretAngle());
        } else {
            telemetry.addData("Hood Position", shooter.getHoodPosition());
            telemetry.addData("Target Velocity", shooter.getShooterTargetVelocity());
            telemetry.addData("Current Velocity", shooter.getShooterVelocity());
            telemetry.addData("Target Angle", turret.getTargetAngle());
            telemetry.addData("Turret Angle", turret.getTurretAngle());
        }


        // Update PanelsTelemetry
        //drawPath();
        drawDebug(drivetrainTeleOp.getFollower());
        panelsTelemetry.update();
        telemetry.update();
    }


}
