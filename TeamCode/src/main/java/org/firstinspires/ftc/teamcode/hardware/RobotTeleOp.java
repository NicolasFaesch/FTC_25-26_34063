package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Controller;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;
import org.firstinspires.ftc.teamcode.lib.PositionChecker;
import org.firstinspires.ftc.teamcode.lib.ShooterLUT;


public class RobotTeleOp extends Robot{

    private static final double AIMING_MAX_HEADING_ERROR = 3;

    public DrivetrainTeleOp drivetrainTeleOp;

    private Controller controller1, controller2;



    public RobotTeleOp(HardwareMap hardwareMap, Alliance alliance, Pose2D startPose, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, alliance);
        drivetrainTeleOp = new DrivetrainTeleOp(hardwareMap,startPose);
        drivetrainTeleOp.setTargetPose((alliance==Alliance.RED) ? PoseStorage.targetPoseRed :PoseStorage.targetPoseBlue);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    public void update(double loopTime) throws Exception {
        drivetrainTeleOp.update(controller1.getLeftJoystickXValue(), controller1.getLeftJoystickYValue(),
                controller1.getRightJoystickXValue(), loopTime);
        super.update(drivetrainTeleOp.getPose());

        controller1.update();
        controller2.update();


        Pose2D limelightPose;
        if(controller2.getLeftJoystickButton() == Controller.ButtonState.PRESSED) { // use MT1 for calib
            limelightPose = getLimelightPose(drivetrainTeleOp.getVelocityX(), drivetrainTeleOp.getVelocityY(), drivetrainTeleOp.getAngularVelocity(), false);
        } else { // otherwise MT2
            limelightPose = getLimelightPose(drivetrainTeleOp.getVelocityX(), drivetrainTeleOp.getVelocityY(), drivetrainTeleOp.getAngularVelocity(), true);
        }
        if (isUsingLimelight()) {
            drivetrainTeleOp.overridePose(limelightPose);
        }


        boolean validShootingPose = drivetrainTeleOp.getDistance() >= ShooterLUT.minDistance;
        validShootingPose &= PositionChecker.checkInZones(drivetrainTeleOp.getPose());

        if(validShootingPose) {
            colorLED.setColor(ColorLED.Color.GREEN);
        } else {
            colorLED.setColor(ColorLED.Color.RED);
        }

        validShootingPose &= Math.abs(drivetrainTeleOp.getHeadingError()) <= AIMING_MAX_HEADING_ERROR;

        shooter.update(drivetrainTeleOp.getDistance(), validShootingPose);

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






}
