package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Controller;


public class RobotTeleOp extends Robot{

    private DrivetrainTeleOp drivetrainTeleOp;

    private Controller controller1;
    private Controller controller2;

    private Gamepad gamepad1;
    private Gamepad gamepad2;


    public RobotTeleOp(HardwareMap hardwareMap, Alliance alliance, Pose2D startPose) {
        super(hardwareMap, alliance);
        drivetrainTeleOp = new DrivetrainTeleOp(hardwareMap,startPose);

        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);
    }

    public void update(double loopTime) {
        super.update();
        drivetrainTeleOp.update(controller1.getLeftJoystickXValue(), controller1.getLeftJoystickYValue(),
                controller1.getRightJoystickXValue(), loopTime);
        limelight.update(drivetrainTeleOp.getPose().getHeading(AngleUnit.DEGREES));
        shooter.update(drivetrainTeleOp.getDistance(), true);

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
    }






}
