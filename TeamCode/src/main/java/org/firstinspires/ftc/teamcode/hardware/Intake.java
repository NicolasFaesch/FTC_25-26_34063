package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private static final double INTAKING_POWER = 1.0;
    private static final double OUTTAKING_POWER = -0.5;
    private static final double STORING_POWER = 0.4;

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        STORING
    }

    private State state;

    private DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        setState(State.IDLE);
    }
    public void setState(State state) {
        this.state = state;

        changeState();
    }

    public State getState() {
        return state;
    }

    private void changeState() {
        switch(state) {
            case IDLE:
                intakeMotor.setPower(0);
                break;
            case INTAKING:
                intakeMotor.setPower(INTAKING_POWER);
                break;
            case OUTTAKING:
                intakeMotor.setPower(OUTTAKING_POWER);
                break;
            case STORING:
                intakeMotor.setPower(STORING_POWER);
                break;
        }
    }

}
