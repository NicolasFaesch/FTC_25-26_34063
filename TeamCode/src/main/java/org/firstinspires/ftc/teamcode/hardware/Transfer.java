package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    private static final double INTAKING_POWER = 1.0;
    private static final double OUTTAKING_POWER = -0.5;
    private static final double STORING_POWER = 0.4;
    private static final double FEEDING_POWER = 0.7;

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        STORING,
        FEEDING
    }

    private State state;

    private DcMotorEx transferMotor;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class,"transfer");

        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setDirection(DcMotorEx.Direction.REVERSE);

        setState(State.IDLE);
    }
    public void setState(State state) {
        if(state != this.state) {
            this.state = state;

            changeState();
        }
    }

    public State getState() {
        return state;
    }

    private void changeState() {
        switch(state) {
            case IDLE:
                transferMotor.setPower(0);
                break;
            case INTAKING:
                transferMotor.setPower(INTAKING_POWER);
                break;
            case OUTTAKING:
                transferMotor.setPower(OUTTAKING_POWER);
                break;
            case STORING:
                transferMotor.setPower(STORING_POWER);
                break;
            case FEEDING:
                transferMotor.setPower(FEEDING_POWER);
                break;
        }
    }

}
