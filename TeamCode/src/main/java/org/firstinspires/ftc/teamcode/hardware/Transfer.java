package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Transfer {

    public static double INTAKING_POWER = 0.5;
    public static double OUTTAKING_POWER = -0.5;
    public static double STORING_POWER = 0.0;
    public static double DISENGAGING_POWER = -0.3;
    public static double FEEDING_POWER = 0.8;

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
        transferMotor.setDirection(DcMotorEx.Direction.FORWARD);

        setState(State.IDLE);
    }
    public void setState(State state) {
        if(state != this.state) {
            this.state = state;
        }
    }

    public State getState() {
        return state;
    }

    public void update(boolean blockerChanging) {
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
                transferMotor.setPower(blockerChanging ? DISENGAGING_POWER : FEEDING_POWER);
                break;
        }
    }

}
