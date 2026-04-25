package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Intake {

    public static double INTAKING_POWER = 1.0;
    public static double OUTTAKING_POWER = -0.5;
    public static double STORING_POWER = 0.3;
    public static double DISENGAGING_POWER = 0.4;
    public static double FEEDING_POWER = 1.0;

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        STORING,
        FEEDING
    }

    private State state;

    private DcMotorEx intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

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

    public void update(boolean blockerDisengaged) {
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
            case FEEDING:
                intakeMotor.setPower(blockerDisengaged ? FEEDING_POWER : DISENGAGING_POWER);
                break;
        }
    }

}
