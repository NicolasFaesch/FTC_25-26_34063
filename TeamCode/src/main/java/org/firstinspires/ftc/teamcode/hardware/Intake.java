package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.util.Timing;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

@Configurable
public class Intake {

    public static double INTAKING_POWER = 1.0;
    public static double OUTTAKING_POWER = -0.4;
    public static double STORING_POWER = 0.6;
    public static double DISENGAGING_POWER = 0.4;
    public static double FEEDING_POWER = 0.6;
    public static double FEEDING_POWER_FAR = 0.5;

    public static long REVERSING_OUTTAKING_TIME_MS = 50;

    public enum State {
        IDLE,
        INTAKING,
        OUTTAKING,
        STORING,
        FEEDING
    }

    private State state;

    private DcMotorEx intakeMotor;
    private Timing.Timer reversingOuttakingTimer = new Timing.Timer(REVERSING_OUTTAKING_TIME_MS, TimeUnit.MILLISECONDS);



    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intake");

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);

        setState(State.IDLE);
        reversingOuttakingTimer.start(); // initially start to make sure it's completed
    }
    public void setState(State state) {
        if(state != this.state) {
            this.state = state;
        }
    }

    public State getState() {
        return state;
    }

    public void update(boolean isFarSide, Transfer.FeedingState feedingState) {
        switch(state) {
            case IDLE:
                intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                intakeMotor.setPower(0);
                break;
            case INTAKING:
                if(reversingOuttakingTimer.done()) {
                    intakeMotor.setPower(INTAKING_POWER);
                } else {
                    intakeMotor.setPower(OUTTAKING_POWER);
                }
                break;
            case OUTTAKING:
                intakeMotor.setPower(OUTTAKING_POWER);
                break;
            case STORING:
                intakeMotor.setPower(STORING_POWER);
                break;
            case FEEDING:
                switch (feedingState) {
                    case IDLE:
                        intakeMotor.setPower(STORING_POWER);
                        break;
                    case WAITING:
                        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        intakeMotor.setPower(0);
                        break;
                    case FEEDING:
                        intakeMotor.setPower(isFarSide ? FEEDING_POWER_FAR:FEEDING_POWER);
                        break;
                    case RETRACTING:
                        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        intakeMotor.setPower(0);
                        break;
                }
        }
    }

    public void reverseOuttake() {
        reversingOuttakingTimer.start();
    }

}
