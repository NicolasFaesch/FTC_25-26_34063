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
    public static double DISENGAGING_POWER = -0.5;
    public static double FEEDING_VELOCITY_NORMED = 1.0;
    public static double FEEDING_VELOCITY_NORMED_FAR = 0.8;

    // motor parameters (DON'T CHANGE)
    private static final double MOTOR_CPR = 28.0;  // encoder counts per revolution
    //private static final double GEAR_RATIO = 1/3.7;  // input gear teeth / output gear teeth
    private static final double MOTOR_MAX_VELOCITY = 6000; // in rpm

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

    public void update(boolean blockerChanging, boolean readyToShoot, boolean isFarSide) {
        switch(state) {
            case IDLE:
                if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setPower(0);
                break;
            case INTAKING:
                if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setPower(INTAKING_POWER);
                break;
            case OUTTAKING:
                if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setPower(OUTTAKING_POWER);
                break;
            case STORING:
                if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                    transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setPower(STORING_POWER);
                break;
            case FEEDING:
                if (blockerChanging) {
                    if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    transferMotor.setPower(DISENGAGING_POWER);
                } else if (readyToShoot){
                    if(transferMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
                        transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    transferMotor.setVelocity(isFarSide ? normalizedToTPS(FEEDING_VELOCITY_NORMED_FAR):normalizedToTPS(FEEDING_VELOCITY_NORMED));
                } else {
                    if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    transferMotor.setPower(STORING_POWER);
                }
                break;
        }
    }

    private double normalizedToTPS(double normalizedVelocity) { // encoder Ticks Per Second. Raw value for motor
        return normalizedVelocity * MOTOR_MAX_VELOCITY * MOTOR_CPR / 60.0;
    }

}
