package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.util.Timing.Timer;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

@Configurable
public class Transfer {

    public static double INTAKING_POWER = 0.3;
    public static double OUTTAKING_POWER = -0.5;
    public static double STORING_POWER = 0.1;
    public static double DISENGAGING_VELOCITY_NORMED = -0.5;
    public static double FEEDING_VELOCITY_NORMED = 1.0;
    public static double FEEDING_VELOCITY_NORMED_FAR = 1.0;

    // for far shot transfer feeds first, retracts a bit and then waits before next shot
    public static long FEEDING_TIME_FAR_MS = 100;
    public static long RETRACTING_TIME_FAR_MS = 50;
    public static long WAITING_TIME_FAR_MS = 150;

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

    public enum FeedingState {
        FEEDING,
        RETRACTING,
        WAITING,
        IDLE
    }

    private Timer feeding_far_timer = new Timer(FEEDING_TIME_FAR_MS, TimeUnit.MILLISECONDS);
    private Timer retracting_far_timer = new Timer(RETRACTING_TIME_FAR_MS, TimeUnit.MILLISECONDS);
    private Timer waiting_far_timer = new Timer(WAITING_TIME_FAR_MS, TimeUnit.MILLISECONDS);

    private State state;
    private FeedingState feedingState;

    private DcMotorEx transferMotor;

    public Transfer(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class,"transfer");

        transferMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        transferMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setDirection(DcMotorEx.Direction.FORWARD);

        setState(State.IDLE);
        feedingState = FeedingState.IDLE;
    }
    public void setState(State state) {
        if(state != this.state) {
            this.state = state;
        }
    }

    public State getState() {
        return state;
    }

    public void update(boolean blockerClosing, boolean readyToShoot, boolean isFarSide) {
        if(state == State.FEEDING) {
            if(transferMotor.getMode() != DcMotor.RunMode.RUN_USING_ENCODER)
                transferMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            feedingState = FeedingState.IDLE;
        }
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
                if (blockerClosing) {
                    transferMotor.setVelocity(normalizedToTPS(DISENGAGING_VELOCITY_NORMED));
                    feedingState = FeedingState.RETRACTING;
                } else if (readyToShoot){
                    if(isFarSide) {
                        switch (feedingState){
                            case WAITING:
                                if(!waiting_far_timer.done())
                                    break;
                            case IDLE:
                                feedingState = FeedingState.FEEDING;
                                transferMotor.setVelocity(normalizedToTPS(FEEDING_VELOCITY_NORMED_FAR));
                                feeding_far_timer.start();
                                break;
                            case FEEDING:
                                if (feeding_far_timer.done()) {
                                    feedingState = FeedingState.RETRACTING;
                                    transferMotor.setVelocity(normalizedToTPS(DISENGAGING_VELOCITY_NORMED));
                                    retracting_far_timer.start();
                                }
                                break;
                            case RETRACTING:
                                if(retracting_far_timer.done()) {
                                    feedingState = FeedingState.WAITING;
                                    transferMotor.setVelocity(0);
                                    transferMotor.setPower(0);
                                    waiting_far_timer.start();
                                }
                                break;
                        }
                    } else { // close shots, full feed
                        transferMotor.setVelocity(normalizedToTPS(FEEDING_VELOCITY_NORMED));
                        feedingState = FeedingState.FEEDING;
                    }
                } else {
                    if(transferMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER)
                        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    transferMotor.setPower(STORING_POWER);
                    feedingState = FeedingState.IDLE;
                }
                break;
        }
    }

    private double normalizedToTPS(double normalizedVelocity) { // encoder Ticks Per Second. Raw value for motor
        return normalizedVelocity * MOTOR_MAX_VELOCITY * MOTOR_CPR / 60.0;
    }

    public FeedingState getFeedingState() { return feedingState;}

}
