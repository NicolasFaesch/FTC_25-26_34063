package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.ShooterLUT;

import java.util.concurrent.TimeUnit;

public class Shooter {



    // Feeder Timings (in milliseconds)
    private static final long FEEDER_TRAVEL_TIME = 120; // time for the feeder servo to move up or down
    private static final long FEEDER_IDLE_TIME = 150; // time the feeder has to wait for ball to come into position  0.7

    // Feeder Positions
    private static final double FEEDER_RETRACTED = 0.825; // 0.95
    private static final double FEEDER_EXTENDED = 0.6;

    // Hood Positions (for manual mode)
    private static final double HOOD_MIN_POSITION = 0.1;
    private static final double HOOD_MAX_POSITION = 0.975;
    private static final double HOOD_STEP_SIZE = 0.025;

    // Shooter Velocity Params (in RPM)
    private static final double SHOOTER_MIN_VELOCITY = 2500; // for manual override
    private static final double SHOOTER_MAX_VELOCITY = 4500; // for manual override
    private static final double SHOOTER_STEP_SIZE = 100; // for manual override
    private static final double SHOOTER_VELOCITY_THRESHOLD = 200; // threshold to decide if fast enough too shoot
    private static final double SHOOTER_IDLE_VELOCITY = 1500; // Idling speed

    // Shooter Velocity PIDF Coefficients
    private static final double SHOOTER_KP = 0.0;
    private static final double SHOOTER_KI = 0.0;
    private static final double SHOOTER_KD = 0.0;
    private static final double SHOOTER_KV = 0.0;

    // motor parameters (DON'T CHANGE)
    private static final double MOTOR_CPR = 28.0;  // encoder counts per revolution
    private static final double GEAR_RATIO = 1.0;  // input speed / output speed

    // State of the shooter system for external setting
    public enum State {
        IDLE,
        AIMING,
        SHOOTING
    }

    // State of the shooter motors
    private enum ShooterMotorState {
        OFF,
        IDLE,
        UP_TO_SPEED,
        RAMPING
    }

    // Feeder State
    enum FeederState {
        READY,
        EXTENDING,
        RETRACTING
    }

    public enum ShooterMotorIdlingState {
        OFF,
        SPINNING
    }

    private State state;
    private ShooterMotorState shooterMotorState;
    private FeederState feederState;
    private ShooterMotorIdlingState shooterMotorIdlingState;

    private Timing.Timer feederExtendingTimer = new Timing.Timer(FEEDER_TRAVEL_TIME, TimeUnit.MILLISECONDS);
    private Timing.Timer feederRetractingTimer = new Timing.Timer(FEEDER_TRAVEL_TIME + FEEDER_IDLE_TIME, TimeUnit.MILLISECONDS); // includes waiting for feeding of new ball

    private boolean readyToShoot;

    private DcMotorEx shooterRight;
    private DcMotorEx shooterLeft;

    private Servo feeder;
    private Servo hood;

    private double hoodPositionManual = (HOOD_MAX_POSITION+HOOD_MIN_POSITION)/2; // manual set hood position
    private double shooterTargetVelocityManual = (SHOOTER_MAX_VELOCITY + SHOOTER_MIN_VELOCITY)/2; // manual override velocity

    private double hoodPosition = (HOOD_MIN_POSITION + HOOD_MAX_POSITION)/2; // hood position as calculated by LUT
    private double shooterTargetVelocity; // shooter velocity as calculated by LUT
    private double shooterVelocity; // actual velocity of the shooter

    private InterpLUT shooterVelocityLUT = ShooterLUT.createVelocityLUT();
    private InterpLUT hoodLUT = ShooterLUT.createHoodLUT();

    private boolean manualOverride = false;
    private int ballsShot;


    public Shooter(HardwareMap hardwareMap, ShooterMotorIdlingState initialshooterMotorIdlingState) {
        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        hood = hardwareMap.get(Servo.class, "hood");
        feeder = hardwareMap.get(Servo.class, "feeder_servo");


        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);
        shooterRight.setDirection(DcMotorEx.Direction.FORWARD);

        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood.setPosition(hoodPosition);
        feeder.setPosition(FEEDER_RETRACTED);
        feederState = FeederState.READY;

        setShooterMotorIdlingMode(initialshooterMotorIdlingState);
        setState(State.IDLE);

        resetShots();
    }


    private void changeState() {
        switch(state) {
            case IDLE:
                if(shooterMotorIdlingState == ShooterMotorIdlingState.OFF) {
                    shooterLeft.setPower(0);
                    shooterRight.setPower(0);
                    shooterMotorState = ShooterMotorState.OFF;
                } else {
                    shooterLeft.setVelocity(toTPS(SHOOTER_IDLE_VELOCITY));
                    shooterRight.setVelocity(toTPS(SHOOTER_IDLE_VELOCITY));
                    shooterMotorState = ShooterMotorState.IDLE;
                }
                break;
            case AIMING: // DON'T add a break. The goal is to jump to shooting case since identical actions (for now)
            case SHOOTING:
                shooterMotorState = ShooterMotorState.RAMPING;
                break;
        }
    }

    public void update(double distance, boolean validShootingPose) {
        shooterTargetVelocity = shooterVelocityLUT.get(distance);
        hoodPosition = hoodLUT.get(distance);
        shooterVelocity = toRPM((shooterLeft.getVelocity() + shooterRight.getVelocity())/2);

        if(state == State.AIMING || state == State.SHOOTING) {
            boolean shooterToSpeed = false;
            if(manualOverride) {
                setShooterTargetVelocity(shooterTargetVelocityManual);
                hood.setPosition(hoodPositionManual);
                shooterToSpeed = Math.abs(shooterVelocity-shooterTargetVelocityManual) <= SHOOTER_VELOCITY_THRESHOLD;
            } else {
                setShooterTargetVelocity(shooterTargetVelocity);
                hood.setPosition(hoodPosition);
                shooterToSpeed = Math.abs(shooterVelocity-shooterTargetVelocity) <= SHOOTER_VELOCITY_THRESHOLD;
            }

            if(shooterToSpeed) {
                shooterMotorState = ShooterMotorState.UP_TO_SPEED;
            } else {
                shooterMotorState = ShooterMotorState.RAMPING;
            }
        }

        readyToShoot = validShootingPose && shooterMotorState == ShooterMotorState.UP_TO_SPEED && feederState == FeederState.READY;

        switch (feederState) {
            case READY:
                if(readyToShoot && state == State.SHOOTING) {
                    feeder.setPosition(FEEDER_EXTENDED);
                    feederState = FeederState.EXTENDING;
                    feederExtendingTimer.start();
                }
                break;
            case EXTENDING:
                if (feederExtendingTimer.done()) {
                    feeder.setPosition(FEEDER_RETRACTED);
                    feederState = FeederState.RETRACTING;
                    feederRetractingTimer.start();
                }
                break;
            case RETRACTING:
                if (feederRetractingTimer.done()) {
                    feederState = FeederState.READY;
                    ballsShot += 1;
                }
                break;
        }

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

    public FeederState getFeederState() {return feederState;}

    public void setShooterMotorIdlingMode(ShooterMotorIdlingState shooterMotorIdlingState) {
        this.shooterMotorIdlingState = shooterMotorIdlingState;
    }

    public ShooterMotorIdlingState getShooterMotorIdlingState() {
        return shooterMotorIdlingState;
    }

    public void setManualOverride(boolean manualOverride) {
        this.manualOverride = manualOverride;
    }

    public boolean getManualOverride() {
        return manualOverride;
    }

    public void increaseManualHoodPos() {
        hoodPositionManual += HOOD_STEP_SIZE;
        if (hoodPositionManual > HOOD_MAX_POSITION) {
            hoodPositionManual = HOOD_MAX_POSITION;
        }
    }

    public void decreaseManualHoodPos() {
        hoodPositionManual -= HOOD_STEP_SIZE;
        if (hoodPositionManual < HOOD_MIN_POSITION) {
            hoodPositionManual = HOOD_MIN_POSITION;
        }
    }

    public void increaseManualShooterVel() {
        shooterTargetVelocityManual += SHOOTER_STEP_SIZE;
        if(shooterTargetVelocityManual > SHOOTER_MAX_VELOCITY) {
            shooterTargetVelocityManual = SHOOTER_MAX_VELOCITY;
        }
    }

    public void decreaseManualShooterVel() {
        shooterTargetVelocityManual -= SHOOTER_STEP_SIZE;
        if(shooterTargetVelocityManual < SHOOTER_MIN_VELOCITY) {
            shooterTargetVelocityManual = SHOOTER_MIN_VELOCITY;
        }
    }

    public double getHoodPosition() {return  hoodPosition;}

    public double getShooterTargetVelocity() {return shooterTargetVelocity;}

    public double getHoodPositionManual() {return hoodPositionManual;}

    public double getShooterTargetVelocityManual() {return shooterTargetVelocityManual;}

    private double toRPM(double velocityRaw) {
        return velocityRaw / MOTOR_CPR * 60.0 / GEAR_RATIO;
    }

    private double toTPS(double velocityRPM) { // encoder Ticks Per Second. Raw value for motor
        return velocityRPM * MOTOR_CPR / 60.0 * GEAR_RATIO;
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    private void setShooterTargetVelocity(double shooterTargetVelocity) {
        shooterLeft.setVelocity(toTPS(shooterTargetVelocity));
        shooterRight.setVelocity(toTPS(shooterTargetVelocity));
    }

    public void resetShots() {
        ballsShot = 0;
    }

    public int getBallsShot() {
        return  ballsShot;
    }

}
