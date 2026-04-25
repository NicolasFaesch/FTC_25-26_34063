package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.ShooterLUT;

import java.util.concurrent.TimeUnit;

@Configurable
public class Shooter {

    // Hood Positions (for manual mode)
    public static double HOOD_MIN_POSITION = 0.175;
    public static double HOOD_MAX_POSITION = 0.8;
    public static double HOOD_STEP_SIZE = 0.025;

    // Blocker Positions & time
    public static double BLOCKER_ENGAGED_POSITION = 0.8;
    public static double BLOCKER_DISENGAGED_POSITION = 0.475;
    public static long BLOCKER_TIME_MS = 120;

    // Shooter Velocity Params (in RPM)
    public static double SHOOTER_MIN_VELOCITY = 2500; // for manual override
    public static double SHOOTER_MAX_VELOCITY = 6000; // for manual override
    public static double SHOOTER_STEP_SIZE = 100; // for manual override
    public static double SHOOTER_VELOCITY_THRESHOLD = 250; // threshold to decide if fast enough to shoot
    public static double SHOOTER_IDLE_VELOCITY = 1500; // Idling speed

    // Shooter Velocity PIDF Coefficients
    public static double SHOOTER_KP = 150.0;
    public static double SHOOTER_KI = 0.0;
    public static double SHOOTER_KD = 60.0;
    public static double SHOOTER_KV = 12.0;

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

    public enum ShooterMotorIdlingState {
        OFF,
        SPINNING
    }

    public enum BlockerState {
        ENGAGED,
        ENGAGING,
        DISENGAGED,
        DISENGAGING
    }

    private State state;
    private ShooterMotorState shooterMotorState;
    private ShooterMotorIdlingState shooterMotorIdlingState;
    private BlockerState blockerState;

    private boolean readyToShoot;

    private DcMotorEx shooterBottom;
    private DcMotorEx shooterTop;

    private Servo hood;
    private Servo blocker;

    private double hoodPositionManual = (HOOD_MAX_POSITION+HOOD_MIN_POSITION)/2; // manual set hood position
    private double shooterTargetVelocityManual = (SHOOTER_MAX_VELOCITY + SHOOTER_MIN_VELOCITY)/2; // manual override velocity

    private double hoodPosition = (HOOD_MIN_POSITION + HOOD_MAX_POSITION)/2; // hood position as calculated by LUT
    private double shooterTargetVelocity; // shooter velocity as calculated by LUT
    private double shooterVelocity; // actual velocity of the shooter

    private boolean manualOverride = false;


    private Timer blockerTimer = new Timer(BLOCKER_TIME_MS, TimeUnit.MILLISECONDS);



    public Shooter(HardwareMap hardwareMap) {
        shooterTop = hardwareMap.get(DcMotorEx.class, "shooter top");
        shooterBottom = hardwareMap.get(DcMotorEx.class, "shooter bottom");

        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");

        shooterTop.setDirection(DcMotorEx.Direction.REVERSE);
        shooterBottom.setDirection(DcMotorEx.Direction.REVERSE);

        shooterTop.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterBottom.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooterTop.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterBottom.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        hood.setPosition(hoodPosition);
        blocker.setPosition(BLOCKER_ENGAGED_POSITION);

        setShooterPIDFGains(SHOOTER_KP, SHOOTER_KI, SHOOTER_KD, SHOOTER_KV);

        setShooterMotorIdlingMode(ShooterMotorIdlingState.OFF);
        setState(State.IDLE);
        blockerState = BlockerState.ENGAGED;
    }


    private void changeState() {
        switch(state) {
            case IDLE:
                if(blockerState == BlockerState.DISENGAGED || blockerState == BlockerState.DISENGAGING) {
                    blocker.setPosition(BLOCKER_ENGAGED_POSITION);
                    blockerTimer.start();
                }
                if(shooterMotorIdlingState == ShooterMotorIdlingState.OFF) {
                    shooterTop.setPower(0);
                    shooterBottom.setPower(0);
                    shooterMotorState = ShooterMotorState.OFF;
                } else {
                    shooterTop.setVelocity(toTPS(SHOOTER_IDLE_VELOCITY));
                    shooterBottom.setVelocity(toTPS(SHOOTER_IDLE_VELOCITY));
                    shooterMotorState = ShooterMotorState.IDLE;
                }
                break;
            case AIMING:
            case SHOOTING:
                shooterMotorState = ShooterMotorState.RAMPING;
                break;
        }
    }

    public void update(double hoodPos, double flywheelVelocity, boolean validShootingPose, boolean turretReady) {
        shooterTargetVelocity = flywheelVelocity;
        shooterVelocity = toRPM((shooterTop.getVelocity() + shooterBottom.getVelocity())/2);
        if(manualOverride) {
            hood.setPosition(hoodPositionManual);
            shooterTargetVelocity = shooterTargetVelocityManual;
        } else {
            hood.setPosition(hoodPos);
        }

        if(state == State.AIMING || state == State.SHOOTING) {
            setShooterTargetVelocity(shooterTargetVelocity);
            boolean shooterToSpeed = Math.abs(shooterVelocity-shooterTargetVelocity) <= SHOOTER_VELOCITY_THRESHOLD;
            if(shooterToSpeed) {
                shooterMotorState = ShooterMotorState.UP_TO_SPEED;
            } else {
                shooterMotorState = ShooterMotorState.RAMPING;
            }
        }

        readyToShoot = validShootingPose && shooterMotorState == ShooterMotorState.UP_TO_SPEED && turretReady;

        // update blocker state
        if(blockerState == BlockerState.DISENGAGING && blockerTimer.done()) {
            blockerState = BlockerState.DISENGAGED;
        }

        if (blockerState == BlockerState.ENGAGING && blockerTimer.done()) {
            blockerState = BlockerState.ENGAGED;
        }

        // engage or disengage blocker if good to shoot
        if(state == State.SHOOTING && readyToShoot) {
            if(blockerState == BlockerState.ENGAGED || blockerState == BlockerState.ENGAGING) {
                blocker.setPosition(BLOCKER_DISENGAGED_POSITION);
                blockerState = BlockerState.DISENGAGING;
                blockerTimer.start();
            }
        } else {
            if(blockerState == BlockerState.DISENGAGED || blockerState == BlockerState.DISENGAGING) {
                blocker.setPosition(BLOCKER_ENGAGED_POSITION);
                blockerState = BlockerState.ENGAGING;
                blockerTimer.start();
            }
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

    public void setShooterMotorIdlingMode(ShooterMotorIdlingState shooterMotorIdlingState) {
        this.shooterMotorIdlingState = shooterMotorIdlingState;
    }

    public ShooterMotorIdlingState getShooterMotorIdlingState() {
        return shooterMotorIdlingState;
    }

    public BlockerState getBlockerState() {return blockerState;}

    public boolean getReadyToShoot() {return readyToShoot;}

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

    public void setShooterTargetVelocity(double shooterTargetVelocity) {
        shooterTop.setVelocity(toTPS(shooterTargetVelocity));
        shooterBottom.setVelocity(toTPS(shooterTargetVelocity));
    }

    public void setShooterPIDFGains(double kP, double kI, double kD, double kV) {
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kP, kI, kD, kV);
        shooterTop.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooterBottom.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

}
