package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Turret {

    // Turret Angles
    public static double TURRET_MIN_ANGLE = -170;  // in deg
    public static double TURRET_MAX_ANGLE = 170; // in deg
    public static double TURRET_ANGLE_STEP_SIZE = 2.0;  // for manual adjustment in deg
    public static double TURRET_STORED_ANGLE = 0; // in deg

    // The minimum power for the axon servos to be able to move
    public static double SERVO_DEADBAND_POWER = 0.075;
    // Range in which the deadband power is not applied (deg)
    public static double SERVO_DEADBAND_RANGE = 1.5;

    // Turret Servos Position PID Coefficients
    public static double COARSE_KP = 0.0025;
    public static double COARSE_KD = 0.0003;

    public static double FINE_KP = 0.003;
    public static double FINE_KD = 0.00025;

    public static double FINE_PID_THRESHOLD = 10.0; // angle deviation threshold for activation of fine PID

    public static double ON_TARGET_THRESHOLD = 0.5; // max angle deviation for target check to be true

    // Servo power limits
    public static double SERVO_TRACKING_MAX_SPEED = 1.0;
    public static double SERVO_STORED_MAX_SPEED = 0.5;

    private static final double GEAR_RATIO_SERVO = 48.0/20.0;  // ratio between servos and intermediary gear
    private static final double GEAR_RATIO_TURRET= 60.0/144.0;  // ratio between intermediary gear and turret

    // State of the shooter system for external setting
    public enum State {
        IDLE,
        STORED,
        TRACKING
    }

    private State state;

    private double targetAngleManual = (TURRET_MIN_ANGLE+TURRET_MAX_ANGLE)/2; // manual override angle

    private boolean manualOverride = false;

    private double turretAngle;
    private double targetAngle;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private CRServoImplEx servoLeft;
    private CRServoImplEx servoRight;

    private AxonEncoder axonEncoder;
    private AS5600Driver magneticEncoder;

    public double getFusedAngle(double fineAngle, double coarseAngle) {

        // 1. Walk the angle from the servo, through the gears, to the turret.
        double roughTurretAngle = coarseAngle * (GEAR_RATIO_SERVO * GEAR_RATIO_TURRET);

        // 2. The Window Size
        double windowSize = 360.0 * GEAR_RATIO_TURRET;
        int rotationIndex = (int) Math.floor(roughTurretAngle / windowSize);

        // 3. The Precise Angle
        // Scale the fine angle down to the turret frame
        double preciseAngle = (rotationIndex * windowSize) + (fineAngle * GEAR_RATIO_TURRET);

        // 4. Backlash/Desync Protection
        double diff = preciseAngle - roughTurretAngle;

        if (diff > (windowSize / 2.0)) {
            preciseAngle -= windowSize;
        } else if (diff < -(windowSize / 2.0)) {
            preciseAngle += windowSize;
        }

        if(preciseAngle > 180)
            preciseAngle -= 360;
        return preciseAngle;
    }

    public Turret(HardwareMap hardwareMap) {
        servoLeft = hardwareMap.get(CRServoImplEx.class, "turret left");
        servoRight = hardwareMap.get(CRServoImplEx.class, "turret right");

        axonEncoder = new AxonEncoder(hardwareMap, "axon encoder");
        // TODO: implement encoder
        //magneticEncoder = hardwareMap.get(AS5600Driver.class, "magnetic encoder");

        setState(State.IDLE);
        changeState(); // force set state for initial state
    }


    private void changeState() {
        switch(state) {
            case IDLE:
                servoLeft.setPwmDisable();
                servoRight.setPwmDisable();
                break;
            case STORED:
            case TRACKING:
                if(!servoLeft.isPwmEnabled())
                    servoLeft.setPwmEnable();
                if (servoRight.isPwmEnabled())
                    servoRight.setPwmEnable();
                break;
        }
    }

    public void update(double targetAngle) {
        this.targetAngle = targetAngle;
        // TODO: add magnetic encoder
        turretAngle = getFusedAngle((axonEncoder.getAngle()*GEAR_RATIO_SERVO)%360, axonEncoder.getAngle());

        if(state == State.IDLE) { // do nothing if idle
            servoLeft.setPower(0);
            servoRight.setPower(0);
            return;
        }

        if(manualOverride)
            this.targetAngle = targetAngleManual;

        if (state==State.STORED)
            this.targetAngle = TURRET_STORED_ANGLE;

        // clip target angle inside valid range
        this.targetAngle = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, targetAngle));

        double error = targetAngle - turretAngle;
        double dt = timer.seconds();
        timer.reset(); // reset timer for the next loop

        // Prevent divide by zero on first loop
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        lastError = error;

        // Dual PD Logic
        double power = 0;
        if (Math.abs(error) > FINE_PID_THRESHOLD) {
            // Coarse Mode: Get there fast
            power = (error * COARSE_KP) + (derivative * COARSE_KD);
        } else {
            // Fine Mode: Settle smoothly
            power = (error * FINE_KP) + (derivative * FINE_KD);
        }

        if (!isOnTarget()) {
            // Instantly skip the deadzone so the servo is always awake
            double basePower = Math.signum(error) * SERVO_DEADBAND_POWER;

            if(Math.abs(error) > SERVO_DEADBAND_RANGE)
                power += Math.signum(error) * SERVO_DEADBAND_POWER;
        } else {
            // When on target, send 0 to let it relax/free-spin
            power = 0;
        }

        // Speed Clamping
        double maxSpeed = (state == State.STORED) ? SERVO_STORED_MAX_SPEED : SERVO_TRACKING_MAX_SPEED;
        power = Math.max(-maxSpeed, Math.min(maxSpeed, power));

        // Software Endstops (Anti-Wire-Rip)
        // If we are at/past the min angle, AND the PID is trying to drive us further negative... STOP.
        if (turretAngle <= TURRET_MIN_ANGLE && power < 0) {
            power = 0;
        }
        // If we are at/past the max angle, AND the PID is trying to drive us further positive... STOP.
        else if (turretAngle >= TURRET_MAX_ANGLE && power > 0) {
            power = 0;
        }

        servoLeft.setPower(power);
        servoRight.setPower(power);
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

    public boolean isOnTarget() {
        return Math.abs(turretAngle - targetAngle) < ON_TARGET_THRESHOLD;
    }


    public void setManualOverride(boolean manualOverride) {
        this.manualOverride = manualOverride;
    }

    public boolean getManualOverride() {
        return manualOverride;
    }


    public void increaseManualTurretAngle() {
        targetAngleManual += TURRET_ANGLE_STEP_SIZE;
        if(targetAngleManual > TURRET_MAX_ANGLE) {
            targetAngleManual = TURRET_MAX_ANGLE;
        }
    }

    public void decreaseManualTurretAngle() {
        targetAngleManual -= TURRET_ANGLE_STEP_SIZE;
        if(targetAngleManual < TURRET_MIN_ANGLE) {
            targetAngleManual = TURRET_MIN_ANGLE;
        }
    }

    public double getTargetAngleManual() {return targetAngleManual;}

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setMagneticEncoderZero() {
        magneticEncoder.setZero();
    }

    public void setAxonEncoderZero() {
        axonEncoder.setZero();
    }

    public double getAxonRawDegrees() {
        return axonEncoder.getRawDegrees();
    }

    public double getAxonVoltage() {
        return axonEncoder.getVoltage();
    }


}
