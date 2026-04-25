package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class Turret {

    // Turret Angles
    public static double TURRET_MIN_ANGLE = -80;  // in deg
    public static double TURRET_MAX_ANGLE = 110; // in deg
    public static double TURRET_ANGLE_STEP_SIZE = 2.0;  // for manual adjustment in deg
    public static double TURRET_STORED_ANGLE = 0; // in deg

    /*
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

    // Servo power limits
    public static double SERVO_TRACKING_MAX_SPEED = 1.0;
    public static double SERVO_STORED_MAX_SPEED = 0.5;

    private ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
     */
    private static final double SERVO_RANGE_DEGREES = 350;
    public static double LEFT_SERVO_ZERO_DEG_POSITION = 0.5;
    public static double RIGHT_SERVO_ZERO_DEG_POSITION = 0.5;

    public static double ON_TARGET_THRESHOLD = 5; // max angle deviation for target check to be true

    private static final double GEAR_RATIO_SERVO = 48.0/20.0;  // ratio between servos and intermediary gear
    private static final double GEAR_RATIO_TURRET= 60.0/144.0;  // ratio between intermediary gear and turret

    // State of the shooter system for external setting
    public enum State {
        IDLE,
        STORED,
        TRACKING
    }

    private State state;

    private double targetAngleManual = TURRET_STORED_ANGLE; // manual override angle

    private boolean manualOverride = false;

    private double turretAngle;
    private double targetAngle;



    private ServoImplEx servoLeft;
    private ServoImplEx servoRight;

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
        servoLeft = hardwareMap.get(ServoImplEx.class, "turret left");
        servoRight = hardwareMap.get(ServoImplEx.class, "turret right");

        axonEncoder = new AxonEncoder(hardwareMap, "axon encoder");
        // TODO: implement encoder
        magneticEncoder = hardwareMap.get(AS5600Driver.class, "magnetic encoder");

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
                if (!servoRight.isPwmEnabled())
                    servoRight.setPwmEnable();
                break;
        }
    }

    public void update(double targetAngle) {
        this.targetAngle = targetAngle;
        // TODO: add magnetic encoder
        turretAngle = getFusedAngle(magneticEncoder.getAngle(), axonEncoder.getAngle());

        if(state == State.IDLE) { // do nothing if idle
            // servoLeft.setPower(0);
            // servoRight.setPower(0);
            return;
        }

        if(manualOverride)
            this.targetAngle = targetAngleManual;

        if (state==State.STORED)
            this.targetAngle = TURRET_STORED_ANGLE;

        // clip target angle inside valid range
        this.targetAngle = Math.max(TURRET_MIN_ANGLE, Math.min(TURRET_MAX_ANGLE, this.targetAngle));

        // 1. Calculate the required servo angle to reach the turret target.
        // Dividing by the total gear ratio moves us from the turret's frame back to the servo's frame.
        double totalGearRatio = GEAR_RATIO_SERVO * GEAR_RATIO_TURRET;
        double servoTargetAngle = this.targetAngle / totalGearRatio;

        // 2. Convert the degrees into the 0.0 - 1.0 position scale.
        // A 1-degree turn is equal to (1 / 350) in positional units.
        double positionDelta = servoTargetAngle / SERVO_RANGE_DEGREES;

        // 3. Apply the delta to your calibrated zero positions.
        double leftPosition = LEFT_SERVO_ZERO_DEG_POSITION + positionDelta;

        double rightPosition = RIGHT_SERVO_ZERO_DEG_POSITION + positionDelta;

        // 4. Safely clamp the positions between 0.0 and 1.0.
        // (Sending a position outside this range will crash the REV Control Hub)
        leftPosition = Math.max(0.0, Math.min(1.0, leftPosition));
        rightPosition = Math.max(0.0, Math.min(1.0, rightPosition));

        // 5. Command the servos to move!
        servoLeft.setPosition(leftPosition);
        servoRight.setPosition(rightPosition);
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
        // TODO: add correct check
        return (targetAngle < TURRET_MAX_ANGLE && targetAngle > TURRET_MIN_ANGLE);
                //Math.abs(turretAngle - targetAngle) < ON_TARGET_THRESHOLD;
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
