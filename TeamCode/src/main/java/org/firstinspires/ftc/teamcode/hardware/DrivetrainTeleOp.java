package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.lib.Constants;
import org.firstinspires.ftc.teamcode.lib.PoseStorage;

@Configurable
public class DrivetrainTeleOp extends Drivetrain{
    public static final double FORWARD_SCALING = 1.0;
    public static final double STRAFE_SCALING = 1.0;
    public static final double TURN_SCALING = 0.7;

    private boolean parkingMode;

    public DrivetrainTeleOp(HardwareMap hardwareMap) {
        super(hardwareMap, PoseStorage.currentPose);
        follower.startTeleOpDrive(false);
        parkingMode = false;
    }


    public void update(double leftStickX, double leftStickY, double rightStickX, double speedScalar) {
        if(parkingMode) {
            // TODO: implement whole drive automation and rename parking mode
            super.update();
        } else {
            //Normal case:
            double forward = -leftStickY * FORWARD_SCALING * speedScalar;
            double strafe = -leftStickX * STRAFE_SCALING * speedScalar;
            double turn = -rightStickX * TURN_SCALING * speedScalar;

            follower.setTeleOpDrive(forward, strafe, turn);
        }
        super.update();
    }

    public void park() {
        parkingMode = true;
    }

    public void noParking() {
        parkingMode = false;
    }

}
