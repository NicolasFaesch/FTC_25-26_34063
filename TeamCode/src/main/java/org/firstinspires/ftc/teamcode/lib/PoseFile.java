package org.firstinspires.ftc.teamcode.lib;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class PoseFile {

    private static final File file =
            new File(Environment.getExternalStorageDirectory(), "FIRST/lastPose.txt");

    public static void writePose(Pose2D pose) {

        try {
            FileWriter writer = new FileWriter(file, false); // overwrite

            writer.write(
                    pose.getX(DistanceUnit.INCH) + "," +
                            pose.getY(DistanceUnit.INCH) + "," +
                            pose.getHeading(AngleUnit.RADIANS)
            );

            writer.close();

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static Pose2D readPose() {

        try {

            if (!file.exists()) return null;

            Scanner scanner = new Scanner(file);

            String line = scanner.nextLine();
            scanner.close();

            String[] data = line.split(",");

            double x = Double.parseDouble(data[0]);
            double y = Double.parseDouble(data[1]);
            double heading = Double.parseDouble(data[2]);

            return new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading);

        } catch (Exception e) {
            e.printStackTrace();
        }

        return null;
    }
}