package org.firstinspires.ftc.teamcode.TestingTuning;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.util.Scanner;

public class NumberFile {

    static File file = new File(Environment.getExternalStorageDirectory(),
            "testNumber.txt");

    public static void writeNumber(int number) {

        try {
            FileWriter writer = new FileWriter(file, false);
            writer.write(String.valueOf(number));
            writer.close();
        }
        catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static int readNumber() {

        try {

            if(!file.exists())
                return -1;

            Scanner scanner = new Scanner(file);

            int number = scanner.nextInt();

            scanner.close();

            return number;

        }
        catch (Exception e) {
            e.printStackTrace();
        }

        return -1;
    }
}