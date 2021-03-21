package org.firstinspires.ftc.teamcode.Common;

public class UtilMethods {
    // sleep method from LinearOpMode
    public static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public static double ensureRange(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    public static int ensureRange(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }


    public static boolean inRange(double value, double min, double max) {
        return (value>= min) && (value<= max);
    }

    public static boolean atTarget(double target, double value,  double plusMinus) {
        return (value>= target-plusMinus) && (value<= target+plusMinus);
    }
}
