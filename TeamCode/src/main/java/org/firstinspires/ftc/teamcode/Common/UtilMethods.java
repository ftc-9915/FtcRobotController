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
}
