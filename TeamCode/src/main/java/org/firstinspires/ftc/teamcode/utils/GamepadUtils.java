package org.firstinspires.ftc.teamcode.utils;

import java.lang.Math;

public abstract class GamepadUtils {
    public static final double DEADZONE = 0.1;

    public static double deadzone(double input, double deadzone) {
        if(Math.abs(input) >= deadzone) return input;
        return Math.pow(input, 3) / Math.pow(deadzone, 2); //Makes input follow a cubic curve to smooth deadzone
    }
    public static double deadzone(double input) {
        return deadzone(input, DEADZONE);
    }
}