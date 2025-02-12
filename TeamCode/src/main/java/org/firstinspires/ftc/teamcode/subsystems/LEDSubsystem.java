package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LEDSubsystem extends SubsystemBase {
    //DcMotor ledMotor;
    RevBlinkinLedDriver blinkIn;
    OpMode opMode;
    public LEDSubsystem(OpMode opMode) {
        this.opMode = opMode;
        HardwareMap hardwareMap = opMode.hardwareMap;

        //ledMotor = hardwareMap.get(DcMotor.class, "LED");
        blinkIn = hardwareMap.get(RevBlinkinLedDriver.class, "blinkIn");
        //ledMotor.setPower(1);
        blinkIn.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }
}
