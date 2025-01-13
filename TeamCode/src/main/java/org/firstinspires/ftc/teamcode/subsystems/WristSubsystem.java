package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class WristSubsystem extends SubsystemBase {
    //Constants
    public static final String WRIST_SERVO_NAME = "wristServo";
    public static final String CLAW_SERVO_NAME = "clawServo";
    public static final double CLAW_OPEN_POSITION = 0.5;
    public static final double CLAW_CLOSED_POSITION = 0.1;

    //Hardware Components
    private final Servo wristServo;
    private final Servo clawServo;

    //Additional Elements
    public enum WristPosition {
        COLLAPSED(0.84),
        READY(0.15),
        INTAKE(0.04),
        OUTTAKE(0.67);

        public final double position;
        WristPosition(double position) {
            this.position = position;
        }
    }

    public WristSubsystem(OpMode opMode) {
        wristServo = opMode.hardwareMap.get(Servo.class, WRIST_SERVO_NAME);
        clawServo = opMode.hardwareMap.get(Servo.class, CLAW_SERVO_NAME);
    }

    public void setWristPosition(double position) {
        wristServo.setPosition(Range.clip(position, 0.0, 1.0));
    }

    public void setClawPosition(double position) {
        clawServo.setPosition(Range.clip(position, 0.0, 1.0));
    }

    public void setWristPosition(WristPosition position) {
        wristServo.setPosition(position.position);
    }

    public void closeClaw() {
        clawServo.setPosition(CLAW_CLOSED_POSITION);
    }

    public void openClaw() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }

    public double getWristPosition() {
        return wristServo.getPosition();
    }

    public double getClawPosition() {
        return clawServo.getPosition();
    }
}