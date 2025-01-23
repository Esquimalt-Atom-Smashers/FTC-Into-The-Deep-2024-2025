package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpinningWristSubsystem extends SubsystemBase {
    //Constants
    public static final String WRIST_SERVO_NAME = "sampWrist";
    public static final String INTAKE_SERVO_NAME = "sampClaw";
    public static final double WRIST_INTAKE_POSITION = 1.0;
    public static final double WRIST_STOWED_POSITION = 0.38;
    public static final double WRIST_OUTTAKE_POSITION = 0.68;

    private final Servo intakeServo;
    private final Servo wristServo;

    private final Telemetry telemetry;

    public static enum WristPositions {
        INTAKE(WRIST_INTAKE_POSITION),
        STOWED(WRIST_STOWED_POSITION),
        OUTTAKE(WRIST_OUTTAKE_POSITION);

        public double value;

        private WristPositions(double value) {
            this.value = value;
        }
    }

    public SpinningWristSubsystem(OpMode opMode) {
        this.telemetry = opMode.telemetry;

        intakeServo = opMode.hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        wristServo = opMode.hardwareMap.get(Servo.class, WRIST_SERVO_NAME);
    }

    public void intake() {
        intakeServo.setPosition(1.0);
    }

    public void outtake() {
        intakeServo.setPosition(0.0);
    }

    public void stopIntakeServo() {
        intakeServo.setPosition(0.5);
    }

    public void toPosition(WristPositions position) {
        wristServo.setPosition(position.value);
    }
}