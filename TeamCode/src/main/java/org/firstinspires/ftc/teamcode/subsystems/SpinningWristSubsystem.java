package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SpinningWristSubsystem extends SubsystemBase {
    //Constants
    public static final String WRIST_SERVO_NAME = "sampWrist";
    public static final String INTAKE_SERVO_NAME = "sampClaw";
    public static final double WRIST_INTAKE_POSITION = 1;
    public static final double WRIST_STOWED_POSITION = 0.2;
    public static final double WRIST_OUTTAKE_POSITION = 0.65;

    //Hardware
    private final Servo intakeServo;
    private final Servo wristServo;

    //Additional Properties
    private final Telemetry telemetry;

    public enum WristPosition {
        INTAKE(WRIST_INTAKE_POSITION),
        STOWED(WRIST_STOWED_POSITION),
        OUTTAKE(WRIST_OUTTAKE_POSITION);

        public double value;

        WristPosition(double value) {
            this.value = value;
        }
    }

    private WristPosition currentWristPosition = WristPosition.STOWED;

    public SpinningWristSubsystem(OpMode opMode) {
        this.telemetry = opMode.telemetry;

        intakeServo = opMode.hardwareMap.get(Servo.class, INTAKE_SERVO_NAME);
        wristServo = opMode.hardwareMap.get(Servo.class, WRIST_SERVO_NAME);
    }

    //Physical Operations

    public void intake() {
        intakeServo.setPosition(1.0);
    }

    public void outtake() {
        intakeServo.setPosition(0.0);
    }

    public void stopIntakeServo() {
        intakeServo.setPosition(0.5);
    }

    public void toPosition(WristPosition position) {
        wristServo.setPosition(position.value);
        currentWristPosition = position;
    }

    //Getters

    public WristPosition getCurrentWristPosition() {
        return currentWristPosition;
    }

    //Commands

    public static class MoveWristToPositionCommand extends CommandBase {
        private final WristPosition position;
        private final SpinningWristSubsystem spinningWristSubsystem;

        public MoveWristToPositionCommand(SpinningWristSubsystem spinningWristSubsystem, WristPosition position) {
            this.position = position;
            this.spinningWristSubsystem = spinningWristSubsystem;

            addRequirements(spinningWristSubsystem);
        }

        @Override
        public void initialize() {
            spinningWristSubsystem.toPosition(position);
        }

        @Override
        public boolean isFinished() {
            return spinningWristSubsystem.getCurrentWristPosition() == position;
        }
    }
}