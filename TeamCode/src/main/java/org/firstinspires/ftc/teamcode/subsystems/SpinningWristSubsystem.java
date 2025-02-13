package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    private final ArmSubsystem armSubsystem;
    private boolean firstTime = true; //used in periodic

    public enum WristPosition {
        INTAKE(WRIST_INTAKE_POSITION),
        STOWED(WRIST_STOWED_POSITION),
        OUTTAKE(WRIST_OUTTAKE_POSITION);

        public double value;

        WristPosition(double value) {
            this.value = value;
        }
    }

    private WristPosition currentWristPosition;
    private WristPosition targetWristPosition;

    public SpinningWristSubsystem(OpMode opMode, ArmSubsystem armSubsystem, WristPosition startingPosition) {
        this.telemetry = opMode.telemetry;
        this.armSubsystem = armSubsystem;

        targetWristPosition = startingPosition;
        currentWristPosition = startingPosition;

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
        targetWristPosition = position;
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

    @Override
    public void periodic() {
        if((armSubsystem.getSlidePosition() - ArmSubsystem.WRIST_OUT_MAX_SLIDE_POSITION) <= ArmSubsystem.TOLERANCE || armSubsystem.getTargetElbowPosition() > 300) {
            wristServo.setPosition(targetWristPosition.value);
            currentWristPosition = targetWristPosition;
            if(targetWristPosition != WristPosition.OUTTAKE) armSubsystem.removeWristOutMaxSlidePosition();
            firstTime = true;
        } else if(currentWristPosition == WristPosition.OUTTAKE || targetWristPosition != currentWristPosition) {
            armSubsystem.addWristOutMaxSlidePosition();
            if(firstTime) {
                armSubsystem.setTargetLinearSlidePosition(armSubsystem.getTargetLinearSlidePosition());
                firstTime = false;
            }
        }
    }

    //RR actions
    public class Intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intake();
            return false;
        }
    }
    public Action Intake() {
        return new Intake();
    }

    public class Outtake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            outtake();
            return false;
        }
    }
    public Action Outtake() {return new Outtake();}
}