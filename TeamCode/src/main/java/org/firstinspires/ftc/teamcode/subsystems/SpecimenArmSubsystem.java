package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecimenArmSubsystem extends SubsystemBase {
    //Constants
    public static final String ELBOW_SERVO_NAME = "specElbow";
    public static final String CLAW_SERVO_NAME = "specClaw";

    //Hardware Components
    private Servo elbowServo;
    private Servo clawServo;

    //Additional Properties
    private boolean firstRun;
    private boolean scoringSpec;
    private boolean specInClaw;

    public enum ClawPosition {
        OPEN(0),
        CLOSED(1);

        public final double position;

        ClawPosition(double position) {
            this.position = position;
        }
    }

    private ClawPosition position = ClawPosition.CLOSED;

    public SpecimenArmSubsystem (OpMode opMode) {
        elbowServo = opMode.hardwareMap.get(Servo.class, "specElbow");
        clawServo = opMode.hardwareMap.get(Servo.class, "specClaw");

        firstRun = true;
        scoringSpec = false;
        specInClaw = false;
    }

    //Physical Operations

    public void goToStartPos() {
        if(firstRun) {
            wallPosition();
            firstRun = false;
            scoringSpec = false;
        }
    }

    public void wallPosition() {elbowServo.setPosition(.13);} //go to wall to grab specimen

    public void liftPosition() {elbowServo.setPosition(0.11);} //pull away from wall (may not be needed)

    public void scoreSpecimen() {elbowServo.setPosition(0.90);} //score on high bar

    public void putDown() {elbowServo.setPosition(0.36);}

    public void touchRung() {elbowServo.setPosition(1);}

    public void openClaw() {
        clawServo.setPosition(ClawPosition.OPEN.position);
    }

    public void closeClaw() {
        clawServo.setPosition(ClawPosition.CLOSED.position);
    }

    public void toggleClaw() {
        switch(position) {
            case OPEN:
                closeClaw();
                position = ClawPosition.CLOSED;
                break;
            case CLOSED:
                openClaw();
                position = ClawPosition.OPEN;
                break;
        }
    }

    //RR Action
    public class ScoreSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            scoreSpecimen();
            return false;
        }
    }
    public Action ScoreSpecimen() {
        return new ScoreSpecimen();
    }

    //Wall Pos
    public class WallPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            wallPosition();
            return false;
        }
    }
    public Action WallPos() {
        return new WallPos();
    }

    //Lift Pos
    public class LiftPos implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            liftPosition();
            return false;
        }
    }
    public Action LiftPos() { return new LiftPos(); }

    //Put down
    public class PutDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            putDown();
            return false;
        }
    }
    public Action PutDown() {
        return new PutDown();
    }

    //Open Claw
    public class OpenClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            openClaw();
            return false;
        }
    }
    public Action OpenClaw() {
        return new OpenClaw();
    }

    //Close claw
    public class CloseClaw implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            closeClaw();
            return false;
        }
    }
    public Action CloseClaw() {
        return new CloseClaw();
    }
}