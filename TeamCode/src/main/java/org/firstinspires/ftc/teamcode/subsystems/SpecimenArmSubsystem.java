package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SpecimenArmSubsystem extends SubsystemBase {
    //Constants
    public static final String ELBOW_SERVO_NAME = "specElbow";
    public static final String CLAW_SERVO_NAME = "specClaw";
    public static double WALL_POSITION = 0.2;
    public static double LIFT_POSITION = 0.16;
    public static double SCORE_SPECIMEN_POSITION = 0.95;
    public static double PUT_DOWN_POSITION = 0.46;
    public static double TOUCH_RUNG_POSITION = 0.12;

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

    public void wallPosition() {elbowServo.setPosition(WALL_POSITION);} //go to wall to grab specimen

    public void liftPosition() {elbowServo.setPosition(LIFT_POSITION);} //pull away from wall (may not be needed)

    public void scoreSpecimen() {elbowServo.setPosition(SCORE_SPECIMEN_POSITION);} //score on high bar

    public void putDown() {elbowServo.setPosition(PUT_DOWN_POSITION);}

    public void touchRung() {elbowServo.setPosition(TOUCH_RUNG_POSITION);}

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
