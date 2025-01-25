package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecimenArmSubsystem extends SubsystemBase {
    public static final String ELBOW_SERVO_NAME = "specElbow";
    public static final String CLAW_SERVO_NAME = "specClaw";

    private Servo elbowServo;
    private Servo clawServo;
    private boolean firstRun;
    private boolean scoringSpec;
    private boolean specInClaw;

    public SpecimenArmSubsystem (OpMode opMode) {
        elbowServo = opMode.hardwareMap.get(Servo.class, "specElbow");
        clawServo = opMode.hardwareMap.get(Servo.class, "specClaw");

        firstRun = true;
        scoringSpec = false;
        specInClaw = false;
    }

    public void goToStartPos() {
        if(firstRun) {
            wallPosition();
            firstRun = false;
            scoringSpec = false;
        }
    }

    public void wallPosition() {elbowServo.setPosition(.11);} //go to wall to grab specimen

    public void liftPosition() {elbowServo.setPosition(0.07);} //pull away from wall (may not be needed)

    public void scoreSpecimen() {elbowServo.setPosition(0.86);} //score on high bar

    public void readyScore() {elbowServo.setPosition(0.7);} //not used

    public void touchRung() {elbowServo.setPosition(1);}

    public void changeScoreSpecimenState() {
        if(elbowServo.getPosition() == 0.15 || elbowServo.getPosition() == 0) {
            if(scoringSpec) readyScore();
            else scoreSpecimen();
        } else {
            if(scoringSpec) {
                readyScore();
                scoringSpec = false;
            } else {
                scoreSpecimen();
                scoringSpec = true;
            }
        }
    }

    public void changeGetSpecimenState() {
        if(elbowServo.getPosition() == 0.95 || elbowServo.getPosition() == 0.7) {
            if(scoringSpec) liftPosition();
            else wallPosition();
        } else {
            if(specInClaw) {
                liftPosition();
                specInClaw = false;
            } else {
                wallPosition();
                specInClaw = true;
            }
        }
    }

    public void openClaw() {
        clawServo.setPosition(1);
    }

    public void closeClaw() {
        clawServo.setPosition(0);
    }
}