package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase {
    //Constants
    public static final String ELBOW_MOTOR_NAME = "elbowMotor";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "slideMotor";
    //public static final String SLIDE_LIMIT_SWITCH_NAME = "";
    public static final DcMotor.Direction ELBOW_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotorEx.Direction LINEAR_SLIDE_DIRECTION = DcMotor.Direction.REVERSE;
    public static final double ELBOW_P = 0.013;
    public static final double ELBOW_I = 0;
    public static final double ELBOW_D = 0.0001;
    public static final double SLIDE_P = 0.0065;
    public static final double SLIDE_I = 0;
    public static final double SLIDE_D = 0.00015;
    public static final int SLIDE_MAX_POSITION = 2340;
    public static final int ELBOW_MAX_POSITION = 690;
    public static final int SLIDE_MIN_POSITION = 0;
    public static final int ELBOW_MIN_POSITION = 0;

    //Hardware Components
    private final DcMotorEx elbowMotor;
    private final DcMotorEx linearSlideMotor;
    //private final DigitalChannel slideLimitSwitch;

    //Additional Elements
    private final PIDController elbowController;
    private final PIDController linearSlideController;
    private int targetElbowPosition;
    private int targetLinearSlidePosition;
    private final Telemetry telemetry;
    private final ElapsedTime elbowTimer;
    private final ElapsedTime linearSlideTimer;

    public enum ArmPosition {
        INTAKE_POSITION(0, 0),
        HIGH_OUTTAKE_POSITION(690, 2340),
        LOW_OUTTAKE_POSITION(690, 980); //Check positions

        public final int elbowPos;
        public final int slidePos;

        ArmPosition(int elbowPos, int slidePos) {
            this.elbowPos = elbowPos;
            this.slidePos = slidePos;
        }
    }

    public ArmSubsystem(OpMode opMode) {
        telemetry = opMode.telemetry;

        elbowMotor = opMode.hardwareMap.get(DcMotorEx.class, ELBOW_MOTOR_NAME);
        linearSlideMotor = opMode.hardwareMap.get(DcMotorEx.class, LINEAR_SLIDE_MOTOR_NAME);
        //slideLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, SLIDE_LIMIT_SWITCH_NAME);

        //Motor Initialization
        elbowMotor.setDirection(ELBOW_DIRECTION);
        linearSlideMotor.setDirection(LINEAR_SLIDE_DIRECTION);

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //slideLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        setTargetArmPosition(ELBOW_MIN_POSITION, SLIDE_MIN_POSITION);

        elbowController = new PIDController(ELBOW_P, ELBOW_I, ELBOW_D);
        linearSlideController = new PIDController(SLIDE_P, SLIDE_I, SLIDE_D);

        elbowTimer = new ElapsedTime();
        linearSlideTimer = new ElapsedTime();
        elbowTimer.reset();
        linearSlideTimer.reset();
    }

    public boolean getSlideAtTarget() {
        return linearSlideMotor.getCurrentPosition() + 2 >= targetLinearSlidePosition && linearSlideMotor.getCurrentPosition() - 2 <= targetLinearSlidePosition; //Arbitrary buffer
    }

    public boolean getElbowAtTarget() {
        return elbowMotor.getCurrentPosition() + 2 >= targetElbowPosition && elbowMotor.getCurrentPosition() - 2 <= targetElbowPosition; //Arbitrary buffer
    }

    public void setTargetElbowPosition(int target) {
        targetElbowPosition = Range.clip(target, ELBOW_MIN_POSITION, ELBOW_MAX_POSITION);
    }

    public void setTargetLinearSlidePosition(int target) {
        //targetLinearSlidePosition = isLimitSwitchPressed() && target <= linearSlideMotor.getCurrentPosition() ? SLIDE_MIN_POSITION : Range.clip(target, SLIDE_MIN_POSITION, SLIDE_MAX_POSITION);
        targetLinearSlidePosition = Range.clip(target, SLIDE_MIN_POSITION, SLIDE_MAX_POSITION);
    }

    public void setTargetArmPosition(int elbowTarget, int linearTarget) {
        setTargetElbowPosition(elbowTarget);
        setTargetLinearSlidePosition(linearTarget);
    }

    public void setTargetArmPosition(ArmPosition armPosition) {
        setTargetElbowPosition(armPosition.elbowPos);
        setTargetLinearSlidePosition(armPosition.slidePos);
    }

    public void addToLinearSlideTarget(int input) {
        targetLinearSlidePosition += input;
    }

    public void addToElbowTarget(int input) {
        targetElbowPosition += input;
    }

//    private boolean isLimitSwitchPressed() {
//        return !slideLimitSwitch.getState(); //true for pressed and false for not pressed
//    }

    private void runElbowPID() {
        double power = elbowController.calculate(elbowMotor.getCurrentPosition(), targetElbowPosition);

        //If the power to the motors has been set above 0.5 for longer than five seconds it will shut off
        if(Math.abs(power) > 0.5) {
            if(elbowTimer.seconds() > 3) elbowMotor.setPower(0);
            else elbowMotor.setPower(power);
        } else {
            elbowTimer.reset();
            elbowMotor.setPower(power);
        }
    }

    private void runLinearSlidePID() {
        double power = linearSlideController.calculate(linearSlideMotor.getCurrentPosition(), targetLinearSlidePosition);

        //If the power to the motors has been set above 0.5 for longer than five seconds it will shut off
        if(Math.abs(power) > 0.5) {
            if(linearSlideTimer.seconds() > 3) linearSlideMotor.setPower(0);
            else linearSlideMotor.setPower(power);
        } else {
            linearSlideTimer.reset();
            linearSlideMotor.setPower(power);
        }
    }

    private boolean firstCall = true;
    @Override
    public void periodic() {
        if(firstCall) {
            elbowTimer.reset();
            linearSlideTimer.reset();
            firstCall = false;
        }

        telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
        telemetry.addData("Arm Target", targetElbowPosition);
        telemetry.addData("Slide Target", targetLinearSlidePosition);

        runElbowPID();
        runLinearSlidePID();
    }
}