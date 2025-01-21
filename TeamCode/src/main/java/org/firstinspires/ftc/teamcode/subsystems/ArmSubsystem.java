package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
    public static final String ELBOW_MOTOR_NAME = "sampElbow";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "sampSlide";
    //public static final String SLIDE_LIMIT_SWITCH_NAME = "";
    public static final DcMotor.Direction ELBOW_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotorEx.Direction LINEAR_SLIDE_DIRECTION = DcMotor.Direction.REVERSE;
    public static final double ELBOW_P = 0.013;
    public static final double ELBOW_I = 0;
    public static final double ELBOW_D = 0.0001;
    public static final double SLIDE_P = 0.0065;
    public static final double SLIDE_I = 0;
    public static final double SLIDE_D = 0.00015;
    public static final int SLIDE_MAX_POSITION = 2300;
    public static final int ELBOW_MAX_POSITION = 690;
    public static final int SLIDE_MIN_POSITION = 0;
    public static final int ELBOW_MIN_POSITION = 0;
    public static final int TICKS_PER_ELBOW_ROTATION = 1440; //get a real value!
    public static final double POWER_TO_HOLD_ARM = 0.1; //get real value
    public static final double POWER_TO_HOLD_SLIDE = 0.1; //get real value

    //Hardware Components
    private final DcMotorEx elbowMotor;
    private final DcMotorEx linearSlideMotor;
    //private final DigitalChannel slideLimitSwitch;

    //Additional Elements
    private final PIDController elbowController;
    private final PIDController linearSlideController;
    private int targetElbowPosition;
    private int targetLinearSlidePosition;
    private double maxLinearPower;
    private double maxElbowPower;
    private final Telemetry telemetry;
    private final ElapsedTime elbowTimer;
    private final ElapsedTime linearSlideTimer;
    private boolean updateFirstCall; //Used to reset the timers on the first call of periodic

    private int previousElbowTarget = 0;
    private int previousSlideTarget = 0;
    private int previousElbowPosition = 0;
    private int previousSlidePosition = 0;

    private final int tolerance = 3;

    public enum ArmPosition {
        INTAKE_POSITION(0, 0),
        HIGH_OUTTAKE_POSITION(690, 2300),
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
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //slideLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        setTargetArmPosition(ELBOW_MIN_POSITION, SLIDE_MIN_POSITION);
        setElbowMaxPower(1.0);
        setLinearMaxPower(1.0);

        elbowController = new PIDController(ELBOW_P, ELBOW_I, ELBOW_D);
        linearSlideController = new PIDController(SLIDE_P, SLIDE_I, SLIDE_D);

        updateFirstCall = true;

        elbowTimer = new ElapsedTime();
        linearSlideTimer = new ElapsedTime();
        elbowTimer.reset();
        linearSlideTimer.reset();
    }

    public boolean getSlideAtTarget() {
        return Math.abs(linearSlideMotor.getCurrentPosition() - targetLinearSlidePosition) <= tolerance;
    }

    public void setElbowMaxPower(double power) {
        maxElbowPower = Range.clip(power, -1.0, 1.0);
    }

    public void setLinearMaxPower(double power) {
        maxLinearPower = Range.clip(power, -1.0, 1.0);
    }

    public boolean getElbowAtTarget() {
        return Math.abs(elbowMotor.getCurrentPosition() - targetElbowPosition) <= tolerance;
    }

    public void setTargetElbowPosition(int target) {
        targetElbowPosition = Range.clip(target, ELBOW_MIN_POSITION, ELBOW_MAX_POSITION);
    }

    public double getElbowDegrees() {
        return (double) elbowMotor.getCurrentPosition() / (double) TICKS_PER_ELBOW_ROTATION * 360.0;
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

    public void resetEncoders() {
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public SequentialCommandGroup getMoveArmToPositionCommand(ArmPosition position, double maxPower) {
        double previousElbowMaxPower = maxElbowPower;
        double previousLinearMaxPower = maxLinearPower;

        if(elbowMotor.getCurrentPosition() <= (position.elbowPos + 20) && elbowMotor.getCurrentPosition() >= (position.elbowPos - 20)) {
            return new SequentialCommandGroup(
                new ArmToPositionCommand(this, position, maxPower),
                new RunCommand(() -> {
                    setLinearMaxPower(previousLinearMaxPower);
                    setElbowMaxPower(previousElbowMaxPower);
                })
            );
        } else {
            return new SequentialCommandGroup(
                    new SlideToPositionCommand(this, SLIDE_MIN_POSITION, maxPower),
                    new ElbowToPositionCommand(this, position.elbowPos, maxPower),
                    new SlideToPositionCommand(this, position.slidePos, maxPower),
                    new RunCommand(() -> {
                        setLinearMaxPower(previousLinearMaxPower);
                        setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getMoveArmToPositionCommand(ArmPosition position) {
        if(elbowMotor.getCurrentPosition() <= position.elbowPos + 20 && elbowMotor.getCurrentPosition() >= position.elbowPos - 20) {
            return new SequentialCommandGroup(new RunCommand(() -> setTargetArmPosition(position)));
        } else {
            return new SequentialCommandGroup(
                    new SlideToPositionCommand(this, SLIDE_MIN_POSITION),
                    new ElbowToPositionCommand(this, position.elbowPos),
                    new SlideToPositionCommand(this, position.slidePos));
        }
    }

    public void addToLinearSlideTarget(int input) {
        setTargetLinearSlidePosition(targetLinearSlidePosition + input);
    }

    public void addToLinearSlideTarget(int input, boolean overrideLimits) {
        if(overrideLimits) {
            targetLinearSlidePosition += input;
        } else {
            setTargetLinearSlidePosition(targetLinearSlidePosition + input);
        }
    }

    public void addToElbowTarget(int input) {
        setTargetElbowPosition(targetElbowPosition + input);
    }

    public void addToElbowTarget(int input, boolean overrideLimits) {
        if(overrideLimits) {
            targetElbowPosition += input;
        } else {
            setTargetElbowPosition(targetElbowPosition + input);
        }
    }

//    private boolean isLimitSwitchPressed() {
//        return !slideLimitSwitch.getState(); //true for pressed and false for not pressed
//    }

    private void runElbowPID() {
        double pid = Range.clip(elbowController.calculate(elbowMotor.getCurrentPosition(), targetElbowPosition), -maxElbowPower, maxElbowPower);
        double feedForward = Math.cos(Math.toRadians(getElbowDegrees())) * POWER_TO_HOLD_ARM;

        if(Math.abs(targetElbowPosition - ELBOW_MIN_POSITION) <= tolerance) {
            feedForward = 0;
        }

        //Check if the elbow motor has moved past its previous position by the tolerance
        if(Math.abs(elbowMotor.getCurrentPosition() - previousElbowPosition) > tolerance) {
            elbowTimer.reset();
            previousElbowTarget = targetElbowPosition;
            previousElbowPosition = elbowMotor.getCurrentPosition();
            elbowMotor.setPower(pid + feedForward);
        } else if(elbowTimer.seconds() > 1 && previousElbowTarget == targetElbowPosition) {
            elbowMotor.setPower(feedForward); //Stop the motor if the timer exceeds 1 second and the target hasn't changed
        } else {
            elbowMotor.setPower(pid + feedForward);
        }

//        if(Math.abs(power) > 0.5) {
//            if(elbowTimer.seconds() > 3) elbowMotor.setPower(0);
//            else elbowMotor.setPower(power);
//        } else {
//            elbowTimer.reset();
//            elbowMotor.setPower(power);
//        }
    }

    //Commands

    public static class SlideToPositionCommand extends CommandBase {
        private final ArmSubsystem armSubsystem;
        private final int target;
        private final double maxPower;
        private double previousMaxPower;

        public SlideToPositionCommand(ArmSubsystem armSubsystem, int target, double maxPower) {
            this.armSubsystem = armSubsystem;
            this.target = target;
            this.maxPower = maxPower;
            addRequirements(armSubsystem);
        }

        public SlideToPositionCommand(ArmSubsystem armSubsystem, int target) {
            this.armSubsystem = armSubsystem;
            this.target = target;
            this.maxPower = armSubsystem.maxLinearPower;
            addRequirements(armSubsystem);
        }

        @Override
        public void initialize() {
            previousMaxPower = armSubsystem.maxLinearPower;
            armSubsystem.setLinearMaxPower(maxPower);
            armSubsystem.setTargetLinearSlidePosition(target);
        }

        @Override
        public boolean isFinished() {
            return armSubsystem.getSlideAtTarget();
        }

        @Override
        public void end(boolean interrupted) {
            armSubsystem.setLinearMaxPower(previousMaxPower);
        }
    }

    public static class ElbowToPositionCommand extends CommandBase {
        private final ArmSubsystem armSubsystem;
        private final int target;
        private final double maxPower;
        private double previousMaxPower;

        public ElbowToPositionCommand(ArmSubsystem armSubsystem, int target) {
            this.armSubsystem = armSubsystem;
            this.target = target;
            this.maxPower = armSubsystem.maxElbowPower;
            addRequirements(armSubsystem);
        }

        public ElbowToPositionCommand(ArmSubsystem armSubsystem, int target, double maxPower) {
            this.armSubsystem = armSubsystem;
            this.target = target;
            this.maxPower = maxPower;
            addRequirements(armSubsystem);
        }

        @Override
        public void initialize() {
            previousMaxPower = armSubsystem.maxElbowPower;
            armSubsystem.setElbowMaxPower(maxPower);
            armSubsystem.setTargetElbowPosition(target);
        }

        @Override
        public boolean isFinished() {
            return armSubsystem.getElbowAtTarget();
        }

        @Override
        public void end(boolean interrupted) {
            armSubsystem.setElbowMaxPower(previousMaxPower);
        }
    }

    public static class ArmToPositionCommand extends CommandBase {
        private final ArmSubsystem armSubsystem;
        private final ArmPosition position;
        private final double elbowMaxPower;
        private final double linearMaxPower;
        private double previousElbowMaxPower;
        private double previousLinearMaxPower;

        public ArmToPositionCommand(ArmSubsystem armSubsystem, ArmPosition position) {
            this.armSubsystem = armSubsystem;
            this.position = position;
            this.elbowMaxPower = armSubsystem.maxElbowPower;
            this.linearMaxPower = armSubsystem.maxLinearPower;
            addRequirements(armSubsystem);
        }

        public ArmToPositionCommand(ArmSubsystem armSubsystem, ArmPosition position, double maxPower) {
            this.armSubsystem = armSubsystem;
            this.position = position;
            this.elbowMaxPower = maxPower;
            this.linearMaxPower = maxPower;
            addRequirements(armSubsystem);
        }

        @Override
        public void initialize() {
            previousElbowMaxPower = armSubsystem.maxElbowPower;
            previousLinearMaxPower = armSubsystem.maxLinearPower;
            armSubsystem.setElbowMaxPower(elbowMaxPower);
            armSubsystem.setTargetArmPosition(position);
        }

        @Override
        public boolean isFinished() {
            return armSubsystem.getElbowAtTarget() && armSubsystem.getSlideAtTarget();
        }

        @Override
        public void end(boolean interrupted) {
            armSubsystem.setElbowMaxPower(previousElbowMaxPower);
            armSubsystem.setLinearMaxPower(previousLinearMaxPower);
        }
    }

    private void runLinearSlidePID() {
        double pid = Range.clip(linearSlideController.calculate(linearSlideMotor.getCurrentPosition(), targetLinearSlidePosition), -maxElbowPower, maxLinearPower);
        double feedForward = Math.sin(Math.toRadians(getElbowDegrees())) * POWER_TO_HOLD_SLIDE;

        //Check if the elbow motor has moved beyond the tolerance
        if(Math.abs(linearSlideMotor.getCurrentPosition() - previousSlidePosition) > tolerance) {
            linearSlideTimer.reset();
            previousSlideTarget = targetLinearSlidePosition;
            previousSlidePosition = linearSlideMotor.getCurrentPosition();
            linearSlideMotor.setPower(pid + feedForward);
        } else if(linearSlideTimer.seconds() > 1 && previousSlideTarget == targetLinearSlidePosition) {
            linearSlideMotor.setPower(feedForward); //Stop the motor if the timer exceeds 1 second and the target hasn't changed
        } else {
            linearSlideMotor.setPower(pid + feedForward);
        }

//        //If the power to the motors has been set above 0.5 for longer than five seconds it will shut off
//        if(Math.abs(power) > 0.5) {
//            if(linearSlideTimer.seconds() > 3) linearSlideMotor.setPower(0);
//            else linearSlideMotor.setPower(power);
//        } else {
//            linearSlideTimer.reset();
//            linearSlideMotor.setPower(power);
//        }
    }

    @Override
    public void periodic() {
        if(updateFirstCall) {
            elbowTimer.reset();
            linearSlideTimer.reset();
            updateFirstCall = false;
        }

        telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
        telemetry.addData("Arm Target", targetElbowPosition);
        telemetry.addData("Slide Target", targetLinearSlidePosition);

        runElbowPID();
        runLinearSlidePID();
    }
}