package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//TODO: Add states

@Config
public class ArmSubsystem extends SubsystemBase {
    //Constants
    public static final String ELBOW_MOTOR_NAME = "sampElbow";
    public static final String LINEAR_SLIDE_MOTOR_NAME = "sampSlide";
    public static final String SLIDE_LIMIT_SWITCH_NAME = "sampSlideLimitSwitch";
    public static final String ELBOW_LIMIT_SWITCH_NAME = "sampElbowMagSwitch";
    public static final DcMotor.Direction ELBOW_DIRECTION = DcMotor.Direction.FORWARD;
    public static final DcMotorEx.Direction LINEAR_SLIDE_DIRECTION = DcMotor.Direction.REVERSE;
    public static double ELBOW_P = 0.013;
    public static double ELBOW_I = 0;
    public static double ELBOW_D = 0.0001;
    public static double SLIDE_P = 0.008;
    public static double SLIDE_I = 0;
    public static double SLIDE_D = 0.00015;
    public static final int SLIDE_MAX_POSITION = 1900; //(high bucket plus a buffer)
    public static final int SLIDE_MAX_POSITION_DOWN = 1700;
    public static final int WRIST_OUT_MAX_SLIDE_POSITION = 1370;

    public static final int ELBOW_MAX_POSITION = 685;
    public static final int SLIDE_MIN_POSITION = 0;
    public static final int ELBOW_MIN_POSITION = 0;
    public static final int TICKS_PER_ELBOW_ROTATION = 2740;
    public static final double POWER_TO_HOLD_ARM = 0.263; //0.263 with light intake
    public static final double POWER_TO_HOLD_SLIDE = 0.09; //previously 0.104
    public static final int TOLERANCE = 25;

    //Hardware Components
    private final DcMotorEx elbowMotor;
    private final DcMotorEx linearSlideMotor;
    private final RevTouchSensor slideLimitSwitch;
    private final DigitalChannel elbowLimitSwitch;

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
    private boolean elbowPIDtimeout = false;
    private boolean slidePIDtimeout = false;
    private boolean usingWristOutMaxSlidePosition = false;

    //Used for Periodic
    private boolean updateFirstCall; //Used to reset the timers on the first call of periodic

    private int previousElbowTarget = 0;
    private int previousSlideTarget = 0;
    private int previousElbowPosition = 0;
    private int previousSlidePosition = 0;

    public enum ArmPosition {
        INTAKE_POSITION(0, 0),
        HIGH_OUTTAKE_POSITION(685, 1867),
        LOW_OUTTAKE_POSITION(685, 500); //Check positions

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
        slideLimitSwitch = opMode.hardwareMap.get(RevTouchSensor.class, SLIDE_LIMIT_SWITCH_NAME);
        elbowLimitSwitch = opMode.hardwareMap.get(DigitalChannel.class, ELBOW_LIMIT_SWITCH_NAME);

        //Motor Initialization
        elbowMotor.setDirection(ELBOW_DIRECTION);
        linearSlideMotor.setDirection(LINEAR_SLIDE_DIRECTION);

        elbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elbowLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

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

    //Physical Operations

    public void resetEncoders() {
        resetElbowEncoder();
        resetSlideEncoder();
    }

    public void resetElbowEncoder() {
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void resetSlideEncoder() {
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean isSlideLimitSwitchPressed() {
        return slideLimitSwitch.getValue() == 1; //true for pressed and false for not pressed
    }

    private boolean isElbowLimitSwitchPressed() {
        return !elbowLimitSwitch.getState(); //true for pressed and false for not pressed
    }

    private void runElbowPID() {
        double pid = Range.clip(elbowController.calculate(elbowMotor.getCurrentPosition(), targetElbowPosition), -maxElbowPower, maxElbowPower);
        double feedForward = Math.cos(Math.toRadians(getElbowDegrees())) * POWER_TO_HOLD_ARM;

        if(Math.abs(elbowMotor.getCurrentPosition() - ELBOW_MIN_POSITION) <= TOLERANCE) {
            feedForward = 0;
        }

        telemetry.addData("Elbow Feed Forward", feedForward);

        //Check if the elbow motor has moved past its previous position by the tolerance
        if(Math.abs(elbowMotor.getCurrentPosition() - previousElbowPosition) > 2) {
            elbowTimer.reset();
            previousElbowTarget = targetElbowPosition;
            previousElbowPosition = elbowMotor.getCurrentPosition();
            elbowMotor.setPower(pid + feedForward);
            elbowPIDtimeout = false;
        } else if(elbowTimer.seconds() > 1 && previousElbowTarget == targetElbowPosition) {
            elbowMotor.setPower(feedForward); //Stop the motor if the timer exceeds 1 second and the target hasn't changed
            elbowPIDtimeout = true;
        } else {
            elbowMotor.setPower(pid + feedForward);
            elbowPIDtimeout = false;
        }
    }

    private void runLinearSlidePID() {
        double pid = Range.clip(linearSlideController.calculate(linearSlideMotor.getCurrentPosition(), targetLinearSlidePosition), -maxLinearPower, maxLinearPower);
        double feedForward = Math.sin(Math.toRadians(getElbowDegrees())) * POWER_TO_HOLD_SLIDE *
                ((double) linearSlideMotor.getCurrentPosition() / (double) (SLIDE_MAX_POSITION - SLIDE_MIN_POSITION)
                        - (double) SLIDE_MIN_POSITION / (double) (SLIDE_MAX_POSITION - SLIDE_MIN_POSITION));

        telemetry.addData("Linear Slide Feed Forward", feedForward);

        //Check if the elbow motor has moved beyond the tolerance
        if(Math.abs(linearSlideMotor.getCurrentPosition() - previousSlidePosition) > 2) {
            linearSlideTimer.reset();
            previousSlideTarget = targetLinearSlidePosition;
            previousSlidePosition = linearSlideMotor.getCurrentPosition();
            linearSlideMotor.setPower(pid + feedForward);
            slidePIDtimeout = false;
        } else if(linearSlideTimer.seconds() > 1 && previousSlideTarget == targetLinearSlidePosition) {
            linearSlideMotor.setPower(feedForward); //Stop the motor if the timer exceeds 1 second and the target hasn't changed
            slidePIDtimeout = true;
        } else {
            linearSlideMotor.setPower(pid + feedForward);
            slidePIDtimeout = false;
        }
    }

    //Setters

    public void setElbowMaxPower(double power) {
        maxElbowPower = Range.clip(power, -1.0, 1.0);
    }

    public void setLinearMaxPower(double power) {
        maxLinearPower = Range.clip(power, -1.0, 1.0);
    }

    public void setTargetLinearSlidePosition(int target) {
        if(targetElbowPosition < 300) {
            targetLinearSlidePosition = Range.clip(target, SLIDE_MIN_POSITION, (usingWristOutMaxSlidePosition) ? WRIST_OUT_MAX_SLIDE_POSITION : SLIDE_MAX_POSITION_DOWN);
        } else {
            targetLinearSlidePosition = Range.clip(target, SLIDE_MIN_POSITION, SLIDE_MAX_POSITION);
        }
    }

    public void setTargetElbowPosition(int target) {
        targetElbowPosition = Range.clip(target, ELBOW_MIN_POSITION, ELBOW_MAX_POSITION);
    }

    protected void addWristOutMaxSlidePosition() {
        usingWristOutMaxSlidePosition = true;
    }

    protected void removeWristOutMaxSlidePosition() {
        usingWristOutMaxSlidePosition = false;
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

    //Getters

    public boolean getSlideAtTarget() {
        return Math.abs(linearSlideMotor.getCurrentPosition() - targetLinearSlidePosition) <= TOLERANCE;
    }

    public boolean getElbowAtTarget() {
        return Math.abs(elbowMotor.getCurrentPosition() - targetElbowPosition) <= TOLERANCE;
    }

    public double getElbowDegrees() {
        return (double) elbowMotor.getCurrentPosition() / (double) TICKS_PER_ELBOW_ROTATION * 360.0;
    }

    public double getMaxElbowPower() {
        return maxElbowPower;
    }

    public double getMaxLinearPower() {
        return maxLinearPower;
    }

    public int getElbowPosition() {
        return elbowMotor.getCurrentPosition();
    }

    public int getSlidePosition() {
        return linearSlideMotor.getCurrentPosition();
    }

    public int getTargetLinearSlidePosition() {
        return targetLinearSlidePosition;
    }

    public int getTargetElbowPosition() {
        return targetElbowPosition;
    }

    //Commands

    public SequentialCommandGroup getMoveArmToPositionCommand(ArmPosition position, double maxLinearPower, double maxElbowPowerGoingUp, double maxElbowPowerGoingDown) {
        double previousElbowMaxPower = this.maxElbowPower;
        double previousLinearMaxPower = this.maxLinearPower;

        if(Math.abs(elbowMotor.getCurrentPosition() - position.elbowPos) <= TOLERANCE) {
            return new SequentialCommandGroup(
                    new ArmToPositionCommand(this, position, maxLinearPower, maxElbowPower),
                    new RunCommand(() -> {
                        setLinearMaxPower(previousLinearMaxPower);
                        setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    new SlideToPositionCommand(this, SLIDE_MIN_POSITION, maxLinearPower),
                    new ElbowToPositionCommand(this, position.elbowPos, (position.elbowPos < elbowMotor.getCurrentPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    new SlideToPositionCommand(this, position.slidePos, maxLinearPower),
                    new RunCommand(() -> {
                        setLinearMaxPower(previousLinearMaxPower);
                        setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getMoveArmToPositionCommand(ArmPosition position) {
        if(Math.abs(elbowMotor.getCurrentPosition() - position.elbowPos) <= TOLERANCE) {
            return new SequentialCommandGroup(new RunCommand(() -> setTargetArmPosition(position)));
        } else {
            return new SequentialCommandGroup(
                    new SlideToPositionCommand(this, SLIDE_MIN_POSITION),
                    new ElbowToPositionCommand(this, position.elbowPos),
                    new SlideToPositionCommand(this, position.slidePos));
        }
    }

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
            armSubsystem.linearSlideTimer.reset();
            armSubsystem.slidePIDtimeout = false;
        }

        @Override
        public boolean isFinished() {
            return armSubsystem.getSlideAtTarget() || armSubsystem.slidePIDtimeout;
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
            armSubsystem.elbowTimer.reset();
            armSubsystem.elbowPIDtimeout = false;
        }

        @Override
        public boolean isFinished() {
            return armSubsystem.getElbowAtTarget() || armSubsystem.elbowPIDtimeout;
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

        public ArmToPositionCommand(ArmSubsystem armSubsystem, ArmPosition position, double maxSlidePower, double maxElbowPower) {
            this.armSubsystem = armSubsystem;
            this.position = position;
            this.elbowMaxPower = maxElbowPower;
            this.linearMaxPower = maxSlidePower;
            addRequirements(armSubsystem);
        }

        @Override
        public void initialize() {
            previousElbowMaxPower = armSubsystem.maxElbowPower;
            previousLinearMaxPower = armSubsystem.maxLinearPower;
            armSubsystem.setElbowMaxPower(elbowMaxPower);
            armSubsystem.setLinearMaxPower(linearMaxPower);
            armSubsystem.setTargetArmPosition(position);
            armSubsystem.linearSlideTimer.reset();
            armSubsystem.elbowTimer.reset();
            armSubsystem.elbowPIDtimeout = false;
            armSubsystem.slidePIDtimeout = false;
        }

        @Override
        public boolean isFinished() {
            return (armSubsystem.getElbowAtTarget() || armSubsystem.elbowPIDtimeout)
                    && (armSubsystem.getSlideAtTarget() || armSubsystem.slidePIDtimeout);
        }

        @Override
        public void end(boolean interrupted) {
            armSubsystem.setElbowMaxPower(previousElbowMaxPower);
            armSubsystem.setLinearMaxPower(previousLinearMaxPower);
        }
    }

    //Periodic

    @Override
    public void periodic() {
        if(updateFirstCall) {
            elbowTimer.reset();
            linearSlideTimer.reset();
            updateFirstCall = false;
        }

        //Reset the linear slide encoders when the limit switch is pressed
        if(isSlideLimitSwitchPressed()) {
            resetSlideEncoder();
        }

        //Reset the elbow encoders when the limit switch is pressed
        if(isElbowLimitSwitchPressed()) {
            resetElbowEncoder();
        }

        telemetry.addData("Elbow Position", elbowMotor.getCurrentPosition());
        telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
        telemetry.addData("Arm Target", targetElbowPosition);
        telemetry.addData("Slide Target", targetLinearSlidePosition);
        telemetry.addData("Elbow Degrees", getElbowDegrees());

        runElbowPID();
        runLinearSlidePID();
    }
}