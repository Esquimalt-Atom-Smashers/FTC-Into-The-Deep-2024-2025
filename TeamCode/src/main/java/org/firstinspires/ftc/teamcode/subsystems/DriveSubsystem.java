package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.roadrunner.MecanumDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.bosch.BHI260IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.Math;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class DriveSubsystem extends SubsystemBase {

    OpMode opMode;
    Telemetry telemetry;
    //Constants
    static final String IMU_NAME = "imu";
    static final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );
    static final String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
    static final String FRONT_RIGHT_MOTOR_NAME = "frontRightMotor";
    static final String BACK_LEFT_MOTOR_NAME = "rearLeftMotor";
    static final String BACK_RIGHT_MOTOR_NAME = "rearRightMotor";
    static final DcMotorSimple.Direction FRONT_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction FRONT_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    static final DcMotorSimple.Direction BACK_LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    static final DcMotorSimple.Direction BACK_RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    //Hardware
    private final DcMotorEx frontLeftMotor;
    private final DcMotorEx frontRightMotor;
    public final DcMotorEx backLeftMotor;
    private final DcMotorEx backRightMotor;
    private final MecanumDrive mecanumDrive;
    private final BHI260IMU imu;

    //Showing moving velocity
    private double fieldCentricStrafe;
    private double fieldCentricDrive;
    private double turn;

    //Additional Properties
    private double speedMultiplier = 1.0;
    private boolean fieldCentric = true;
    private boolean usingRoadRunner = true;

    //RR
    private FtcDashboard dash;
    private List<Action> runningActions;
    private Pose2d currentPos;
    private int TOLERANCE = 1;

    public DriveSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        imu = opMode.hardwareMap.get(BHI260IMU.class, IMU_NAME);
        imu.initialize(IMU_PARAMETERS);

        frontLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = opMode.hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, BACK_LEFT_MOTOR_NAME);
        backRightMotor = opMode.hardwareMap.get(DcMotorEx.class, BACK_RIGHT_MOTOR_NAME);
        mecanumDrive = new MecanumDrive(opMode.hardwareMap, new Pose2d(0, 0, 0));

        frontLeftMotor.setDirection(FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(FRONT_RIGHT_MOTOR_DIRECTION);
        backLeftMotor.setDirection(BACK_LEFT_MOTOR_DIRECTION);
        backRightMotor.setDirection(BACK_RIGHT_MOTOR_DIRECTION);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //for RR
        this.dash = FtcDashboard.getInstance();
        runningActions = new ArrayList<>();

        resetGyro();
        resetEncoders();
    }

    //Physical Operations

    public void drive(double forward, double strafe, double turn) {
        if (fieldCentric && usingRoadRunner) driveFieldCentricRoadRunner(forward, strafe, turn);
        else if (fieldCentric) driveFieldCentric(forward, strafe, turn);
        else if (usingRoadRunner) driveRobotCentricRoadRunner(forward, strafe, turn);
        else driveRobotCentric(forward, strafe, turn);
    }

    private void driveRobotCentricRoadRunner(double forward, double strafe, double turn) {
//        forward = Range.clip(forward, -1, 1);
//        strafe = Range.clip(strafe, -1, 1);
//        turn = Range.clip(turn, -1, 1);

        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                strafe * speedMultiplier,
                                forward * speedMultiplier),
                        turn * speedMultiplier)
        );
    }

    private void driveFieldCentricRoadRunner(double forward, double strafe, double turn) {
        forward = Range.clip(forward, -1, 1);
        strafe = Range.clip(strafe, -1, 1);
        turn = Range.clip(turn, -1, 1);

        double gyroRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double fieldCentricStrafe = strafe * Math.cos(-gyroRadians) - forward * Math.sin(-gyroRadians);
        double fieldCentricDrive = strafe * Math.sin(-gyroRadians) + forward * Math.cos(-gyroRadians);

        mecanumDrive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                fieldCentricStrafe * speedMultiplier,
                                fieldCentricDrive * speedMultiplier),
                        turn * speedMultiplier)
        );

    }

    //untested
    private void driveFieldCentric(double forward, double strafe, double turn) {
        double gyroRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        this.fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
        this.fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);
        this.turn = turn;

        frontLeftMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe - turn), -1, 1) * speedMultiplier);
    }

    //untested
    private void driveRobotCentric(double forward, double strafe, double turn) {
        frontLeftMotor.setPower(Range.clip((forward + strafe + turn), -1, 1) * speedMultiplier);
        frontRightMotor.setPower(Range.clip((forward - strafe - turn), -1, 1) * speedMultiplier);
        backLeftMotor.setPower(Range.clip((forward - strafe + turn), -1, 1) * speedMultiplier);
        backRightMotor.setPower(Range.clip((forward + strafe - turn), -1, 1) * speedMultiplier);
    }

    public void resetGyro() {
        imu.resetYaw();
    }

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Setters

    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Range.clip(multiplier, 0, 1);
    }

    public void setUsingFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void setUsingRoadRunner(boolean usingRoadRunner) {
        this.usingRoadRunner = usingRoadRunner;
    }

    //Getters

    public boolean getUsingFieldCentric() {
        return fieldCentric;
    }

    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    //Periodic

    @Override
    public void periodic() {
        telemetry.addData("fieldCentric", fieldCentric);
        mecanumDrive.updatePoseEstimate();
        currentPos = new Pose2d(mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, mecanumDrive.pose.heading.real);
    }

    //RR action wrapper
    public static class ActionCommand extends CommandBase {
        private final Action action;
        private boolean finished = false;
        private DriveSubsystem driveSubsystem;

        public ActionCommand(Action action, DriveSubsystem driveSubsystem) {
            this.driveSubsystem = driveSubsystem;
            this.action = action;

            addRequirements(driveSubsystem);
        }

        @Override
        public void execute() {
            TelemetryPacket packet = new TelemetryPacket();
            action.preview(packet.fieldOverlay());
            finished = !action.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        @Override
        public boolean isFinished() {
            return finished;
        }
    }

    public class ToBasket implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            mecanumDrive.actionBuilder(currentPos).
                    strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                    .build();
            mecanumDrive.updatePoseEstimate();
            return !(Math.abs(mecanumDrive.pose.position.x) <= TOLERANCE && Math.abs(mecanumDrive.pose.position.y) <= TOLERANCE && Math.toDegrees(Math.abs(mecanumDrive.pose.heading.real)) <= TOLERANCE);
        }
    }
    public Action ToBasket(DriveSubsystem driveSubsystem) {return new ToBasket();}

}
