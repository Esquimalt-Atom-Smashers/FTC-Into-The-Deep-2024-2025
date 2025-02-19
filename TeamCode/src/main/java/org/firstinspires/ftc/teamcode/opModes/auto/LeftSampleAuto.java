package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningWristSubsystem;

@Autonomous(name = "LeftSampleAuto", group = "Scoring Auto")
public class LeftSampleAuto extends LinearOpMode {

    DriveSubsystem driveSubsystem;
    SpecimenArmSubsystem specimenArmSubsystem;
    ArmSubsystem armSubsystem;
    SpinningWristSubsystem spinningWristSubsystem;
    CommandManager commandManager;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(41,65 , Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        driveSubsystem = new DriveSubsystem(this);
        specimenArmSubsystem = new SpecimenArmSubsystem(this);
        armSubsystem = new ArmSubsystem(this);
        spinningWristSubsystem = new SpinningWristSubsystem(this, armSubsystem, SpinningWristSubsystem.WristPosition.STOWED);

        commandManager = new CommandManager(armSubsystem, driveSubsystem, specimenArmSubsystem, spinningWristSubsystem);

        waitForStart();

        TrajectoryActionBuilder intoPos1 = drive.actionBuilder(beginPose)
                .strafeToLinearHeading( new Vector2d(50, 50), Math.toRadians(135));

        TrajectoryActionBuilder outtakePos1 = intoPos1.endTrajectory().fresh()
                .strafeToLinearHeading( new Vector2d(55, 55), Math.toRadians(135));

        TrajectoryActionBuilder intoPos2 = outtakePos1.endTrajectory().fresh()
                .strafeToLinearHeading( new Vector2d(50, 50), Math.toRadians(175));

        Actions.runBlocking(
            new ParallelAction(
                    new SequentialAction(
                        intoPos1.build(),
                        getGoToHighBasketAction(),
                        outtakePos1.build(),
                        spinningWristSubsystem.getOuttakeAction(),
                        intoPos2.build(),
                        getGoToHomePositionAction(),
                        new SleepAction(1),
                        armSubsystem.getSlideToPositionAction(armSubsystem, 50)
                    ),
                    new RunFTCLibCommands()
                    )
            );
    }

    public class GoToHighBasketAction implements Action {
        SequentialCommandGroup highBasketCommand = commandManager.getToHighBasketPositionCommand();
        ArmSubsystem.ArmToPositionCommand armToPositionCommand = new ArmSubsystem.ArmToPositionCommand(armSubsystem, ArmSubsystem.ArmPosition.INTAKE_POSITION);

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!highBasketCommand.isScheduled()) highBasketCommand.schedule();

            if(armSubsystem.getTargetLinearSlidePosition() == ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION.slidePos && armSubsystem.getTargetElbowPosition() == ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION.elbowPos) {
                return !armToPositionCommand.isFinished();
            } else return true;
        }
    }

    public static class RunFTCLibCommands implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            CommandScheduler.getInstance().run();
            return true;
        }
    }

    public Action getGoToHighBasketAction() {
        return new GoToHighBasketAction();
    }

    public class GoToHomePositionAction implements Action {
        SequentialCommandGroup homePositionCommand = commandManager.getToHomePosition();
        ArmSubsystem.ArmToPositionCommand armToPositionCommand = new ArmSubsystem.ArmToPositionCommand(armSubsystem, ArmSubsystem.ArmPosition.INTAKE_POSITION);

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!homePositionCommand.isScheduled()) homePositionCommand.schedule();

            if(armSubsystem.getTargetLinearSlidePosition() == ArmSubsystem.ArmPosition.INTAKE_POSITION.slidePos && armSubsystem.getTargetElbowPosition() == ArmSubsystem.ArmPosition.INTAKE_POSITION.elbowPos) {
                return !armToPositionCommand.isFinished();
            } else return true;
        }
    }

    public Action getGoToHomePositionAction() {
        return new GoToHomePositionAction();
    }
}