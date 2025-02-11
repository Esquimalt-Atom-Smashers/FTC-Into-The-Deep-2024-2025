package org.firstinspires.ftc.teamcode.opModes.auto;

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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningWristSubsystem;

@Autonomous(name = "LeftSampleAuto", group = "Real")
public final class LeftSampleAuto extends LinearOpMode {

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
                .strafeToLinearHeading( new Vector2d(50, 50), Math.toRadians(135));

        Actions.runBlocking(
            new SequentialAction(
                    intoPos1.build(),
                    GoToHighBasketAction(),
                    outtakePos1.build()
//                    spinningWristSubsystem.Outtake()
//                    intoPos2.build()
                    //GoToHomePositionAction()
            )
        );

        while(!isStopRequested() || opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }

    public class GoToHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            commandManager.getToHighBasketPositionCommand().schedule();
//            spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.OUTTAKE);
//            armSubsystem.setTargetLinearSlidePosition(armSubsystem.);
            return !armSubsystem.atHighBasketPosition();
        }
    }
    public Action GoToHighBasketAction() {
        return new GoToHighBasketAction();
    }

    public class GoToHomePositionAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            commandManager.getToHomePosition().schedule();
            return !armSubsystem.atIntakePosition();
        }
    }
    public Action GoToHomePositionAction() {
        return new GoToHomePositionAction();
    }
}