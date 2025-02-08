package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
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
        Pose2d beginPose = new Pose2d(-16.5,65 , Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        driveSubsystem = new DriveSubsystem(this);
        specimenArmSubsystem = new SpecimenArmSubsystem(this);
        armSubsystem = new ArmSubsystem(this);
        spinningWristSubsystem = new SpinningWristSubsystem(this, armSubsystem, SpinningWristSubsystem.WristPosition.STOWED);

        commandManager = new CommandManager(armSubsystem, driveSubsystem, specimenArmSubsystem, spinningWristSubsystem);

        waitForStart();

        TrajectoryActionBuilder intoPos = drive.actionBuilder(beginPose)
                .strafeToConstantHeading( new Vector2d(59, 59) );

        Actions.runBlocking(
                new ParallelAction(
                        intoPos.build()
                )
        );
    }

    public class GoToHighBasketAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            commandManager.getToHighBasketPositionCommand().schedule();
            return false;
        }
    }
    public Action GoToHighBasketAction() {
        return new GoToHighBasketAction();
    }

    public class GoToHomePositionAction implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            commandManager.getToHomePosition().schedule();
            return false;
        }
    }
    public Action GoToHomePositionAction() {
        return new GoToHomePositionAction();
    }
}