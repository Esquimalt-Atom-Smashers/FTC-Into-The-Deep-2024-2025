package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningWristSubsystem;

public class CommandManager {
    ArmSubsystem armSubsystem;
    DriveSubsystem driveSubsystem;
    SpecimenArmSubsystem specimenArmSubsystem;
    SpinningWristSubsystem spinningWristSubsystem;

    public CommandManager(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, SpecimenArmSubsystem specimenArmSubsystem, SpinningWristSubsystem spinningWristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.specimenArmSubsystem = specimenArmSubsystem;
        this.spinningWristSubsystem = spinningWristSubsystem;
    }

    public SequentialCommandGroup getToHighBasketPositionCommand() {
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();

        if(Math.abs(armSubsystem.getElbowPosition() - position.elbowPos) <= ArmSubsystem.TOLERANCE) {
            return new SequentialCommandGroup(
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, ArmSubsystem.SLIDE_MIN_POSITION, maxLinearPower),
                    new ArmSubsystem.ElbowToPositionCommand(armSubsystem, position.elbowPos, (position.elbowPos < armSubsystem.getElbowPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, position.slidePos, maxLinearPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getToLowBasketPosition() {
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.LOW_OUTTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();

        if(Math.abs(armSubsystem.getElbowPosition() - position.elbowPos) <= ArmSubsystem.TOLERANCE) {
            return new SequentialCommandGroup(
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, ArmSubsystem.SLIDE_MIN_POSITION, maxLinearPower),
                    new ArmSubsystem.ElbowToPositionCommand(armSubsystem, position.elbowPos, (position.elbowPos < armSubsystem.getElbowPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, position.slidePos, maxLinearPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getToHomePosition() {
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.INTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();

        if(Math.abs(armSubsystem.getElbowPosition() - position.elbowPos) <= ArmSubsystem.TOLERANCE) {
            return new SequentialCommandGroup(
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.STOWED),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, ArmSubsystem.SLIDE_MIN_POSITION, maxLinearPower),
                    new SpinningWristSubsystem.MoveWristToPositionCommand(spinningWristSubsystem, SpinningWristSubsystem.WristPosition.STOWED),
                    new ArmSubsystem.ElbowToPositionCommand(armSubsystem, position.elbowPos, (position.elbowPos < armSubsystem.getElbowPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, position.slidePos, maxLinearPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }
}