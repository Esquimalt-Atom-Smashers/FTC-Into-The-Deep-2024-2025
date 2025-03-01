package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningWristSubsystem;

//@TeleOp(name="BadControlsTeleOp", group = "Real")
public class SpinnyTeleOp extends OpMode {
    ArmSubsystem armSubsystem;
    SpinningWristSubsystem spinningWristSubsystem;
    DriveSubsystem driveSubsystem;
    SpecimenArmSubsystem specimenArmSubsystem;

    private boolean firstTime = true;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        armSubsystem = new ArmSubsystem(this);
        spinningWristSubsystem = new SpinningWristSubsystem(this);
        driveSubsystem = new DriveSubsystem(this);
        specimenArmSubsystem = new SpecimenArmSubsystem(this);

        driveSubsystem.setUsingFieldCentric(false);

        bindOperatorControls();
        bindDriverControls();
    }

    private void bindOperatorControls() {
        armSubsystem.setLinearMaxPower(0.5);
        armSubsystem.setElbowMaxPower(0.5);


        //split
        Trigger readyScoreSpecimen = new Trigger(() -> gamepad2.dpad_left);
        readyScoreSpecimen.whenActive(() -> specimenArmSubsystem.readyScore());

        Trigger scoreSpecimen = new Trigger(() -> gamepad2.dpad_up);
        scoreSpecimen.whenActive(() -> specimenArmSubsystem.scoreSpecimen());

        Trigger intakeSpecimen = new Trigger(() -> gamepad2.dpad_right);
        intakeSpecimen.whenActive(() -> specimenArmSubsystem.liftPosition());

        Trigger receiveSpecimen = new Trigger(() -> gamepad2.dpad_down);
        receiveSpecimen.whenActive(() -> specimenArmSubsystem.wallPosition());

        Trigger openClaw = new Trigger(() -> gamepad2.right_bumper);
        openClaw.whenActive(() -> specimenArmSubsystem.openClaw());

        Trigger closeClaw = new Trigger(() -> gamepad2.left_bumper);
        closeClaw.whenActive(() -> specimenArmSubsystem.closeClaw());

        Trigger elbowControl = new Trigger(() -> Math.abs(gamepad2.left_stick_y) > 0);
        elbowControl.whileActiveContinuous(() -> armSubsystem.addToElbowTarget((int) (gamepad2.left_stick_y * -30)));

        Trigger linearControl = new Trigger(() -> Math.abs(gamepad1.right_stick_y) > 0);
        linearControl.whileActiveContinuous(() -> armSubsystem.addToLinearSlideTarget((int) (gamepad1.right_stick_y * -30)));

        Trigger elbowManualControl = new Trigger(() -> gamepad2.options);
        elbowManualControl.whileActiveContinuous(() -> {
            armSubsystem.addToElbowTarget((int) (gamepad2.left_stick_y * -30), true);
            armSubsystem.addToLinearSlideTarget((int) (gamepad2.right_stick_y * -30), true);
        });
    }

    private void bindDriverControls() {
        driveSubsystem.setSpeedMultiplier(0.5);

        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x));
        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger resetGyro = new Trigger(() -> gamepad1.back);
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger speedVariationTrigger = new Trigger(() -> gamepad1.right_trigger > 0);
        speedVariationTrigger.whileActiveContinuous(() -> driveSubsystem.setSpeedMultiplier(gamepad1.right_trigger * 0.5 + 0.5));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(0.5));

        //split
        Trigger highPosition = new Trigger(() -> gamepad1.dpad_up);
        highPosition.whenActive(() -> {
            spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.OUTTAKE);
            armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION, 0.8, 0.4, 0.25).schedule();
        });

        Trigger intakePosition = new Trigger(() -> gamepad1.dpad_down);
        intakePosition.whenActive(() -> {
            spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.STOWED);
            armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.INTAKE_POSITION, 0.8, 0.4, 0.25).schedule();
        });

        Trigger lowPosition = new Trigger(() -> gamepad1.dpad_right);
        lowPosition.whenActive(() -> {
            spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.OUTTAKE);
            armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.LOW_OUTTAKE_POSITION, 0.8, 0.4, 0.25).schedule();
        });

        Trigger intake = new Trigger(() -> gamepad1.right_bumper);
        intake.whenActive(() -> spinningWristSubsystem.intake());
        intake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger outtake = new Trigger(() -> gamepad1.left_bumper);
        outtake.whenActive(() -> spinningWristSubsystem.outtake());
        outtake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger wristIntake = new Trigger(() -> gamepad1.a);
        wristIntake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.INTAKE));

        Trigger wristOuttake = new Trigger(() -> (gamepad1.b));
        wristOuttake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.OUTTAKE));

        Trigger wristStorage = new Trigger(() -> (gamepad1.y));
        wristStorage.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.STOWED));

        Trigger resetEncoders = new Trigger(() -> gamepad1.share);
        resetEncoders.whenActive(() -> armSubsystem.resetEncoders());


    }

    @Override
    public void loop() {
        if(firstTime) {
            spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPositions.STOWED);
            firstTime = false;
        }
        CommandScheduler.getInstance().run();
    }
}