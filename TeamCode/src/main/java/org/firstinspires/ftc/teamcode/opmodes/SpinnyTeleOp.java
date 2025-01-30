package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningWristSubsystem;

@TeleOp(name="TeleOp", group = "Real")
public class SpinnyTeleOp extends OpMode {
    ArmSubsystem armSubsystem;
    SpinningWristSubsystem spinningWristSubsystem;
    DriveSubsystem driveSubsystem;
    SpecimenArmSubsystem specimenArmSubsystem;

    CommandManager commandManager;

    private boolean firstTime = true;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        spinningWristSubsystem = new SpinningWristSubsystem(this);
        driveSubsystem = new DriveSubsystem(this);
        specimenArmSubsystem = new SpecimenArmSubsystem(this);
        armSubsystem = new ArmSubsystem(this, spinningWristSubsystem);

        commandManager = new CommandManager(armSubsystem, driveSubsystem, specimenArmSubsystem, spinningWristSubsystem);

        driveSubsystem.setUsingFieldCentric(false);

        bindOperatorControls();
        bindDriverControls();
    }

    private void bindOperatorControls() {
        armSubsystem.setLinearMaxPower(0.5);
        armSubsystem.setElbowMaxPower(0.5);

        Trigger highPosition = new Trigger(() -> gamepad2.dpad_up);
        highPosition.whenActive(() -> commandManager.getToHighBasketPositionCommand().schedule());

        Trigger intakePosition = new Trigger(() -> gamepad2.dpad_down);
        intakePosition.whenActive(() -> commandManager.getToHomePosition().schedule());

        Trigger lowPosition = new Trigger(() -> gamepad2.dpad_right);
        lowPosition.whenActive(() -> commandManager.getToLowBasketPosition().schedule());

        Trigger linearControl = new Trigger(() -> Math.abs(gamepad2.right_stick_y) > 0 && !gamepad2.options);
        linearControl.whileActiveContinuous(() -> armSubsystem.addToLinearSlideTarget((int) (gamepad2.right_stick_y * -30)));

        Trigger intake = new Trigger(() -> gamepad2.right_bumper);
        intake.whenActive(() -> spinningWristSubsystem.intake());
        intake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger outtake = new Trigger(() -> gamepad2.left_bumper);
        outtake.whenActive(() -> spinningWristSubsystem.outtake());
        outtake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger wristIntake = new Trigger(() -> gamepad2.a);
        wristIntake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.INTAKE));

        Trigger wristOuttake = new Trigger(() -> (gamepad2.b));
        wristOuttake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.OUTTAKE));

        Trigger wristStorage = new Trigger(() -> (gamepad2.y));
        wristStorage.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.STOWED));

        Trigger resetEncoders = new Trigger(() -> gamepad2.share);
        resetEncoders.whenActive(() -> armSubsystem.resetEncoders());

        Trigger elbowControl = new Trigger(() -> Math.abs(gamepad2.left_stick_y) > 0 && !gamepad2.options);
        elbowControl.whileActiveContinuous(() -> armSubsystem.addToElbowTarget((int) (gamepad2.left_stick_y * -30)));

        Trigger elbowManualOverrideControl = new Trigger(() -> gamepad2.options);
        elbowManualOverrideControl.whileActiveContinuous(() -> {
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

        Trigger readyScoreSpecimen = new Trigger(() -> gamepad1.square);
        readyScoreSpecimen.whenActive(() -> specimenArmSubsystem.readyScore());

        Trigger scoreSpecimen = new Trigger(() -> gamepad1.triangle);
        scoreSpecimen.whenActive(() -> specimenArmSubsystem.scoreSpecimen());

        Trigger intakeSpecimen = new Trigger(() -> gamepad1.circle);
        intakeSpecimen.whenActive(() -> specimenArmSubsystem.liftPosition());

        Trigger receiveSpecimen = new Trigger(() -> gamepad1.cross);
        receiveSpecimen.whenActive(() -> specimenArmSubsystem.wallPosition());

        Trigger openClaw = new Trigger(() -> gamepad1.right_bumper);
        openClaw.whenActive(() -> specimenArmSubsystem.openClaw());

        Trigger closeClaw = new Trigger(() -> gamepad1.left_bumper);
        closeClaw.whenActive(() -> specimenArmSubsystem.closeClaw());
    }

    @Override
    public void loop() {
        if(firstTime) {
            spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.STOWED);
            firstTime = false;
        }
        CommandScheduler.getInstance().run();
    }
}