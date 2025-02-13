package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CommandManager;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpinningWristSubsystem;

@TeleOp(name = "TeleOp (one person)", group = "AA(at arena)")
public class OnePersonTeleOp extends OpMode {
    ArmSubsystem armSubsystem;
    SpinningWristSubsystem spinningWristSubsystem;
    DriveSubsystem driveSubsystem;
    SpecimenArmSubsystem specimenArmSubsystem;
    LEDSubsystem ledSubsystem;

    CommandManager commandManager;

    TelemetryPacket packet;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        driveSubsystem = new DriveSubsystem(this);
        specimenArmSubsystem = new SpecimenArmSubsystem(this);
        armSubsystem = new ArmSubsystem(this);
        spinningWristSubsystem = new SpinningWristSubsystem(this, armSubsystem, SpinningWristSubsystem.WristPosition.STOWED);
        ledSubsystem = new LEDSubsystem(this);

        commandManager = new CommandManager(armSubsystem, driveSubsystem, specimenArmSubsystem, spinningWristSubsystem);

        bindDriverControls();
    }

    private void bindDriverControls() {
        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));

        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger goToBasket = new Trigger(() -> gamepad1.start);
        goToBasket.whenActive(() -> commandManager.drivebaseToBasket().schedule());

        Trigger resetGyro = new Trigger(() -> gamepad1.back);
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger speedVariationTrigger = new Trigger(() -> gamepad1.right_trigger > 0);
        speedVariationTrigger.whileActiveContinuous(() -> driveSubsystem.setSpeedMultiplier(Math.abs(gamepad1.right_trigger - 1) * 0.4 + 0.2));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        armSubsystem.setLinearMaxPower(0.5);
        armSubsystem.setElbowMaxPower(0.5);

        Trigger highPosition = new Trigger(() -> gamepad1.dpad_up);
        highPosition.whenActive(() -> commandManager.getToHighBasketPositionCommand().schedule());

        Trigger intakePosition = new Trigger(() -> gamepad1.dpad_down);
        intakePosition.whenActive(() -> commandManager.getToHomePosition().schedule());

        Trigger lowPosition = new Trigger(() -> gamepad1.dpad_right);
        lowPosition.whenActive(() -> commandManager.getToLowBasketPosition().schedule());

        Trigger linearControl = new Trigger(() -> Math.abs(gamepad1.right_stick_y) > 0 && !gamepad1.options);
        linearControl.whileActiveContinuous(() -> armSubsystem.addToLinearSlideTarget((int) (gamepad1.right_stick_y * -50)));

        Trigger intake = new Trigger(() -> gamepad1.right_bumper);
        intake.whenActive(() -> spinningWristSubsystem.intake());
        intake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger outtake = new Trigger(() -> gamepad1.left_bumper);
        outtake.whenActive(() -> spinningWristSubsystem.outtake());
        outtake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger wristIntake = new Trigger(() -> gamepad1.a);
        wristIntake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.INTAKE));

        Trigger wristOuttake = new Trigger(() -> (gamepad1.b));
        wristOuttake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.OUTTAKE));

        Trigger wristStorage = new Trigger(() -> (gamepad1.y));
        wristStorage.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.STOWED));
    }

    @Override
    public void loop() {
        packet = new TelemetryPacket();
        CommandScheduler.getInstance().run();
    }
}