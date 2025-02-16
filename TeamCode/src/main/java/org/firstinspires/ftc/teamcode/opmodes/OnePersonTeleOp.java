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

@TeleOp(name = "TeleOp (one person)", group = "Competition TeleOp")
public class OnePersonTeleOp extends OpMode {
    ArmSubsystem armSubsystem;
    SpinningWristSubsystem spinningWristSubsystem;
    DriveSubsystem driveSubsystem;
    SpecimenArmSubsystem specimenArmSubsystem;
    LEDSubsystem ledSubsystem;

    CommandManager commandManager;

    TelemetryPacket packet;

    enum ControlMode {
        SPECIMEN_MODE,
        SAMPLE_MODE
    }
    ControlMode currentMode;

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
        currentMode = ControlMode.SAMPLE_MODE;
    }

    private void bindDriverControls() {
        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x));

        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger resetGyro = new Trigger(() -> gamepad1.back);
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger speedVariationTrigger = new Trigger(() -> gamepad1.right_trigger > 0);
        speedVariationTrigger.whileActiveContinuous(() -> driveSubsystem.setSpeedMultiplier(Math.abs(gamepad1.right_trigger - 1) * 0.4 + 0.2));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger changeControlMode = new Trigger(() -> gamepad1.options);
        changeControlMode.whenActive(() -> {
            switch (currentMode) {
                case SAMPLE_MODE:
                    currentMode = ControlMode.SPECIMEN_MODE;
                    commandManager.getToHomePosition().schedule();
                    ledSubsystem.inSampleMode();
                    break;
                case SPECIMEN_MODE:
                    currentMode = ControlMode.SAMPLE_MODE;
                    ledSubsystem.inSpecimenMode();
                    break;
            }
        });

        armSubsystem.setLinearMaxPower(0.5);
        armSubsystem.setElbowMaxPower(0.5);

        //sample mode control
        Trigger highPosition = new Trigger(() -> gamepad1.dpad_up && currentMode == ControlMode.SAMPLE_MODE);
        highPosition.whenActive(() -> commandManager.getToHighBasketPositionCommand().schedule());

        Trigger intakePosition = new Trigger(() -> gamepad1.dpad_down && currentMode == ControlMode.SAMPLE_MODE);
        intakePosition.whenActive(() -> commandManager.getToHomePosition().schedule());

        Trigger lowPosition = new Trigger(() -> gamepad1.dpad_right && currentMode == ControlMode.SAMPLE_MODE);
        lowPosition.whenActive(() -> commandManager.getToLowBasketPosition().schedule());

        Trigger removeFromChamber = new Trigger(() -> gamepad1.dpad_left && Math.abs(armSubsystem.getElbowPosition()) <= 25 && (currentMode == ControlMode.SAMPLE_MODE));
        removeFromChamber.whenActive(() -> commandManager.getToHomePositionHorizontal().schedule());

        Trigger linearControl = new Trigger(() -> Math.abs(gamepad1.right_stick_y) > 0 && !gamepad1.options && currentMode == ControlMode.SAMPLE_MODE);
        linearControl.whileActiveContinuous(() -> armSubsystem.addToLinearSlideTarget((int) (gamepad1.right_stick_y * -50)));

        Trigger sampleIntake = new Trigger(() -> gamepad1.right_bumper && currentMode == ControlMode.SAMPLE_MODE);
        sampleIntake.whenActive(() -> spinningWristSubsystem.intake());
        sampleIntake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger sampleOuttake = new Trigger(() -> gamepad1.left_bumper && currentMode == ControlMode.SAMPLE_MODE);
        sampleOuttake.whenActive(() -> spinningWristSubsystem.outtake());
        sampleOuttake.whenInactive(() -> spinningWristSubsystem.stopIntakeServo());

        Trigger wristIntake = new Trigger(() -> gamepad1.a && currentMode == ControlMode.SAMPLE_MODE);
        wristIntake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.INTAKE));

        Trigger wristOuttake = new Trigger(() -> (gamepad1.b && currentMode == ControlMode.SAMPLE_MODE));
        wristOuttake.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.OUTTAKE));

        Trigger wristStorage = new Trigger(() -> (gamepad1.y && currentMode == ControlMode.SAMPLE_MODE));
        wristStorage.whenActive(() -> spinningWristSubsystem.toPosition(SpinningWristSubsystem.WristPosition.STOWED));

        //specimen mode control
        Trigger specimenOpenClaw = new Trigger(() -> gamepad1.right_bumper && currentMode == ControlMode.SPECIMEN_MODE);
        specimenOpenClaw.whenActive(() -> specimenArmSubsystem.openClaw());

        Trigger specimenCloseClaw = new Trigger(() -> gamepad1.left_bumper && currentMode == ControlMode.SPECIMEN_MODE);
        specimenCloseClaw.whenActive(() -> specimenArmSubsystem.closeClaw());

        Trigger putDown = new Trigger(() -> gamepad1.square && currentMode == ControlMode.SPECIMEN_MODE);
        putDown.whenActive(() -> specimenArmSubsystem.putDown());

        Trigger scoreSpecimen = new Trigger(() -> gamepad1.triangle && currentMode == ControlMode.SPECIMEN_MODE);
        scoreSpecimen.whenActive(() -> specimenArmSubsystem.scoreSpecimen());

        Trigger intakeSpecimen = new Trigger(() -> gamepad1.circle && currentMode == ControlMode.SPECIMEN_MODE);
        intakeSpecimen.whenActive(() -> specimenArmSubsystem.liftPosition());

        Trigger receiveSpecimen = new Trigger(() -> gamepad1.cross && currentMode == ControlMode.SPECIMEN_MODE);
        receiveSpecimen.whenActive(() -> specimenArmSubsystem.wallPosition());
    }

    @Override
    public void loop() {
        packet = new TelemetryPacket();
        CommandScheduler.getInstance().run();
    }
}