package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name="TeleOp", group = "Real")
public class MainTeleOp extends OpMode {
    ArmSubsystem armSubsystem;
    WristSubsystem wristSubsystem;
    DriveSubsystem driveSubsystem;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        armSubsystem = new ArmSubsystem(this);
        wristSubsystem = new WristSubsystem(this);
        driveSubsystem = new DriveSubsystem(this);

        armSubsystem.setElbowMaxPower(0.5);
        driveSubsystem.setUsingRoadRunner(false);

        bindOperatorControls();
        bindDriverControls();
    }

    private void bindOperatorControls() {
        Trigger highPosition = new Trigger(() -> gamepad2.dpad_up);
        highPosition.whenActive(() -> armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION, 0.3).schedule());

        Trigger intakePosition = new Trigger(() -> gamepad2.dpad_down);
        intakePosition.whenActive(() -> armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.INTAKE_POSITION, 0.3).schedule());

        Trigger lowPosition = new Trigger(() -> gamepad2.dpad_right);
        lowPosition.whenActive(() -> armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.LOW_OUTTAKE_POSITION, 0.3).schedule());

        Trigger linearControl = new Trigger(() -> Math.abs(gamepad2.right_stick_y) > 0.1);
        linearControl.whileActiveContinuous(() -> armSubsystem.addToLinearSlideTarget((int) (gamepad2.right_stick_y * -15)));

        Trigger elbowControl = new Trigger(() -> Math.abs(gamepad2.left_stick_y) > 0.1);
        elbowControl.whileActiveContinuous(() -> armSubsystem.addToElbowTarget((int) (gamepad2.left_stick_y * -15)));

        Trigger toggleClaw = new Trigger(() -> gamepad2.a);
        toggleClaw.whenActive(() -> wristSubsystem.toggleClaw());

        Trigger wristIntake = new Trigger(() -> gamepad2.x);
        wristIntake.whenActive(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.INTAKE));

        Trigger wristOuttake = new Trigger(() -> gamepad2.y);
        wristOuttake.whenActive(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE));

        Trigger wristReady = new Trigger(() -> gamepad2.b);
        wristReady.whenActive(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY));
    }

    private void bindDriverControls() {
        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger resetGyro = new Trigger(() -> gamepad1.back);
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger speedVariationTrigger = new Trigger(() -> gamepad1.right_trigger > 0.1);
        speedVariationTrigger.whileActiveContinuous(() -> driveSubsystem.setSpeedMultiplier(Math.abs(gamepad1.right_trigger - 1) * 0.4 + 0.2));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }
}