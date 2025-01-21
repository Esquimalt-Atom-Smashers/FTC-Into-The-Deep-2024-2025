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

    private boolean firstTime = true;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        armSubsystem = new ArmSubsystem(this);
        wristSubsystem = new WristSubsystem(this);
        driveSubsystem = new DriveSubsystem(this);

        driveSubsystem.setUsingRoadRunner(false);

        bindOperatorControls();
        bindDriverControls();
    }

    private void bindOperatorControls() {
        Trigger highPosition = new Trigger(() -> gamepad2.dpad_up);
        highPosition.whenActive(() -> {
            wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY);
            armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION, 0.6).schedule();
        });

        Trigger intakePosition = new Trigger(() -> gamepad2.dpad_down);
        intakePosition.whenActive(() -> {
            wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY);
            armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.INTAKE_POSITION, 0.6).schedule();
        });

        Trigger lowPosition = new Trigger(() -> gamepad2.dpad_right);
        lowPosition.whenActive(() -> {
            wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY);
            armSubsystem.getMoveArmToPositionCommand(ArmSubsystem.ArmPosition.LOW_OUTTAKE_POSITION, 0.6).schedule();
        });

        Trigger linearControl = new Trigger(() -> Math.abs(gamepad2.right_stick_y) > 0);
        linearControl.whileActiveContinuous(() -> armSubsystem.addToLinearSlideTarget((int) (gamepad2.right_stick_y * -15)));

        Trigger elbowControl = new Trigger(() -> Math.abs(gamepad2.left_stick_y) > 0);
        elbowControl.whileActiveContinuous(() -> armSubsystem.addToElbowTarget((int) (gamepad2.left_stick_y * -15)));

        Trigger toggleClaw = new Trigger(() -> gamepad2.right_bumper);
        toggleClaw.whenActive(() -> wristSubsystem.toggleClaw());

        Trigger wristIntake = new Trigger(() -> gamepad2.a);
        wristIntake.whenActive(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.INTAKE));

        Trigger wristOuttake = new Trigger(() -> gamepad2.y || gamepad2.x);
        wristOuttake.whenActive(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE));

        Trigger wristReady = new Trigger(() -> gamepad2.b);
        wristReady.whenActive(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY));

        Trigger resetEncoders = new Trigger(() -> gamepad2.back && gamepad2.start);
        resetEncoders.whenActive(() -> armSubsystem.resetEncoders());
    }

    private void bindDriverControls() {
        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x));
        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger resetGyro = new Trigger(() -> gamepad1.back);
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger speedVariationTrigger = new Trigger(() -> gamepad1.right_trigger > 0);
        speedVariationTrigger.whileActiveContinuous(() -> driveSubsystem.setSpeedMultiplier(gamepad1.right_trigger * 0.8 + 0.2));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(0.2));
    }

    @Override
    public void loop() {
        if(firstTime) {
            wristSubsystem.setWristPosition(WristSubsystem.WristPosition.COLLAPSED);
            firstTime = false;
        }
        CommandScheduler.getInstance().run();
    }
}