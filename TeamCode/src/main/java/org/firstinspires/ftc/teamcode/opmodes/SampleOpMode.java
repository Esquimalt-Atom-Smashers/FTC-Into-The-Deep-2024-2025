package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp(name="TeleOp", group = "Real")
public class SampleOpMode extends OpMode {
    ArmSubsystem armSubsystem;

    @Override
    public void init() {
        armSubsystem = new ArmSubsystem(this);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) armSubsystem.setTargetArmPosition(ArmSubsystem.ArmPosition.OUTTAKE_POSITION);
        if(gamepad1.dpad_down) armSubsystem.setTargetArmPosition(ArmSubsystem.ArmPosition.INTAKE_POSITION);
        if(gamepad1.right_stick_y > 0.1) armSubsystem.addToLinearSlideTarget((int) (gamepad1.right_stick_y * 25));

        armSubsystem.update();
    }
}