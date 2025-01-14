package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

@TeleOp(name="TeleOp", group = "Real")
public class SampleTeleOp extends OpMode {
    ArmSubsystem armSubsystem;
    WristSubsystem wristSubsystem;

    @Override
    public void init() {
        armSubsystem = new ArmSubsystem(this);
        wristSubsystem = new WristSubsystem(this);

        armSubsystem.setElbowMaxPower(0.5);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) armSubsystem.setTargetArmPosition(ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION);
        if(gamepad1.dpad_down) armSubsystem.setTargetArmPosition(ArmSubsystem.ArmPosition.INTAKE_POSITION);
        if(Math.abs(gamepad1.right_stick_y) > 0.1) armSubsystem.addToLinearSlideTarget((int) (gamepad1.right_stick_y * -5));
        if(Math.abs(gamepad1.left_stick_y) > 0.1) armSubsystem.addToElbowTarget((int) (gamepad1.left_stick_y * -5));

        if(gamepad1.left_bumper) wristSubsystem.closeClaw();
        if(gamepad1.right_bumper) wristSubsystem.openClaw();
        if(gamepad1.x) wristSubsystem.setWristPosition(WristSubsystem.WristPosition.INTAKE);
        if(gamepad1.y) wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE);

        armSubsystem.update();
    }
}