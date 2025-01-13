package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

@TeleOp(name="PID Tuner", group = "Real")
public class PIDTuning extends OpMode {
    private ArmSubsystem armSubsystem;
    int armTarget;
    int slideTarget;

    @Override
    public void init() {
        armSubsystem = new ArmSubsystem(this);
        armTarget = 400;
        slideTarget = 1000;
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) armTarget += 1;
        if(gamepad1.dpad_down) armTarget -= 1;
        if(gamepad1.dpad_right) slideTarget += 1;
        if(gamepad1.dpad_left) slideTarget -= 1;

        armSubsystem.setTargetArmPosition(armTarget, slideTarget);
        CommandScheduler.getInstance().run();
    }
}
