package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class PIDTuning extends OpMode {
    private ArmSubsystem armSubsystem;
    int armTarget;
    int slideTarget;

    @Override
    public void init() {
        armSubsystem = new ArmSubsystem(this);
        armTarget = 0;
        slideTarget = 0;
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up) armTarget += 10;
        if(gamepad1.dpad_down) armTarget -= 10;
        if(gamepad1.dpad_right) slideTarget += 10;
        if(gamepad1.dpad_left) slideTarget -= 10;

        armSubsystem.setTargetArmPosition(armTarget, slideTarget);
        armSubsystem.update();
    }
}
