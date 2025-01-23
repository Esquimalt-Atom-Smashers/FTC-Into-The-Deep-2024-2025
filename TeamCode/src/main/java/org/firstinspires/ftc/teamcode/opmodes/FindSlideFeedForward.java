package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="FindSlideFeedForward", group = "Real")
public class FindSlideFeedForward extends OpMode {
    DcMotorEx slideMotor;
    double powerToMotor = 0;

    @Override
    public void init() {
        slideMotor = hardwareMap.get(DcMotorEx.class, "sampSlide");

        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a) powerToMotor += 0.001;
        if(gamepad1.b) powerToMotor -= 0.001;

        powerToMotor = Range.clip(powerToMotor, -1, 1);
        slideMotor.setPower(powerToMotor);

        telemetry.addData("Power to motor", powerToMotor);
        telemetry.addData("Position", slideMotor.getCurrentPosition());
    }
}
