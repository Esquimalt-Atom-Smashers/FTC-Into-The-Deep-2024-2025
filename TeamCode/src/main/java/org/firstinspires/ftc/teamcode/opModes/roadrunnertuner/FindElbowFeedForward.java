package org.firstinspires.ftc.teamcode.opmodes.roadrunnertuner;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="FindElbowFeedForward", group = "RoadRunner")
@Disabled
public class FindElbowFeedForward extends OpMode {
    DcMotorEx elbowMotor;
    double powerToMotor = 0;

    @Override
    public void init() {
        elbowMotor = hardwareMap.get(DcMotorEx.class, "sampElbow");

        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elbowMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbowMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        if(gamepad1.a) powerToMotor += 0.001;
        if(gamepad1.b) powerToMotor -= 0.001;

        powerToMotor = Range.clip(powerToMotor, -1, 1);
        elbowMotor.setPower(powerToMotor);

        telemetry.addData("Power to motor", powerToMotor);
        telemetry.addData("Position", elbowMotor.getCurrentPosition());
    }
}
