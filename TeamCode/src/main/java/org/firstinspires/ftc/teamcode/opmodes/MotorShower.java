package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="MotorShower", group = "Real")
public class MotorShower extends OpMode {
    DcMotorEx armMotor;
    DcMotorEx linearSlideMotor;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "elbowMotor");
        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        linearSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Linear Slide Position", linearSlideMotor.getCurrentPosition());
    }
}