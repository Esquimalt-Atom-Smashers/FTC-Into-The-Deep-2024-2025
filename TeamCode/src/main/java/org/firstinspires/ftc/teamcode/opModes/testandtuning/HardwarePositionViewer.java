package org.firstinspires.ftc.teamcode.opmodes.testandtuning;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name="MotorShower", group = "z. Test")
public class HardwarePositionViewer extends OpMode {
    DcMotorEx armMotor;
    DcMotorEx linearSlideMotor;
    Servo wristServo;
    Servo clawServo;

    @Override
    public void init() {
        armMotor = hardwareMap.get(DcMotorEx.class, "sampElbow");
        linearSlideMotor = hardwareMap.get(DcMotorEx.class, "sampSlide");
        wristServo = hardwareMap.get(Servo.class, "sampWrist");
        clawServo = hardwareMap.get(Servo.class, "sampClaw");

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
        telemetry.addData("Wrist Position", wristServo.getPosition());
        telemetry.addData("Claw Position", clawServo.getPosition());
    }
}
