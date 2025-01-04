package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class armSubsystem extends OpMode {
    public DcMotorEx slide = null;
    public DcMotorEx elbow = null;

    public Servo wristServo;
    public Servo clawServo;

    public void init() {
        slide = hardwareMap.get(DcMotorEx.class, "slideMotor");
        elbow = hardwareMap.get(DcMotorEx.class, "elbowMotor");

        slide.setDirection(DcMotorEx.Direction.REVERSE);
        elbow.setDirection(DcMotorEx.Direction.FORWARD);

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elbow.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        clawServo = hardwareMap.get(Servo.class, "clawServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");

    }

    public void loop() {

        //reset button? (i dont fully know how these work)
        if (gamepad2.back){
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        //The following are different positions on gamepad2 for the arm, wrist, and claw.
        //All arm positions on dpad, all wrist on buttons, and all claw on bumpers.

        //positions for the arm

        //low basket
        if (gamepad2.dpad_left){
            slide.setTargetPosition(980);
            elbow.setTargetPosition(815);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(0.3);
            elbow.setPower(0.2);
        }
        //full extend
        if (gamepad2.dpad_up){
            slide.setTargetPosition(2100);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(0.3);
        }
        //storage position
        if (gamepad2.dpad_right){
            slide.setTargetPosition(50);
            elbow.setTargetPosition(0);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(0.4);
            elbow.setPower(0.18);
        }
        //grabbing position
        if (gamepad2.dpad_down){
            slide.setTargetPosition(720);
            elbow.setTargetPosition(1);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            elbow.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setPower(0.2);
            elbow.setPower(0.3);
        }

        //positions for the wrist

        //collapsed
        if (gamepad2.x){
            wristServo.setPosition(.84);
        }
        //readying
        if (gamepad2.b){
            wristServo.setPosition(.15);
        }
        //grabbing
        if (gamepad2.a){
            wristServo.setPosition(.04);
        }

        //scoring
        if (gamepad2.y){
            wristServo.setPosition(.67);
        }

        //positions for the claw

        //open
        if(gamepad2.right_bumper){
            clawServo.setPosition(.5);
        }
        //closed
        if(gamepad2.left_bumper){
            clawServo.setPosition(.1);
        }

        telemetry.addData("Slide Position", slide.getCurrentPosition());
        telemetry.addData("Elbow Position", elbow.getCurrentPosition());
        telemetry.update();
    }
}
