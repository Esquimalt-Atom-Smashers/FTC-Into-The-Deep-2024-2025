package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="BasicTeleOp-Aviva", group="Basic_TeleOp")

public class BasicTeleOPAvivaftKieran extends OpMode{
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private Servo servo = null;
    private IMU imu = null;
    private boolean fieldCentric = false;
    private double servoTurn;
    public void init(){
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "rearLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRight  = hardwareMap.get(DcMotor.class, "rearRightMotor");
        servo  = hardwareMap.get(Servo.class, "servo");
        imu = hardwareMap.get(IMU.class, "imu");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        imu.initialize(parameters);
    }

    public void loop() {
        double forward = gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;

        if (gamepad1.dpad_left) {
            servoTurn = 0;
        } else if (gamepad1.dpad_up) {
            servoTurn = 0.5;
        } else if (gamepad1.dpad_right) {
            servoTurn = 1;
        }
        servo.setPosition(servoTurn);
        boolean slowMode = gamepad1.right_bumper;
        //starting value
        if (gamepad1.a) {
            if (fieldCentric)
                fieldCentric = false;
        } else if (gamepad1.b) {
            if (!fieldCentric)
                fieldCentric = true;
        }

        telemetry.addData("", fieldCentric);

        if (gamepad1.back) {
            imu.resetYaw();
        }


        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = strafe * Math.cos(-botHeading) - forward * Math.sin(-botHeading);
        double rotY = strafe * Math.sin(-botHeading) + forward * Math.cos(-botHeading);
        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

        if (slowMode) {
            double slowModeMultiplier = 2.5;
            forward /= slowModeMultiplier;
            strafe  /= slowModeMultiplier;
            turn    /= slowModeMultiplier;
            rotX    /= slowModeMultiplier;
            rotY    /= slowModeMultiplier;
        }

        if (fieldCentric)
            driveFieldCentric(forward, strafe, turn, rotY, rotX, denominator);
        else
            drive(forward, strafe, turn);

    }

    private void drive(double forward, double strafe, double turn){
        frontLeft.setPower(scaleInput (forward + strafe + turn));
        backLeft.setPower(scaleInput  (forward - strafe + turn));
        frontRight.setPower(scaleInput(forward - strafe - turn));
        backRight.setPower(scaleInput (forward + strafe - turn));
    }

    private void driveFieldCentric(double forward, double strafe, double turn, double rotY, double rotX, double denominator){
        frontLeft.setPower(scaleInput ((rotY + rotX + turn) / denominator));
        backLeft.setPower(scaleInput  ((rotY - rotX + turn) / denominator));
        frontRight.setPower(scaleInput((rotY - rotX - turn) / denominator));
        backRight.setPower(scaleInput ((rotY + rotX - turn) / denominator));
    }

    private double scaleInput(double input){
        return Range.clip(input, -1, 1);
    }
}