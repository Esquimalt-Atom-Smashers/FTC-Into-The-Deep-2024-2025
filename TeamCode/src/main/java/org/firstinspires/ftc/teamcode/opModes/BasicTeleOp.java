package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp(name = "BasicTeleOp", group = "Real")
public class BasicTeleOp extends OpMode{
    private IMU imu;
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    private Servo servo;

    private boolean fieldCentric = false;

    private double speedMultiplier = 1.0;

    private ElapsedTime timer;

    private double heading;
    public void init(){
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rearRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");

        frontRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        frontLeftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        timer = new ElapsedTime();
        timer.reset();

        //servo = hardwareMap.get(Servo.class, "servo");


//        servo.setPosition(0);
//        telemetry.addData("servo pos", 0);
    }

    public void loop(){
        drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if (gamepad1.dpad_right) {
            if (fieldCentric) {
                fieldCentric = false;
            }
        }

        if (gamepad1.dpad_left) {
            if (!fieldCentric) {
                fieldCentric = true;
            }
        }

        if (gamepad1.dpad_up) {
            rotate(0);
        }

        if (gamepad1.back) {imu.resetYaw();}

        //speed toggle
        if (gamepad1.right_bumper){
            if (timer.seconds() >= 0.5) {
                timer.reset();
                if (speedMultiplier == 1) { speedMultiplier = 0.5;}
                else {speedMultiplier = 1;}
            }
        }
        heading = getHeading();

        telemetry.addData("heading", heading);
        telemetry.addData("fieldCentric", fieldCentric);
        telemetry.addData("right x", gamepad1.right_stick_x);
        telemetry.addData("timer", timer.seconds());
        telemetry.addData("speed multiplier", speedMultiplier);
        telemetry.addData("right bumper", gamepad1.right_bumper);
        telemetry.update();
    }

    private void drive(double forward, double strafe, double turn){
        final double DEADZONE = 0.1;

        forward = Math.abs(forward) >= DEADZONE ? forward : 0;
        strafe = Math.abs(strafe) >= DEADZONE ? strafe : 0;
        turn = Math.abs(turn) >= DEADZONE ? turn : 0;

        if (fieldCentric) {

            double gyroRadians = Math.toRadians(-getHeading());
            double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
            double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

            frontRightMotor.setPower(scaleInput(fieldCentricDrive - fieldCentricStrafe - turn) * speedMultiplier);
            frontLeftMotor.setPower (scaleInput(fieldCentricDrive + fieldCentricStrafe + turn) * speedMultiplier);
            backRightMotor.setPower (scaleInput(fieldCentricDrive + fieldCentricStrafe - turn) * speedMultiplier);
            backLeftMotor.setPower  (scaleInput(fieldCentricDrive - fieldCentricStrafe + turn) * speedMultiplier);

        } else {
            frontRightMotor.setPower(scaleInput(forward - strafe - turn) * speedMultiplier);
            frontLeftMotor.setPower (scaleInput(forward + strafe + turn) * speedMultiplier);
            backRightMotor.setPower (scaleInput(forward + strafe - turn) * speedMultiplier);
            backLeftMotor.setPower  (scaleInput(forward - strafe + turn) * speedMultiplier);
        }
    }

    private void rotate(double targetDegree){
        double startingHeading = getHeading();

        double angleDifference = targetDegree - startingHeading;
        if (angleDifference > 180){ angleDifference -= 360;
        } else if (angleDifference < -180) { angleDifference += 360;}

        if (angleDifference > 0)  {
            //turn counter-clockwise
            while (Math.abs(getHeading() - targetDegree) > 40) {
                drive(0, 0, 0.5);
            }
            while (Math.abs(getHeading() - targetDegree) >= 1) {
                drive(0, 0, 0.1);
            }
            drive(0, 0, 0);
        } else {
            //turn clockwise
            while (Math.abs(getHeading() - targetDegree) > 40) {
                drive(0, 0, -0.5);
            }
            while (Math.abs(getHeading() - targetDegree) >= 1) {
                drive(0, 0, -0.1);
            }
            drive(0, 0, 0);
        }
    }


    private double scaleInput(double input){
        return Range.clip(input,-1,1);
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}
