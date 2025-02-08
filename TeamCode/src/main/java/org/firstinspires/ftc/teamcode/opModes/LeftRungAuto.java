package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;


@Autonomous(name = "LeftRungAuto", group = "Test")
public class LeftRungAuto extends LinearOpMode{
    private IMU imu;
    private DcMotorEx frontRightMotor;
    private DcMotorEx frontLeftMotor;
    private DcMotorEx backRightMotor;
    private DcMotorEx backLeftMotor;

    private SpecimenArmSubsystem specimenArmSubsystem;

    private ElapsedTime timer;

    public void runOpMode(){
        timer = new ElapsedTime();
        timer.reset();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        specimenArmSubsystem = new SpecimenArmSubsystem(this);

        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rearRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");

        frontRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD);

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.resetYaw();

        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        waitForStart();

        imu.resetYaw();
        telemetry.addData("heading", getHeading());
        telemetry.update();

        //start facing ourselves
        driveByTime(0.1, 0, 0, 1);
        specimenArmSubsystem.touchRung();
        sleep(50);

        driveByTime(0, -0.5, 0, 0.7);
        sleep(50);

        driveByTime(0, 0.5, 0, 0.9);
        sleep(50);

        driveByTime(0.5, 0, 0, 1.7);
        sleep(50);

        driveByTime(0, 0.5, 0, 1);

        sleep(1000);


    }

    private void drive(double forward, double strafe, double turn){

        double gyroRadians = Math.toRadians(-getHeading());
        double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
        double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);

        frontLeftMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe + turn), -1, 1) );
        frontRightMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe - turn), -1, 1) );
        backLeftMotor.setPower(Range.clip((fieldCentricDrive - fieldCentricStrafe + turn), -1, 1) );
        backRightMotor.setPower(Range.clip((fieldCentricDrive + fieldCentricStrafe - turn), -1, 1) );
    }

    private void driveByTime(double forward, double strafe, double turn, double duration){
        timer.reset();
        drive(forward, strafe, turn);

        while (timer.seconds() <= duration);

        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private void rotate(double targetDegree){
        double startingHeading = getHeading();

        double angleDifference = targetDegree - startingHeading;

        if ( Math.abs(angleDifference) < 180 && (angleDifference) > 0)  {
            //turn counter-clockwise
            while (Math.abs(getHeading() - targetDegree) > 40) {
                drive(0, 0, 0.6);
            }
            while (Math.abs(getHeading() - targetDegree) >= 1) {
                drive(0, 0, 0.2);
            }
            drive(0, 0, 0);
        } else {
            //turn clockwise
            while (Math.abs(getHeading() - targetDegree) > 40) {
                drive(0, 0, -0.6);
            }
            while (Math.abs(getHeading() - targetDegree) >= 1) {
                drive(0, 0, -0.2);
            }
            drive(0, 0, 0);
        }
    }

    private double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }



}