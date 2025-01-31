//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.hardware.IMU;
//
//
//@TeleOp(name = "Mecanum Drive with Field Centric", group = "Drive Test")
//public class MecanumDriveWithFieldCentric extends OpMode {
//
//    // Drive
//    public DcMotorEx frontLeft;
//    public DcMotorEx frontRight;
//    public DcMotorEx backLeft;
//    public DcMotorEx backRight;
//
//    private IMU imu;
//
//    private boolean FIELD_CENTRIC = true;
//    private double SPEED_MULTIPLIER = 1.0;
//
//    private double DEADZONE = 0.1;
//
//
//    @Override
//    public void init(){
//        imu = hardwareMap.get(IMU.class, "imu");
//
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
//        backLeft = hardwareMap.get(DcMotorEx.class, "rearLeftMotor");
//        backRight = hardwareMap.get(DcMotorEx.class, "rearRightMotor");
//
//        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
//        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addData("Status", "Initialized");
//    }
//
//    @Override
//    public void loop() {
//        // // Drive
//        // double forward = -gamepad1.left_stick_y;
//        // double strafe = gamepad1.left_stick_x;
//        // double turn = gamepad1.right_stick_x;
//
//        // drive(forward, strafe, turn, FIELD_CENTRIC, SPEED_MULTIPLIER)
//
//        configureIMU();
//        drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, FIELD_CENTRIC, SPEED_MULTIPLIER);
//    }
//
//    /** Configure the gyro */
//    private void configureIMU() {
//        IMU.Parameters parameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.UP
//                )
//        );
//        imu.initialize(parameters);
//        imu.resetYaw();
//    }
//
//    public void drive(double forward, double strafe, double turn, boolean fieldCentric, double multiplier) {
//        // Dead zone for the joysticks
//        forward = Math.abs(forward) >= DEADZONE ? forward : 0;
//        strafe = Math.abs(strafe) >= DEADZONE ? strafe : 0;
//        turn = Math.abs(turn) >= DEADZONE ? turn : 0;
//        multiplier = Range.clip(multiplier, 0, 1);
//
//        if (fieldCentric) {
//            // Field centric drive
//            double gyroRadians = Math.toRadians(-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            double fieldCentricStrafe = strafe * Math.cos(gyroRadians) - forward * Math.sin(gyroRadians);
//            double fieldCentricDrive = strafe * Math.sin(gyroRadians) + forward * Math.cos(gyroRadians);
//
//            frontLeft.setPower(scaleInput(fieldCentricDrive + fieldCentricStrafe + turn, multiplier));
//            frontRight.setPower(scaleInput(fieldCentricDrive - fieldCentricStrafe + turn, multiplier));
//            backLeft.setPower(scaleInput(fieldCentricDrive - fieldCentricStrafe - turn, multiplier));
//            backRight.setPower(scaleInput(fieldCentricDrive + fieldCentricStrafe - turn, multiplier));
//        }
//        else {
//            // Robot centric drive
//            frontLeft.setPower(scaleInput(forward + strafe + turn, multiplier));
//            frontRight.setPower(scaleInput(forward - strafe - turn, multiplier));
//            backLeft.setPower(scaleInput(forward - strafe + turn, multiplier));
//            backRight.setPower(scaleInput(forward + strafe - turn, multiplier));
//        }
//
//    }
//
//    private double scaleInput(double input, double multiplier) {
//        return Range.clip(input * multiplier, -1, 1);
//    }
//}