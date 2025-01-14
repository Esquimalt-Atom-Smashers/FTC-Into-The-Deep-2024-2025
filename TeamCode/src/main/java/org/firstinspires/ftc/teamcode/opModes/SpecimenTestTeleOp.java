package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

@TeleOp(name = "SpecimenTestTeleOp", group = "Real")
public class SpecimenTestTeleOp extends OpMode {


    private Servo elbowServo;
    private Servo clawServo;
    private Double elbowPos;
    private Double clawPos;

    public void init() {
        elbowServo = hardwareMap.get(Servo.class, "specElbow");
        clawServo = hardwareMap.get(Servo.class, "specClaw");

        elbowServo.resetDeviceConfigurationForOpMode();


        elbowPos = 0.0;
        clawPos = 0.0;
    }

    public void loop() {
        if(gamepad1.dpad_up){elbowPos = Range.clip(elbowPos + 0.1, -1, 1);}
        if(gamepad1.dpad_down){elbowPos = Range.clip(elbowPos - 0.1, -1, 1);}

        if(gamepad1.dpad_right){clawServo.setPosition(0);}
        if(gamepad1.dpad_left){clawServo.setPosition(1);}

        elbowServo.setPosition(elbowPos);

        telemetry.addData("elbowServo Ticks", elbowServo.getPosition());
        telemetry.addData("clawServo", clawServo.getPosition());
        telemetry.addData("clawPos", clawPos);
        telemetry.addData("elbowPos", elbowPos);
    }
}
