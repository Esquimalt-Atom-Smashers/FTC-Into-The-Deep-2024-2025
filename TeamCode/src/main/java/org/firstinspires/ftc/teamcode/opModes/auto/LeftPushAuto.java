package org.firstinspires.ftc.teamcode.opmodes.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;

@Autonomous(name = "LeftPushAuto", group = "Push Auto")
public final class LeftPushAuto extends LinearOpMode {
    private MecanumDrive drive;
    private SpecimenArmSubsystem specimenArmSubsystem;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(41,65 , Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        this.specimenArmSubsystem = new SpecimenArmSubsystem(this);

        waitForStart();

        specimenArmSubsystem.touchRung();
        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .strafeToConstantHeading( new Vector2d(60,55))
                        //first yellow block
                        .strafeToConstantHeading(new Vector2d(39,41))
                        .splineToConstantHeading(new Vector2d(43,20),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(43,40),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(60,60) , Math.toRadians(90))
                        //second yellow block
                        .strafeToConstantHeading(new Vector2d(39,41))
                        .splineToConstantHeading(new Vector2d(56,20), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(58,61))
                        //third yellow
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(47,41,Math.toRadians(90)), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(61.5,20),Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(61.5,60))

                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(20, 20, Math.toRadians(90)), Math.toRadians(180))
                        .build());
    }

    public class TouchRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            specimenArmSubsystem.touchRung();
            return false;
        }
    }
    public Action TouchRung(SpecimenArmSubsystem specimenArmSubsystem) {
        return new TouchRung();
    }
}