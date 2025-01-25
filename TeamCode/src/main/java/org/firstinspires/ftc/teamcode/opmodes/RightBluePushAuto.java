package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;

@Autonomous(name = "RightBluePushAuto", group = "Real")
public final class RightBluePushAuto extends LinearOpMode {
    private  MecanumDrive drive;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(41,65 , Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .strafeToConstantHeading( new Vector2d(62,55))
                        //first yellow block
                        .strafeToConstantHeading(new Vector2d(35,35))
                        .splineToConstantHeading(new Vector2d(45,20),Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(58,64) , Math.toRadians(90))
                        //second yellow block
                        .strafeToConstantHeading(new Vector2d(35,35))
                        .splineToConstantHeading(new Vector2d(54,20), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(54,64))
                        //third yellow
                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(45,35,Math.toRadians(180)), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(60,20),Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(54,64))

                        .setTangent(Math.toRadians(270))
                        .splineToLinearHeading(new Pose2d(17, 20, Math.toRadians(270)), Math.toRadians(180))
                        .build());
    }

    public class updatePose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            drive.updatePoseEstimate();
            return false;
        }
    }
    public Action updatePose(MecanumDrive drive) {
        return new updatePose();
    }
}