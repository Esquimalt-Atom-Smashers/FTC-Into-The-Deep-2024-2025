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

@Autonomous(name = "LeftBluePushAuto", group = "Real")
public final class LeftBluePushAuto extends LinearOpMode {
    private  MecanumDrive drive;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(-16.5,65 , Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)

                        .strafeToConstantHeading( new Vector2d(-35.75, 60) )
                        .strafeToConstantHeading(new Vector2d(-35.75,30))

                        .splineToConstantHeading(new Vector2d(-47.6,16),Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d( -47.6, 63))

                        .splineToConstantHeading(new Vector2d(-56.75,16),Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d(-56.75,63))

                        .setTangent(Math.toRadians(270))
                        .strafeToConstantHeading(new Vector2d(-56,63))
                        .splineToLinearHeading(new Pose2d(-62, 20, Math.toRadians(0)), Math.toRadians(90))
                        .strafeToConstantHeading(new Vector2d(-62, 54))

                        .splineToLinearHeading(new Pose2d(-48, 62 ,Math.toRadians(270)), Math.toRadians(180))
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