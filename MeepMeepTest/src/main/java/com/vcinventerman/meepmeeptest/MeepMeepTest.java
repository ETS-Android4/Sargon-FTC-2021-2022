package com.vcinventerman.meepmeeptest;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTest {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800).setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(13, 8.375 * 2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(1 * 12, (6 * 12) - 8.375, Math.toRadians(270)))
                                .strafeTo(new Vector2d((1 * 12), (1 * 12) ))
                                .lineToLinearHeading(new Pose2d(new Vector2d((0.2 * 12), (0.8 * 12)), Math.toRadians(135))) // Go to team shipping hub
                                .waitSeconds(2)
                                .lineToLinearHeading(new Pose2d(new Vector2d((1 * 12), (1 * 12) ), Math.toRadians(0)))
                                .lineToLinearHeading(new Pose2d(new Vector2d(1 * 12, (5.5 * 12) - 8.375), Math.toRadians(0))) // Go near wall
                                .strafeLeft((.5 * 12))
                                .forward(2.5 * 12)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}