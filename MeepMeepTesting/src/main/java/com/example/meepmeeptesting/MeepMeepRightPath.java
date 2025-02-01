package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRightPath {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(80, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        Pose2d initialPose = new Pose2d(10, -62, Math.toRadians(90));

       TrajectoryActionBuilder placeSample0 = myBot.getDrive().actionBuilder(initialPose)
               .strafeToConstantHeading(new Vector2d(-1, -31));

       TrajectoryActionBuilder pushSamples = placeSample0.endTrajectory().fresh()
                       .strafeToConstantHeading(new Vector2d(26, -42))
                       .splineToConstantHeading(new Vector2d(43, -12), Math.toRadians(30))
                       .strafeToConstantHeading(new Vector2d(43, -50))
                       .splineToConstantHeading(new Vector2d(53, -13), Math.toRadians(10))
                       .strafeToConstantHeading(new Vector2d(53,-50))
                       .splineToConstantHeading(new Vector2d(63, -13), Math.toRadians(10))
                       .strafeToConstantHeading(new Vector2d(63, -50));

       TrajectoryActionBuilder specimen1 = pushSamples.endTrajectory().fresh()
                       .strafeToConstantHeading(new Vector2d(38, -65))
                        .strafeToConstantHeading(new Vector2d(-4, -31));

        TrajectoryActionBuilder specimen2 = specimen1.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(-2, -31));

        TrajectoryActionBuilder specimen3 = specimen2.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(0, -31));

        TrajectoryActionBuilder specimen4 = specimen3.endTrajectory().fresh()

                .strafeToConstantHeading(new Vector2d(38, -65))
                .strafeToConstantHeading(new Vector2d(1, -31));

        TrajectoryActionBuilder park = specimen4.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(38,-65));

       myBot.runAction(
               new SequentialAction(
                       placeSample0.build(),
                       pushSamples.build(),
                       specimen1.build(),
                       specimen2.build(),
                       specimen3.build(),
                       specimen4.build(),
                       park.build()
               )
       );

        // Configure and start the MeepMeep visualization
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
