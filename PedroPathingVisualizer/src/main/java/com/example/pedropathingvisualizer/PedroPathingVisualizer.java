package com.example.pedropathingvisualizer;

import com.owlrobotics.visualizer.PathVisualizer;
import com.owlrobotics.visualizer.pedropathing.entities.PedroPathingBotEntity;
import com.owlrobotics.visualizer.pedropathing.paths.*;
import com.owlrobotics.visualizer.pedropathing.geometry.*;
import com.owlrobotics.visualizer.ui.theme.DarkTheme;
import com.owlrobotics.visualizer.util.enums.Backgrounds;
import com.owlrobotics.visualizer.util.enums.PlaneOrigin;
import com.owlrobotics.visualizer.util.enums.RobotImages;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class PedroPathingVisualizer {

    public static void main(String[] args) throws IOException {
        File file = new File("PedroPathingVisualizer/images/field_field.png");
        BufferedImage customFieldImage = ImageIO.read(file);

        PathVisualizer visualizer = new PathVisualizer(900, 144);

        PedroPathingBotEntity test = new PedroPathingBotEntity.Builder()
                .setRobotImage(RobotImages.Pedro_CLASSIC)
                .setRobotSize(16, 16)
                .build();

        test.createNewPath(new PathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(56, 36)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build());

        visualizer
                .setBackground(customFieldImage)
                .setPlaneOrigin(PlaneOrigin.BOTTOM_LEFT)
                .setTheme(new DarkTheme())
                .addEntity(test)
                .start();
    }
}