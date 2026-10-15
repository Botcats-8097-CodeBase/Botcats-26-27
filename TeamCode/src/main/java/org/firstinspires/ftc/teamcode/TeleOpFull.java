package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.ElementType;

@TeleOp (name = "TeleOpFull")
public class TeleOpFull extends OpMode {

    ElapsedTime et = new ElapsedTime();

    @Override
    public void init() {
        initRobot();
    }

    @Override
    public void start() {
        super.start();
        et.reset();

    }

    @Override
    public void loop() {

    }

    void initRobot() {

    }

    @Override
    public void init_loop() {

    }
}
