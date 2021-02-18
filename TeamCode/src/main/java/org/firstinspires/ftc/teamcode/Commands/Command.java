package org.firstinspires.ftc.teamcode.Commands;

public class Command {
    public boolean isFinished;

    public Command() {
        isFinished = false;
    }

    public boolean isFinished(){
        return isFinished;
    }

    public void update() {};

}
