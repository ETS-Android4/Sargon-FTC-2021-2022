package org.firstinspires.ftc.teamcode.commands;

public interface Command {
    Boolean succeeded();

    // Returns whether the command needs to be continued
    Boolean run();

    void cont();
}
