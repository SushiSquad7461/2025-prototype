package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Swerve;

/* Notes for Auto Maker:
 * Start at B1/B2/B3 position with preloaded game piece
 * Add event marker "prepareL3" as robot starts moving
 * Once in scoring position, add event marker "scoreManipulator"
 * ONE PIECE END => End with "resetToIdle." Continue reading for TWO PIECE.
 * At coral station, add event marker "runManipulator"
 * Include a wait time of ~0.5-1.0 seconds to ensure intake
 * Once coral is acquired (beambreak triggered), path to scoring position
 * Add event marker "prepareL3" during return movement
 * Add event marker "scoreManipulator" when in position
 * End with "resetToIdle"
 */

public class AutoCommands {
    private final NetworkTable autoTable;
    private final StringPublisher selectedAutoPublisher;
    private final Map<String, Command> autoCommands = new HashMap<>();
    private String selectedAuto = "Nothing";

    public AutoCommands(Swerve swerve) {
        autoTable = NetworkTableInstance.getDefault().getTable("Auto");
        selectedAutoPublisher = autoTable.getStringTopic("selectedAuto").publish();
        selectedAutoPublisher.set("Nothing");

        registerAutoCommand("Nothing", new InstantCommand());
        //registerAutoCommand(name, makeAuto(name));

        var autoSubscriber = autoTable.getStringTopic("selectedAuto").subscribe("Nothing");
        selectedAuto = autoSubscriber.get();
    }

    private void registerAutoCommand(String name, Command command) {
        autoCommands.put(name, command);
        autoTable.getStringArrayTopic("availableAutos").publish().set(
            autoCommands.keySet().toArray(new String[0])
        );
    }

    private Command makeAuto(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public Command getAuto() {
        return autoCommands.getOrDefault(selectedAuto, new InstantCommand());
    }
}