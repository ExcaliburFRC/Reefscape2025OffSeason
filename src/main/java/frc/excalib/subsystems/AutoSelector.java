package frc.excalib.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.HashMap;
import java.util.Map;

/**
 * Simple and powerful autonomous routine selector.
 * Makes it easy to select and configure autonomous routines from the dashboard.
 * 
 * Example:
 * <pre>
 * AutoSelector auto = AutoSelector.builder()
 *     .addAuto("Score 3 Pieces", scoreThreePieces())
 *     .addAuto("Score 2 + Mobility", scoreTwoAndMove())
 *     .addAuto("Just Mobility", justMobility())
 *     .addAuto("Do Nothing", Commands.none())
 *     .withDefault("Score 3 Pieces")
 *     .build();
 * 
 * // In getAutonomousCommand()
 * return auto.getSelected();
 * </pre>
 * 
 * @author Excalib Team
 */
public class AutoSelector {
    private final SendableChooser<Command> chooser;
    private final Map<String, Command> autos;
    
    private AutoSelector(Builder builder) {
        this.chooser = builder.chooser;
        this.autos = builder.autos;
    }
    
    /**
     * Create a new auto selector builder.
     * @return builder
     */
    public static Builder builder() {
        return new Builder();
    }
    
    /**
     * Get the selected autonomous command.
     * @return selected command
     */
    public Command getSelected() {
        Command selected = chooser.getSelected();
        return selected != null ? selected : Commands.none();
    }
    
    /**
     * Get auto by name.
     * @param name auto name
     * @return command or null
     */
    public Command getAuto(String name) {
        return autos.get(name);
    }
    
    /**
     * Builder for auto selector.
     */
    public static class Builder {
        private final SendableChooser<Command> chooser = new SendableChooser<>();
        private final Map<String, Command> autos = new HashMap<>();
        private String defaultAuto = null;
        
        /**
         * Add an autonomous routine.
         * @param name display name
         * @param command command to run
         * @return this builder
         */
        public Builder addAuto(String name, Command command) {
            autos.put(name, command);
            
            if (defaultAuto == null) {
                defaultAuto = name;
                chooser.setDefaultOption(name, command);
            } else {
                chooser.addOption(name, command);
            }
            
            return this;
        }
        
        /**
         * Set the default auto.
         * @param name name of the default auto
         * @return this builder
         */
        public Builder withDefault(String name) {
            if (!autos.containsKey(name)) {
                throw new IllegalArgumentException("Unknown auto: " + name);
            }
            this.defaultAuto = name;
            
            // Rebuild chooser with new default
            SendableChooser<Command> newChooser = new SendableChooser<>();
            newChooser.setDefaultOption(name, autos.get(name));
            
            for (Map.Entry<String, Command> entry : autos.entrySet()) {
                if (!entry.getKey().equals(name)) {
                    newChooser.addOption(entry.getKey(), entry.getValue());
                }
            }
            
            return this;
        }
        
        /**
         * Build and publish the auto selector.
         * @return configured selector
         */
        public AutoSelector build() {
            SmartDashboard.putData("Auto Selector", chooser);
            return new AutoSelector(this);
        }
    }
}
