package com.airbrakesplugin;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import net.sf.openrocket.document.Simulation;
import net.sf.openrocket.gui.SpinnerEditor;
import net.sf.openrocket.gui.adaptors.DoubleModel;
import net.sf.openrocket.gui.components.UnitSelector;
import net.sf.openrocket.plugin.Plugin;
import net.sf.openrocket.simulation.extension.AbstractSwingSimulationExtensionConfigurator;
import net.sf.openrocket.unit.UnitGroup;
import net.miginfocom.swing.MigLayout;

@Plugin
public class AirbrakeConfigurator
        extends AbstractSwingSimulationExtensionConfigurator<AirbrakeExtension> {

    public AirbrakeConfigurator() {
        super(AirbrakeExtension.class);
    }

    @Override
    protected JComponent getConfigurationComponent(
            AirbrakeExtension ext, Simulation sim, JPanel panel) {

        panel.setLayout(new MigLayout("wrap 3", "[right]10[grow,fill]10[]"));

        // --- CFD data CSV path chooser ---
        panel.add(new JLabel("CFD data CSV:"));
        JTextField pathField = new JTextField(ext.getCfdDataFilePath());
        pathField.getDocument().addDocumentListener(new DocumentListener() {
            @Override public void insertUpdate(DocumentEvent e) { ext.setCfdDataFilePath(pathField.getText()); }
            @Override public void removeUpdate(DocumentEvent e) { ext.setCfdDataFilePath(pathField.getText()); }
            @Override public void changedUpdate(DocumentEvent e) { ext.setCfdDataFilePath(pathField.getText()); }
        });
        panel.add(pathField, "span 1, growx");

        JButton browse = new JButton("Browse...");
        browse.addActionListener(e -> {
            JFileChooser chooser = new JFileChooser();
            chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
            if (chooser.showOpenDialog(panel) == JFileChooser.APPROVE_OPTION) {
                String p = chooser.getSelectedFile().getAbsolutePath();
                pathField.setText(p);
                ext.setCfdDataFilePath(p);
            }
        });
        panel.add(browse, "wrap");

        // --- Brake reference area spinner ---
        panel.add(new JLabel("Airbrake area:"));
        DoubleModel areaModel = new DoubleModel(
            ext, "ReferenceArea", UnitGroup.UNITS_AREA, ext.getReferenceArea()
        );

        JSpinner areaSpinner = new JSpinner(areaModel.getSpinnerModel());
        areaSpinner.setEditor(new SpinnerEditor(areaSpinner));
        panel.add(areaSpinner);
        panel.add(new UnitSelector(areaModel), "wrap");

        // --- Reference length spinner ---
        panel.add(new JLabel("Reference length:"));
        DoubleModel lengthModel = new DoubleModel(
            ext, "ReferenceLength", UnitGroup.UNITS_LENGTH, ext.getReferenceLength()
        );
        JSpinner lengthSpinner = new JSpinner(lengthModel.getSpinnerModel());
        lengthSpinner.setEditor(new SpinnerEditor(lengthSpinner));
        panel.add(lengthSpinner);
        panel.add(new UnitSelector(lengthModel), "wrap");


        // --- Target apogee spinner ---
        panel.add(new JLabel("Target apogee:"));
        DoubleModel apogeeModel = new DoubleModel(
            ext, "TargetApogee", UnitGroup.UNITS_LENGTH, ext.getTargetApogee()
        );
        JSpinner apogeeSpinner = new JSpinner(apogeeModel.getSpinnerModel());
        apogeeSpinner.setEditor(new SpinnerEditor(apogeeSpinner));
        panel.add(apogeeSpinner);
        panel.add(new UnitSelector(apogeeModel), "wrap");

        // --- Deploy altitude threshold spinner ---
        panel.add(new JLabel("Deploy altitude threshold:"));
        DoubleModel threshModel = new DoubleModel(
            ext, "DeployAltitudeThreshold", UnitGroup.UNITS_LENGTH, ext.getDeployAltitudeThreshold()
        );
        JSpinner threshSpinner = new JSpinner(threshModel.getSpinnerModel());
        threshSpinner.setEditor(new SpinnerEditor(threshSpinner));
        panel.add(threshSpinner);
        panel.add(new UnitSelector(threshModel), "wrap");

        // --- Max Mach for deployment spinner ---
        panel.add(new JLabel("Max Mach for deployment:"));
        DoubleModel maxMachModel = new DoubleModel(
            ext, "MaxMachForDeployment", UnitGroup.UNITS_NONE, ext.getMaxMachForDeployment()
        );
        JSpinner maxMachSpinner = new JSpinner(maxMachModel.getSpinnerModel());
        maxMachSpinner.setEditor(new SpinnerEditor(maxMachSpinner));
        panel.add(maxMachSpinner, "wrap");

        return panel;
    }
}
