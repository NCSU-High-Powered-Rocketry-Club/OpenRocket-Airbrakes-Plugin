package com.airbrakesplugin;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;

import info.openrocket.core.document.Simulation;
import info.openrocket.core.plugin.Plugin;
import info.openrocket.core.unit.UnitGroup;

import info.openrocket.swing.gui.SpinnerEditor;
import info.openrocket.swing.gui.adaptors.DoubleModel;
import info.openrocket.swing.gui.components.UnitSelector;
import info.openrocket.swing.simulation.extension.AbstractSwingSimulationExtensionConfigurator;

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
        final JTextField pathField = new JTextField(ext.getCfdDataFilePath());
        pathField.getDocument().addDocumentListener(new DocumentListener() {
            @Override public void insertUpdate(DocumentEvent e) { ext.setCfdDataFilePath(pathField.getText()); }
            @Override public void removeUpdate(DocumentEvent e) { ext.setCfdDataFilePath(pathField.getText()); }
            @Override public void changedUpdate(DocumentEvent e) { ext.setCfdDataFilePath(pathField.getText()); }
        });
        panel.add(pathField, "growx");

        final JButton browse = new JButton("Browse...");
        browse.addActionListener(e -> {
            final JFileChooser chooser = new JFileChooser();
            chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
            if (chooser.showOpenDialog(panel) == JFileChooser.APPROVE_OPTION) {
                final String p = chooser.getSelectedFile().getAbsolutePath();
                pathField.setText(p);
                ext.setCfdDataFilePath(p);
            }
        });
        panel.add(browse, "wrap");

        // --- Airbrake reference area ---
        panel.add(new JLabel("Airbrake area:"));
        final DoubleModel areaModel =
                new DoubleModel(ext, "ReferenceArea", UnitGroup.UNITS_AREA, 0);
        final JSpinner areaSpinner = new JSpinner(areaModel.getSpinnerModel());
        areaSpinner.setEditor(new SpinnerEditor(areaSpinner));
        panel.add(areaSpinner);
        panel.add(new UnitSelector(areaModel), "wrap");

        // --- Reference length (characteristic length) ---
        panel.add(new JLabel("Reference length:"));
        final DoubleModel lengthModel =
                new DoubleModel(ext, "ReferenceLength", UnitGroup.UNITS_DISTANCE, 0);
        final JSpinner lengthSpinner = new JSpinner(lengthModel.getSpinnerModel());
        lengthSpinner.setEditor(new SpinnerEditor(lengthSpinner));
        panel.add(lengthSpinner);
        panel.add(new UnitSelector(lengthModel), "wrap");

        // --- Target apogee ---
        panel.add(new JLabel("Target apogee:"));
        final DoubleModel apogeeModel =
                new DoubleModel(ext, "TargetApogee", UnitGroup.UNITS_DISTANCE, 0);
        final JSpinner apogeeSpinner = new JSpinner(apogeeModel.getSpinnerModel());
        apogeeSpinner.setEditor(new SpinnerEditor(apogeeSpinner));
        panel.add(apogeeSpinner);
        panel.add(new UnitSelector(apogeeModel), "wrap");

        // --- Deploy altitude threshold ---
        panel.add(new JLabel("Deploy altitude threshold:"));
        final DoubleModel threshModel =
                new DoubleModel(ext, "DeployAltitudeThreshold", UnitGroup.UNITS_DISTANCE, 0);
        final JSpinner threshSpinner = new JSpinner(threshModel.getSpinnerModel());
        threshSpinner.setEditor(new SpinnerEditor(threshSpinner));
        panel.add(threshSpinner);
        panel.add(new UnitSelector(threshModel), "wrap");

        // --- Max Mach for deployment (dimensionless) ---
        panel.add(new JLabel("Max Mach for deployment:"));
        final DoubleModel maxMachModel =
                new DoubleModel(ext, "MaxMachForDeployment", UnitGroup.UNITS_COEFFICIENT, 0);
        final JSpinner maxMachSpinner = new JSpinner(maxMachModel.getSpinnerModel());
        maxMachSpinner.setEditor(new SpinnerEditor(maxMachSpinner));
        panel.add(maxMachSpinner, "wrap");

        return panel;
    }
}
