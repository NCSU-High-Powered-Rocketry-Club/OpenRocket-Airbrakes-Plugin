package com.airbrakesplugin;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.awt.event.ItemEvent;

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

    // CFD data CSV path chooser
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

    // Airbrake reference area
    panel.add(new JLabel("Airbrake area:"));
    final DoubleModel areaModel = new DoubleModel(ext, "ReferenceArea", UnitGroup.UNITS_AREA, 0);
    final JSpinner areaSpinner = new JSpinner(areaModel.getSpinnerModel());
    
    areaSpinner.setEditor(new SpinnerEditor(areaSpinner));
    panel.add(areaSpinner);
    panel.add(new UnitSelector(areaModel), "wrap");

    // Reference length (characteristic length)
    panel.add(new JLabel("Reference length:"));
    final DoubleModel lengthModel = new DoubleModel(ext, "ReferenceLength", UnitGroup.UNITS_DISTANCE, 0);
    final JSpinner lengthSpinner = new JSpinner(lengthModel.getSpinnerModel());
    
    lengthSpinner.setEditor(new SpinnerEditor(lengthSpinner));
    panel.add(lengthSpinner);
    panel.add(new UnitSelector(lengthModel), "wrap");

    // Target apogee
    panel.add(new JLabel("Target apogee:"));
    final DoubleModel apogeeModel = new DoubleModel(ext, "TargetApogee", UnitGroup.UNITS_DISTANCE, 0);
    final JSpinner apogeeSpinner = new JSpinner(apogeeModel.getSpinnerModel());
       
    apogeeSpinner.setEditor(new SpinnerEditor(apogeeSpinner));
    panel.add(apogeeSpinner);
    panel.add(new UnitSelector(apogeeModel), "wrap");

    // Max Mach for deployment
    panel.add(new JLabel("Max Mach for deployment:"));
    final DoubleModel maxMachModel = new DoubleModel(ext, "MaxMachForDeployment", UnitGroup.UNITS_COEFFICIENT, 0);
    final JSpinner maxMachSpinner = new JSpinner(maxMachModel.getSpinnerModel());
    
    maxMachSpinner.setEditor(new SpinnerEditor(maxMachSpinner));
    panel.add(maxMachSpinner, "wrap");

    // Apogee tolerance (±)
    panel.add(new JLabel("Apogee tolerance (±):"));
    final DoubleModel tolDoubleModel = new DoubleModel(ext, "ApogeeToleranceMeters", UnitGroup.UNITS_DISTANCE, 0);
    final JSpinner tolSpinner = new JSpinner(tolDoubleModel.getSpinnerModel());
       
    tolSpinner.setEditor(new SpinnerEditor(tolSpinner));
    panel.add(tolSpinner);
    panel.add(new UnitSelector(tolDoubleModel), "wrap");

    // === Burnout-only deployment (GUI) ===
    {
        JPanel burn = new JPanel(new MigLayout("insets 8, fillx", "[right]10[grow,fill][right]10[120!]"));
        burn.setBorder(new TitledBorder("Burnout-only deployment"));

        JCheckBox cbBurnOnly = new JCheckBox("Deploy airbrakes only after motor burnout");
        cbBurnOnly.setToolTipText("When checked, ignores apogee prediction and deploys fully a set time after burnout.");
        cbBurnOnly.setSelected(ext.isDeployAfterBurnoutOnly());

        SpinnerNumberModel delayModel = new SpinnerNumberModel(
            Math.max(0.0, ext.getDeployAfterBurnoutDelayS()), // value
            0.0,                                             // min
            60.0,                                            // max
            0.1                                              // step
        );
        JSpinner spDelay = new JSpinner(delayModel);
        ((JSpinner.DefaultEditor) spDelay.getEditor()).getTextField().setColumns(6);
        spDelay.setEnabled(ext.isDeployAfterBurnoutOnly());

        cbBurnOnly.addItemListener(e -> {
        boolean on = (e.getStateChange() == ItemEvent.SELECTED);
        ext.setDeployAfterBurnoutOnly(on);
        spDelay.setEnabled(on);
        });

        spDelay.addChangeListener(e -> {
        double v = ((Number) spDelay.getValue()).doubleValue();
        ext.setDeployAfterBurnoutDelayS(Math.max(0.0, v));
        });

        burn.add(cbBurnOnly, "span 3, wrap");
        burn.add(new JLabel("Delay after burnout:"));
        burn.add(spDelay, "growx");
        burn.add(new JLabel("s"), "wrap");

        panel.add(burn, "span 3, growx, wrap");
    }

    // Debug group
    final JPanel dbg = new JPanel(new MigLayout("insets 8, wrap 2", "[right]10[grow,fill]"));
    dbg.setBorder(new TitledBorder("Debug"));

    // Enable debug
    final JCheckBox cbEnable = new JCheckBox("Enable debug mode");
    cbEnable.setSelected(ext.isDebugEnabled());
    cbEnable.addItemListener(e -> ext.setDebugEnabled(e.getStateChange() == ItemEvent.SELECTED));
    dbg.add(cbEnable, "span 2");

    // Always-open override
    final JCheckBox cbAlways = new JCheckBox("Always-open override for airbrakes");
    cbAlways.setSelected(ext.isDbgAlwaysOpen());
    cbAlways.addItemListener(e -> ext.setDbgAlwaysOpen(e.getStateChange() == ItemEvent.SELECTED));
    dbg.add(cbAlways, "span 2");

    // Forced deploy fraction (0..1)
    dbg.add(new JLabel("Forced deploy fraction (0–1):"));
    final JSpinner spFrac = new JSpinner(new SpinnerNumberModel(ext.getDbgForcedDeployFrac(), 0.0, 1.0, 0.01));
    spFrac.addChangeListener(e -> {
        Object v = spFrac.getValue();
        double d = (v instanceof Number) ? ((Number) v).doubleValue() : 0.0;
        if (d < 0) d = 0; else if (d > 1) d = 1;
        ext.setDbgForcedDeployFrac(d);
    });
    dbg.add(spFrac, "growx");

    // Trace predictor
    final JCheckBox cbTP = new JCheckBox("Trace apogee predictor");
    cbTP.setSelected(ext.isDbgTracePredictor());
    cbTP.addItemListener(e -> ext.setDbgTracePredictor(e.getStateChange() == ItemEvent.SELECTED));
    dbg.add(cbTP, "span 2");

    // Trace controller
    final JCheckBox cbTC = new JCheckBox("Trace controller");
    cbTC.setSelected(ext.isDbgTraceController());
    cbTC.addItemListener(e -> ext.setDbgTraceController(e.getStateChange() == ItemEvent.SELECTED));
    dbg.add(cbTC, "span 2");

    // Write CSV traces
    final JCheckBox cbCsv = new JCheckBox("Write CSV traces");
    cbCsv.setSelected(ext.isDbgWriteCsv());
    cbCsv.addItemListener(e -> ext.setDbgWriteCsv(e.getStateChange() == ItemEvent.SELECTED));
    dbg.add(cbCsv, "span 2");

    // CSV directory
    dbg.add(new JLabel("CSV directory (optional):"));
    final JTextField tfCsvDir = new JTextField(ext.getDbgCsvDir());
    tfCsvDir.getDocument().addDocumentListener(new DocumentListener() {
        @Override public void insertUpdate(DocumentEvent e) { ext.setDbgCsvDir(tfCsvDir.getText()); }
        @Override public void removeUpdate(DocumentEvent e) { ext.setDbgCsvDir(tfCsvDir.getText()); }
        @Override public void changedUpdate(DocumentEvent e) { ext.setDbgCsvDir(tfCsvDir.getText()); }
    });
    dbg.add(tfCsvDir, "growx");

    // Live console
    final JCheckBox cbConsole = new JCheckBox("Show live debug console");
    cbConsole.setSelected(ext.isDbgShowConsole());
    cbConsole.addItemListener(e -> ext.setDbgShowConsole(e.getStateChange() == ItemEvent.SELECTED));
    dbg.add(cbConsole, "span 2");

    // Add debug group to the main panel (span the three layout columns)
    panel.add(dbg, "span 3, growx");

    return panel;
    }
}
