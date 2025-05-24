package com.airbrakesplugin;

import javax.swing.*;
import javax.swing.event.DocumentEvent;
import javax.swing.event.DocumentListener;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
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

        // --- CSV path chooser ---
        panel.add(new JLabel("CFD data CSV:"));
        JTextField pathField = new JTextField(ext.getCfdDataFilePath());
        pathField.getDocument().addDocumentListener(new DocumentListener() {
            @Override public void insertUpdate(DocumentEvent e) {
                ext.setCfdDataFilePath(pathField.getText());
            }
            @Override public void removeUpdate(DocumentEvent e) {
                ext.setCfdDataFilePath(pathField.getText());
            }
            @Override public void changedUpdate(DocumentEvent e) {
                ext.setCfdDataFilePath(pathField.getText());
            }
        });
        panel.add(pathField, "span 1, growx");

        JButton browse = new JButton("Browse...");
        browse.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                JFileChooser chooser = new JFileChooser();
                chooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
                if (chooser.showOpenDialog(panel) == JFileChooser.APPROVE_OPTION) {
                    String p = chooser.getSelectedFile().getAbsolutePath();
                    pathField.setText(p);
                    ext.setCfdDataFilePath(p);
                }
            }
        });
        panel.add(browse, "wrap");

        // --- Surface area spinner ---
        panel.add(new JLabel("Brake area (m²):"));
        DoubleModel areaModel = new DoubleModel(
            ext, "AirbrakeSurfaceArea", UnitGroup.UNITS_AREA, 0.0
        );
        JSpinner areaSpinner = new JSpinner(areaModel.getSpinnerModel());
        areaSpinner.setEditor(new SpinnerEditor(areaSpinner));
        panel.add(areaSpinner);

        UnitSelector units = new UnitSelector(areaModel);
        panel.add(units, "wrap");

        return panel;
    }
}
