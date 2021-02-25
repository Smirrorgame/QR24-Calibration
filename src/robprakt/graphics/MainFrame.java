package robprakt.graphics;

import java.awt.Color;
import java.awt.Toolkit;

import javax.swing.JFrame;
import javax.swing.JTabbedPane;

import robCalibration.QR24;
import robprakt.Constants;

public class MainFrame extends JFrame {
	
	/**
	 * Tabbed ContentPane for managing menus
	 */
	private JTabbedPane tabbedContentPane;
	
	/**
	 * Contains graphics structure for the calibration menu
	 */
	private CalibrationMenu calibrationMenu;
	
	/**
	 * calibration contains functions for calibrating robots
	 */
	private QR24 calibration;
		
	/**
	 * Create the main frame.
	 */
	public MainFrame(String title) {
		
		//settings for window
		setResizable(false);
		setTitle(title);
		setVisible(true);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(	(int) Toolkit.getDefaultToolkit().getScreenSize().getWidth()-Constants.mainFrameWidth,
					(int) Toolkit.getDefaultToolkit().getScreenSize().getHeight()/2-Constants.mainFrameHeight/2,
					Constants.mainFrameWidth, Constants.mainFrameHeight);

		
		//create calibration object
		calibration = new QR24();
		
		// tabbedPane as basic pane for navigating between menus
		tabbedContentPane = new JTabbedPane();
		tabbedContentPane.setBackground(Color.LIGHT_GRAY);
		setContentPane(tabbedContentPane);
		
		// creating container hierarchy for menus
		calibrationMenu = new CalibrationMenu(calibration);
		
		// adding menus to tabbedPane
		tabbedContentPane.add("Calibration",calibrationMenu);
		
		revalidate();
		
	}
}
