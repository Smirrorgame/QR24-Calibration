package robprakt.graphics;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JPanel;
import javax.swing.JTextPane;

import org.apache.commons.math3.linear.RealMatrix;

import robCalibration.QR24;
import robCalibration.TestDataGenerator;
import robprakt.Constants;

public class CalibrationMenu extends JPanel{
			
	/**
	 * calibration object
	 */
	private QR24 calibration;
	
	/**
	 * Button for setting new value for number of measurements [QR24]
	 */
	private JButton selectRobotData, selectTrackingData;
		
	/**
	 * Buttons for starting calibration sequence.
	 */
	private JButton btnCalR1;
	
	private JFileChooser fileChooser;
	
	private List<String> robotMatrices = new ArrayList<String>();
	private List<String> trackingMatrices = new ArrayList<String>();
	private JTextPane trackerPane, robotPane;
	
	
	public CalibrationMenu(QR24 cal) {
		this.calibration = cal;
		
		//#########################
		//########COMPONENTS#######
		//#########################
		
		//creating grid for calibration-tab
		this.setLayout(new GridLayout(3,1));
		
		//creating rows for the grid with GridBagLayout
		JPanel row1 = new JPanel(new GridBagLayout());
		JPanel row2 = new JPanel(new GridLayout());
		JPanel row3 = new JPanel(new GridBagLayout());
		
		//adding rows to CalibrationMenu
		this.add(row1);
		this.add(row2);
		this.add(row3);
		
		//defining GridBagLayout
		Insets insets = new Insets(10,10,10,10);
		Dimension txtDim = new Dimension(Constants.mainFrameWidth/4,Constants.mainFrameHeight/4);
		
		robotPane = new JTextPane();
		robotPane.setText("Hier erscheint Effektor zu Marker");
		robotPane.setEditable(false);
		robotPane.setPreferredSize(txtDim);
		robotPane.setFont(new Font("Arial", Font.BOLD, 22));
		GridBagConstraints gbc_textPane = new GridBagConstraints();
		gbc_textPane.fill = GridBagConstraints.VERTICAL;
		gbc_textPane.anchor = GridBagConstraints.WEST;
		gbc_textPane.insets = new Insets(10,10,0,0);
		gbc_textPane.gridx = 0;
		gbc_textPane.gridy = 0;
		row2.add(robotPane, gbc_textPane);
		
		trackerPane = new JTextPane();
		trackerPane.setText("Hier erscheint Roboter zu Tracking");
		trackerPane.setEditable(false);
		trackerPane.setPreferredSize(txtDim);
		trackerPane.setFont(new Font("Arial", Font.BOLD, 22));
		GridBagConstraints gbc_textPane_1 = new GridBagConstraints();
		gbc_textPane_1.fill = GridBagConstraints.VERTICAL;
		gbc_textPane_1.anchor = GridBagConstraints.EAST;
		gbc_textPane_1.insets = new Insets(10,10,0,0);
		gbc_textPane_1.gridx = 1;
		gbc_textPane_1.gridy = 0;
		row2.add(trackerPane, gbc_textPane_1);
		
		
		
		
		//buttons for calibration START
		//cutter-robot
		Dimension btnCalDim = new Dimension(Constants.mainFrameHeight/3,Constants.mainFrameHeight/10);
		GridBagConstraints btnCalGBS1 = new GridBagConstraints();
		btnCalGBS1.gridx = 0;
		btnCalGBS1.gridy = 0;
		btnCalGBS1.insets = insets;
				
		//creating buttons for starting calibration of robots
		btnCalR1 = new JButton("<html><center><b>START CALIBRATION</b></center></html>");
		btnCalR1.setPreferredSize(btnCalDim);
		btnCalR1.setFont(new Font("Arial", Font.PLAIN, 15));
		

		//creating button for selecting files
		GridBagConstraints selRobGB = new GridBagConstraints();
		selRobGB.gridx = 2;
		selRobGB.gridy = 0;
		
		selectRobotData = new JButton("<html><center>Select Robot Data File<br><b>CURRENT File:<br>none</b></center></html>");
		selectRobotData.setPreferredSize(btnCalDim);
		selectRobotData.setFont(new Font("Arial", Font.PLAIN, 15));
		
		GridBagConstraints selTrackGB = new GridBagConstraints();
		selTrackGB.gridx = 4;
		selTrackGB.gridy = 0;
		
		selectTrackingData = new JButton("<html><center>Select Tracking Data File<br><b>CURRENT File:<br>none</b></center></html>");
		selectTrackingData.setPreferredSize(btnCalDim);
		selectTrackingData.setFont(new Font("Arial", Font.PLAIN, 15));
		
		//adding buttons to window
		row1.add(selectTrackingData,selTrackGB);
		row1.add(selectRobotData,selRobGB);
		row3.add(btnCalR1,btnCalGBS1);
				
		manageListeners();
		
		fileChooser = new JFileChooser(new File("."));
		fileChooser.setLocale(Locale.GERMANY);

	}
	
	//listener for button to select robot file
	private void manageListeners() {
		ActionListener robotListener = new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				
				int choice = fileChooser.showOpenDialog(null);
				if(choice == JFileChooser.APPROVE_OPTION) {
					try {
						String fileName = fileChooser.getSelectedFile().getName();
						selectRobotData.setText("<html><center>Select Robot Data File<br><b>CURRENT File:<br>"+fileName+"</b></center></html>");
						robotMatrices = Files.readAllLines(fileChooser.getSelectedFile().toPath());
						System.out.println("Matrizen vom Roboter geladen");
					}catch (IOException er) {
						er.printStackTrace();
					}
				}else {
				   System.err.println("No File Chosen!");
				   return;
			   }
			}
		};
		
		ActionListener trackingListener = new ActionListener() {
			
			@Override
			public void actionPerformed(ActionEvent e) {
				
				int choice = fileChooser.showOpenDialog(null);
				if(choice == JFileChooser.APPROVE_OPTION) {
					try {
						String fileName = fileChooser.getSelectedFile().getName();
						selectTrackingData.setText("<html><center>Select Tracking Data File<br><b>CURRENT File:<br>"+fileName+"</b></center></html>");
						trackingMatrices = Files.readAllLines(fileChooser.getSelectedFile().toPath());
						System.out.println("Matrizen vom Trackingsystem geladen");
					}catch (IOException er) {
						er.printStackTrace();
					}
				}else {
				   System.err.println("No File Chosen!");
				   return;
			   }
			}
		};
		
		//starting calibration process for cutter-robot
		ActionListener calibrationListener = new ActionListener() {

			@Override
			public void actionPerformed(ActionEvent e) {
				if(robotMatrices.size()==0 || trackingMatrices.size()==0) {
					System.out.println("Zuerst Bitte Dateien mit Werten Laden!");
					return;
				}else {
					try {
						calibration.genMatrices(robotMatrices, trackingMatrices);
						RealMatrix[] XY = calibration.calibrate();
						System.out.println("Effector to Marker: ");
						calibration.printTable(XY[0]);
						System.out.println("Robot to Tracking: ");
						calibration.printTable(XY[1]);
						robotPane.setText(matToString(XY[0]));
						trackerPane.setText(matToString(XY[1]));
					} catch (Exception e1) {
						e1.printStackTrace();
					}
				}
			}
		};
		
		btnCalR1.addActionListener(calibrationListener);
		selectRobotData.addActionListener(robotListener);
		selectTrackingData.addActionListener(trackingListener);
	}
	
	private String matToString(RealMatrix m) {
		double[][] data = m.getData();
		DecimalFormat df = new DecimalFormat("0.####");
		DecimalFormatSymbols dfs = DecimalFormatSymbols.getInstance();
		dfs.setDecimalSeparator('.');
		df.setDecimalFormatSymbols(dfs);
		String s = "";
		for (int row = 0; row < data.length; row++) {
			for (int col = 0; col < data[row].length; col++) {
				String val = df.format(data[row][col]);
				s+=val+" ";
			}
			s+="\n";
		}
		return s;
	}
	
}
