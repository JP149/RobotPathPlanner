/**
 * EE631 Autonomous Mobile Robotics 
 * Professor Yi Guo
@Author Jaydeep Patel 2013
 **/
package Main;

import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;

@SuppressWarnings("serial")
public class Main extends JFrame implements ActionListener {
	class sim1Thread extends Thread {

		@Override
		public void run() {
			VelocityPlanningSimulation.NoVelocityPlanningMain.main(null);
		}
	}

	class sim2Thread extends Thread {

		@Override
		public void run() {
			VelocityPlanningSimulation.Main.main(null);
		}
	}

	class sim3Thread extends Thread {

		@Override
		public void run() {
			CoopFormationControlSimulation.Main.main(null);
		}
	}

	JMenuBar menuBar = new JMenuBar();
	JMenu menu = new JMenu("File");
	JMenu aboutMenu = new JMenu("About");

	JMenuItem sim1MenuItem = new JMenuItem("Run D* Lite Simulation");
	JMenuItem sim2MenuItem = new JMenuItem("Run Velocity Planning Simulation");
	JMenuItem sim3MenuItem = new JMenuItem("Run Formation Simulation");
	JMenuItem aboutMenuItem = new JMenuItem("About");

	public Main() {
		super();
		setTitle("Simulations");
		setMinimumSize(new Dimension(240, 100));
		setDefaultCloseOperation(EXIT_ON_CLOSE);
		initializeComponents();
	}

	private void initializeComponents() {
		sim1MenuItem.addActionListener(this);
		sim2MenuItem.addActionListener(this);
		sim3MenuItem.addActionListener(this);
		aboutMenuItem.addActionListener(this);
		menu.add(sim1MenuItem);
		menu.add(sim2MenuItem);
		menu.add(sim3MenuItem);
		aboutMenu.add(aboutMenuItem);
		menuBar.add(menu);
		menuBar.add(aboutMenu);
		this.setJMenuBar(menuBar);
		// frame.setSize(sim.map.Width, sim.map.Height);
		this.setVisible(true);
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
	}

	public static void main(String[] args) {
		new Main();
	}

	@Override
	public void actionPerformed(ActionEvent e) {
		Object source = e.getSource();
		if (source.equals(sim1MenuItem)) {
			(new sim1Thread()).start();
		} else if (source.equals(sim2MenuItem)) {
			(new sim2Thread()).start();
		} else if (source.equals(sim3MenuItem)) {
			(new sim3Thread()).start();
		} else if (source.equals(aboutMenuItem)) {
			JOptionPane.showMessageDialog(this,
					"Implemented by \nJaydeep Patel\nSpring 2013", "About",
					JOptionPane.INFORMATION_MESSAGE);
		}

	}
}
