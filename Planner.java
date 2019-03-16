//
// Author: Azali Saudi
// Date Created : 30 Dec 2016
// Last Modified: 21 Jan 2019
// Task: The GUI for Path Planning of Agent Navigation
//

import java.awt.*;
import java.awt.event.*;
import java.applet.*;
import javax.swing.*;
import javax.swing.event.*;
import java.util.*;
import java.io.*;
import javax.imageio.*;
import java.awt.image.*;
import java.util.concurrent.TimeUnit;
import java.awt.geom.Ellipse2D;
import java.util.Timer;
import java.util.TimerTask;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Queue;
import javax.swing.JOptionPane;

public class Planner extends JFrame implements ActionListener {
    //Display parameters
    public static final int Width = 600;
    public static final int Height = 300;

    //Menu Options
    public static final String LOAD_MAP = "Load Map...";
    public static final String RUN_ITER = "Run";
    public static final String RUN_GDS  = "GDS";
    public static final String SAVE_MAP = "Save Map...";
    public static final String SAVE_MATRIX = "Save Matrix...";
    public static final String LOAD_MATRIX = "Load Matrix...";

    //GUI Widgets
    public static JLabel label;
    public static JTextField tfMethod;
    public static JTextField tfStartX;
    public static JTextField tfStartY;
    public static JTextField tfGoalX;
    public static JTextField tfGoalY;
    public static JTextField tfW1;
    public static JTextField tfW2;
    public static JTextField tfR1;
    public static JTextField tfR2;
    public static JTextField tfR3;
    public static JTextField tfR4;
    public static JScrollPane spNote;
    public static JTextArea taNote;
    public static Map canvas;
    public static JMenuBar menu;
    public static JMenu fileMenu;
    public static BufferedImage mapImage;
    public static Timer timer;
    public static Queue<Point> Q;
    public static String fileName;
    public static JScrollPane scroll;

    //The solver
    public Solver solver;
    public Thread iteratorThread;
    public boolean isInitialized = false;

    public Planner() {
        setTitle("Planner");
        setLayout(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        Container content = getContentPane();

        menu = new JMenuBar();
        fileMenu = new JMenu("File");
        fileMenu.addActionListener(this);
        fileMenu.add(LOAD_MAP).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add(RUN_ITER).addActionListener(this);
        fileMenu.add(RUN_GDS ).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add(SAVE_MAP).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add(SAVE_MATRIX).addActionListener(this);
        fileMenu.add(LOAD_MATRIX).addActionListener(this);
        fileMenu.addSeparator();
        fileMenu.add("Exit").addActionListener((ActionEvent event) -> { System.exit(0); });
        menu.add(fileMenu);

        menu.setBounds(0, 0, Width, 20);
        content.add(menu);

        label = new JLabel("0");
        label.setBounds(10,20, 400,30);
        content.add(label);

        canvas = new Map();
        scroll = new JScrollPane(canvas);
        scroll.setBounds(10, 50, Width-40, Height);
        content.add(scroll);

        tfMethod = new JTextField("KSOR");
        tfMethod.setBounds(200,Height+60, 80,25);
        content.add(tfMethod);

        tfStartX = new JTextField("25");
        tfStartX.setBounds(200,Height+85, 39,25);
        content.add(tfStartX);
        tfStartY = new JTextField("246");
        tfStartY.setBounds(241,Height+85, 39,25);
        content.add(tfStartY);
        tfGoalX = new JTextField("148");
        tfGoalX.setBounds(200,Height+110, 39,25);
        content.add(tfGoalX);
        tfGoalY = new JTextField("146");
        tfGoalY.setBounds(241,Height+110, 39,25);
        content.add(tfGoalY);
        tfW1 = new JTextField("-2.18"); // SOR: 1.80
        tfW1.setBounds(10,Height+60, 50,25);
        content.add(tfW1);
        tfW2 = new JTextField("1.82");
        tfW2.setBounds(10,Height+85, 50,25);
        content.add(tfW2);

        tfR1 = new JTextField("1.84");
        tfR1.setBounds(80,Height+60, 50,25);
        content.add(tfR1);
        tfR2 = new JTextField("1.86");
        tfR2.setBounds(80,Height+85, 50,25);
        content.add(tfR2);
        tfR3 = new JTextField("1.95");
        tfR3.setBounds(80,Height+110,50,25);
        content.add(tfR3);
        tfR4 = new JTextField("1.96");
        tfR4.setBounds(80,Height+135,50,25);
        content.add(tfR4);

        taNote = new JTextArea("");
        JScrollPane spNote = new JScrollPane(taNote);
        spNote.setBounds(300,Height+60, 270,96);
        content.add(spNote);

        fileName = "case08.png";
        taNote.append(fileName + "\n");
        mapImage = new BufferedImage(1, 1, BufferedImage.TYPE_INT_RGB);

        setSize(Width, Height+200);
        setVisible(true);
    }

    public void init() {
        try {
            mapImage = ImageIO.read(new File(fileName));
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        repaint();
    }

    class Map extends JPanel {
        public Map() {
            addMouseListener(new MyMouse());
        }

        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            int x, y;
            Ellipse2D.Double circle;

            g.drawImage(mapImage, 0, 0, null);

            Graphics2D g2d = (Graphics2D)g;

            x = Integer.parseInt(tfStartX.getText());
            y = Integer.parseInt(tfStartY.getText());
            circle = new Ellipse2D.Double(x-4, y-4, 8.0, 8.0);
            g.setColor(Color.RED);
            g2d.fill(circle);

            x = Integer.parseInt(tfGoalX.getText());
            y = Integer.parseInt(tfGoalY.getText());
            circle = new Ellipse2D.Double(x-4, y-4, 8.0, 8.0);
            g.setColor(Color.GREEN);
            g2d.fill(circle);
        }

        public Dimension getPreferredSize() {
            return new Dimension(mapImage.getWidth()+400, mapImage.getHeight()+400);
        }

        private class MyMouse extends MouseAdapter {
            public void mousePressed(MouseEvent evt) {
                int x = evt.getX();
                int y = evt.getY();
                if(SwingUtilities.isLeftMouseButton(evt)) {
                    tfStartX.setText(Integer.toString(x));
                    tfStartY.setText(Integer.toString(y));
                }
                else
                if(SwingUtilities.isRightMouseButton(evt)) {
                    tfGoalX.setText(Integer.toString(x));
                    tfGoalY.setText(Integer.toString(y));
                }
                repaint();
            }
        }
    }

    public void actionPerformed(ActionEvent evt) {
        String str = evt.getActionCommand();
        if (str.equals(LOAD_MAP)) {
            try {
                JFileChooser chooser = new JFileChooser(new File(".").getCanonicalPath());
                int returnVal = chooser.showOpenDialog(null);
                if(returnVal == JFileChooser.APPROVE_OPTION) {
                    File fout = chooser.getSelectedFile();
                    fileName = fout.getName();
                    mapImage = ImageIO.read(new File(fileName));
                    taNote.append(fileName + "\n");
                    canvas.repaint();
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
        else if (str.equals(RUN_ITER)) {
            init();   // Load map file
            canvas.repaint();

//            runIter();

            iteratorThread = new Thread(new Iterator());
            iteratorThread.start();
        }
        else if (str.equals(RUN_GDS)) {
            int x = Integer.parseInt(tfStartX.getText());
            int y = Integer.parseInt(tfStartY.getText());
            solver.runGDS(x, y);
            timer = new Timer();
            timer.schedule(new RemindTask(),
                           0,     // initial delay
                           20);   // subsequent rate in ms
        }
        else if (str.equals(SAVE_MAP)) {
            try {
                JFileChooser chooser = new JFileChooser(new File(".").getCanonicalPath());
                int returnVal = chooser.showSaveDialog(null);
                if(returnVal == JFileChooser.APPROVE_OPTION) {
                    File fout = chooser.getSelectedFile();
                    String saveFilename = fout.getName();
                    int wi = mapImage.getWidth();
                    int hi = mapImage.getHeight();
                    BufferedImage bi = new BufferedImage(wi, hi, BufferedImage.TYPE_INT_RGB);
                    canvas.paint(bi.getGraphics());
                    ImageIO.write(bi, "png", new File(saveFilename));
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
        else if (str.equals(SAVE_MATRIX)) {
            try {
                JFileChooser chooser = new JFileChooser(new File(".").getCanonicalPath());
                int returnVal = chooser.showSaveDialog(null);
                if(returnVal == JFileChooser.APPROVE_OPTION) {
                    File fout = chooser.getSelectedFile();
                    String fname = fout.getName();

                    int gx = Integer.parseInt(tfGoalX.getText());
                    int gy = Integer.parseInt(tfGoalY.getText());
                    init();
                    canvas.repaint();

                    if(isInitialized == false) {
                        solver = new Solver(mapImage, gx, gy);
                        isInitialized = true;
                    }
                    solver.saveMatrix(fname);
                    JOptionPane.showMessageDialog(null,
                                                  "Done",
                                                  "Save Matrix",
                                                  JOptionPane.INFORMATION_MESSAGE);
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
        else if (str.equals(LOAD_MATRIX)) {
            try {
                JFileChooser chooser = new JFileChooser(new File(".").getCanonicalPath());
                int returnVal = chooser.showOpenDialog(null);
                if(returnVal == JFileChooser.APPROVE_OPTION) {
                    File fout = chooser.getSelectedFile();
                    String fname = fout.getName();

                    int gx = Integer.parseInt(tfGoalX.getText());
                    int gy = Integer.parseInt(tfGoalY.getText());
                    solver = new Solver(mapImage, gx, gy);
                    init();
                    canvas.repaint();

                    solver.loadMatrix(fname);
                    JOptionPane.showMessageDialog(null,
                                                  "Done",
                                                  "Load Matrix",
                                                  JOptionPane.INFORMATION_MESSAGE);
                }
            }
            catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    public void displayLog(int iter, long elaps) {
        // Print out text area
        taNote.append("Iteration: " + Integer.toString(iter) + "\n");
        taNote.append(String.format("Elapsed: %d min, %d sec, %d ms\n",
                                   (elaps/1000) / 60,
                                   (elaps/1000) % 60,
                                   (elaps%60000) % 1000));
    }

    class RemindTask extends TimerTask {
        Point pt = new Point();
        Graphics2D g2D;

        RemindTask() {
            g2D = mapImage.createGraphics();
            g2D.setColor(Color.BLACK);
        }

        public void run() {
            pt = solver.path.remove();
            g2D.fill(new Ellipse2D.Double(pt.x, pt.y, 1.5, 1.5));
            canvas.repaint();
            if(solver.path.isEmpty()) timer.cancel();
        }
    }

    class Iterator implements Runnable {
        private void displayMeter(int it) {
            if((it % 1000) == 0) {
               label.setText(String.format("%d", it));
            }
        }
        public void run() {
            int gx = Integer.parseInt(tfGoalX.getText());
            int gy = Integer.parseInt(tfGoalY.getText());

            double w = Float.parseFloat(tfW1.getText());
            double ww= Float.parseFloat(tfW2.getText());

            double r = Float.parseFloat(tfR1.getText());
            double s = Float.parseFloat(tfR2.getText());
            double t = Float.parseFloat(tfR3.getText());
            double u = Float.parseFloat(tfR4.getText());

            int iteration = 0;
            boolean converge = false;

            long elapsedTime;
            long stopTime;
            long startTime = System.nanoTime();

            solver = new Solver(mapImage, gx, gy);
            isInitialized = true;

            // FULL-SWEEP CPU
            if (tfMethod.getText().toUpperCase().equals("GS")) {
                while(!converge) {
                    solver.doGS();
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> GS\n"));
            } else
            if (tfMethod.getText().toUpperCase().equals("SOR")) {
                while(!converge) {
                    solver.doSOR(w);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> SOR, w=%.2f\n", w));
            } else
            if (tfMethod.getText().toUpperCase().equals("AOR")) {
                while(!converge) {
                    solver.doAOR(w, r);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> AOR, w=%.2f, r=%.2f\n", w, r));
            } else
            if (tfMethod.getText().toUpperCase().equals("TOR")) {
                while(!converge) {
                    solver.doTOR(w, r, s);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> TOR, w=%.2f, r=%.2f, s=%.2f\n", w, r, s));
            } else
            if (tfMethod.getText().toUpperCase().equals("QOR")) {
                while(!converge) {
                    solver.doQOR(w, r, s, t, u);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> QOR, w=%.2f, r=%.2f, s=%.2f, t=%.2f, u=%.2f\n", w, r, s, t, u));
            } else





			// KSOR on CPU
            if (tfMethod.getText().toUpperCase().equals("KSOR")) {
                while(!converge) {
                    solver.doKSOR(w);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> KSOR, w=%.2f\n", w));
            } else
            // HSKSOR on CPU
            if (tfMethod.getText().toUpperCase().equals("HSKSOR")) {
                while(!converge) {
                    solver.doHSKSOR(w);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeHS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillHS();
                solver.updateMatrix();
                taNote.append(String.format(">>> HSKSOR, w=%.2f\n", w));
            } else
            // QSKSOR on CPU
            if (tfMethod.getText().toUpperCase().equals("QSKSOR")) {
                while(!converge) {
                    solver.doQSKSOR(w);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeQS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillQS();
                solver.updateMatrix();
                taNote.append(String.format(">>> QSKSOR, w=%.2f\n", w));
            } else

			//
			// KAOR -- CPU
			//
            if (tfMethod.getText().toUpperCase().equals("KAOR")) {
                while(!converge) {
                    solver.doKAOR(w, r);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> KAOR, w=%.2f, r=%.2f\n", w, r));
            } else



            // HALF-SWEEP CPU
            if (tfMethod.getText().toUpperCase().equals("HSSOR")) {
                while(!converge) {
                    solver.doHSSOR(w);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeHS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillHS();
                solver.updateMatrix();
                taNote.append(String.format(">>> HSSOR, w=%.2f\n", w));
            } else
            // QUARTER-SWEEP CPU
            if (tfMethod.getText().toUpperCase().equals("QSSOR")) {
                while(!converge) {
                    solver.doQSSOR(w);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeQS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillQS();
                solver.updateMatrix();
                taNote.append(String.format(">>> QSSOR, w=%.2f\n", w));
            } else

 



            // FULL-SWEEP MODIFIED CPU
            if (tfMethod.getText().toUpperCase().equals("MSOR")) {
                while(!converge) {
                    solver.doMSOR(w, ww);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> MSOR, w=%.2f, ww=%.2f\n", w, ww));
            } else
            if (tfMethod.getText().toUpperCase().equals("MAOR")) {
                while(!converge) {
                    solver.doMAOR(w, ww, r);
                    displayMeter(++iteration);
                    converge = solver.checkConverge();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                taNote.append(String.format(">>> MAOR, w=%.2f, ww=%.2f, r=%.2f\n", w, ww, r));
            } else


            // HALF-SWEEP MODIFIED CPU
            if (tfMethod.getText().toUpperCase().equals("HSMSOR")) {
                while(!converge) {
                    solver.doHSMSOR(w, ww);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeHS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillHS();
                solver.updateMatrix();
                taNote.append(String.format(">>> HSMSOR, w=%.2f, ww=%.2f\n", w, ww));
            } else


            // QUARTER-SWEEP MODIFIED CPU
            if (tfMethod.getText().toUpperCase().equals("QSMSOR")) {
                solver.doInitRB();
                while(!converge) {
                    solver.doQSMSOR(w, ww);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeQS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillQS();
                solver.updateMatrix();
                taNote.append(String.format(">>> QSMSOR, w=%.2fm ww=%.2f\n", w, ww));
            } else
            if (tfMethod.getText().toUpperCase().equals("QSMAOR")) {
                solver.doInitRB();
                while(!converge) {
                    solver.doQSMAOR(w, ww, r);
                    displayMeter(++iteration);
                    converge = solver.checkConvergeQS();
                    solver.updateMatrix();
                }
                label.setText(String.format("%d", iteration));
                solver.doFillQS();
                solver.updateMatrix();
                taNote.append(String.format(">>> QSMAOR, w=%.2fm ww=%.2f r=%.2f\n", w, ww, r));
            }


            // NONE OF THE ABOVE
            else {
                System.out.println("Iteration Method Not Found!!!");
            }

            stopTime = System.nanoTime();
            elapsedTime = TimeUnit.NANOSECONDS.toMillis(stopTime - startTime); // Total elapsed in ms
            displayLog(iteration, elapsedTime);
        }
    }

    public static void main(String[] args) {
        Planner a = new Planner();
        a.init();
    }
}

