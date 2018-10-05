//
// Author: Azali Saudi
// Date Created : 30 Dec 2016
// Last Modified: 04 Jan 2017
//                03 Oct 2018
// Task: The Solver implementation i.e. SOR, AOR, etc.
//

import java.util.ArrayList;
import java.awt.image.*;
import javax.imageio.*;
import java.io.*;
import java.awt.Color;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Scanner;

public class Solver {
   static final int WALL_VALUE = 1;
   static final int GOAL_VALUE = 0;
   static final double FREE_VALUE = 1.1999999999999998;
//   static final double FREE_VALUE = 1.19;
   static final double BOUNDARY_VALUE = 1.2;

   static final double EPSILON = 1.0e-15;

   //Matrix variables
   double[] U;
   double[] V;
   int[] W;

   int Nx,Ny;

   public Queue<Point> path;

   public Solver(BufferedImage img, int gx, int gy) {
        Nx = img.getWidth();
        Ny = img.getHeight();

        U = new double[Nx*Ny];
        V = new double[Nx*Ny];
        W = new int[Nx*Ny];

        path = new LinkedList<Point>();

        //
        // Initialize the matrix U, V, W.
        //
        int rgb;
        for(int y = 0; y < Ny; y++)
        for(int x = 0; x < Nx; x++) {
            rgb = img.getRGB(x, y);
            rgb = rgb & 0x00FFFFFF;
            // The boundary wall
            if(rgb == 0) {
                U[x+y*Nx] = V[x+y*Nx] = BOUNDARY_VALUE;
                W[x+y*Nx] = WALL_VALUE;
            }

            // The goal point
            else if((x == gx) && (y == gy)) {
                U[x+y*Nx] = V[x+y*Nx] = GOAL_VALUE;
                W[x+y*Nx] = WALL_VALUE;
            }

            // The free space
            else { //if(rgb == 0xFFFFFF) {
                U[x+y*Nx] = V[x+y*Nx] = FREE_VALUE;
            }
        }

        // Make the 8 nyboring goal points
        int dP[][] = {{-1,0}, {1,0}, {0,-1}, {0,1}, {-1,-1}, {1,-1}, {-1,1}, {1,1}};
        for(int k = 0; k < 8; k++) {
            int x = gx + dP[k][0];
            int y = gy + dP[k][1];
            U[x+y*Nx] = V[x+y*Nx] = GOAL_VALUE;
            W[x+y*Nx] = WALL_VALUE;
        }
   }

   public void updateMatrix() {
        double[] Ut = U;
        U = V;
        V = Ut;
   }

   /**
    * Jacobi is very slow. We use it for benchmarking
    */
   public void doJacobi(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]);
            }
    }

   /**
    * Gauss-Seidel is much better than Jacobi.
    */
   public void doGS(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (V[x-1+y*Nx] + U[x+1+y*Nx] + V[x+(y-1)*Nx] + U[x+(y+1)*Nx]);
            }
    }

   /**
    * SOR is normally 80 - 90% than GS.
    */
   public void doSOR(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (V[x-1+y*Nx] + U[x+1+y*Nx] + V[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
            }
    }

   /**
    * AOR is slightly faster than SOR.
    */
   public void doAOR(double w, double r) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx] +
                            r*0.25 * (V[x-1+y*Nx] - U[x-1+y*Nx] + V[x+(y-1)*Nx] - U[x+(y-1)*Nx]);
            }
   }

   public boolean checkConverge() {
        double err = 0.0;
        int k = 0;
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(U[x+y*Nx] != V[x+y*Nx])
            if(W[x+y*Nx] != WALL_VALUE) {
                err += Math.abs(1.0-U[x+y*Nx]/V[x+y*Nx]);
                ++k;
            }
        if(k > 0) err /= (double)k;
        return (err < EPSILON);
   }

   public void runGDS(int x, int y) {
        int minx = x, miny = y;

        path.clear();
        while(true) {
            if(V[(x-1)+y*Nx] < V[minx+miny*Nx]) { minx = x-1; miny = y; }
            if(V[x+(y-1)*Nx] < V[minx+miny*Nx]) { minx = x; miny = y-1; }
            if(V[(x+1)+y*Nx] < V[minx+miny*Nx]) { minx = x+1; miny = y; }
            if(V[x+(y+1)*Nx] < V[minx+miny*Nx]) { minx = x; miny = y+1; }

            if(V[x-1+(y-1)*Nx] < V[minx+miny*Nx]) { minx = x-1; miny = y-1; }
            if(V[x+1+(y-1)*Nx] < V[minx+miny*Nx]) { minx = x+1; miny = y-1; }
            if(V[x-1+(y+1)*Nx] < V[minx+miny*Nx]) { minx = x-1; miny = y+1; }
            if(V[x+1+(y+1)*Nx] < V[minx+miny*Nx]) { minx = x+1; miny = y+1; }

            path.add(new Point(minx, miny));

            // Opps, we stuck
            if((minx == x) && (miny == y)) break;

            // The goal is found
            if(V[minx+miny*Nx] == GOAL_VALUE) break;

            x = minx; y = miny;
        }
   }

   public void printMatrix() {
        for(int y = 0; y < Ny; y++) {
            for(int x = 0; x < Nx; x++)
                System.out.print(String.format("%f ", U[x+y*Nx]));
            System.out.println();
        }
   }
   
   public void loadMatrix(String sf) {
	    try {
			FileReader fin = new FileReader(sf);
			Scanner scan = new Scanner(fin);
			
			int k = 0;
			while(scan.hasNext())
			{
				double v = scan.nextDouble();
				U[k] = V[k] = v;
				k++;
			}
			
			fin.close();
		}
		catch (Exception e) {
			e.printStackTrace();
		}		
   }

   public void saveMatrix(String sf) {
	    try {
			PrintWriter out = new PrintWriter(sf);
			for(int y = 0; y < Ny; y++) {
				for(int x = 0; x < Nx; x++)
					out.print(String.format("%.16f ", U[x+y*Nx]));
				out.println();
			}
			out.close();
			System.out.println("Saving file " + sf + " done!");
		}
		catch (Exception e) {
			e.printStackTrace();
		}		
   }
}
