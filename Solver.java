//
// Author: Azali Saudi
// Date Created : 30 Dec 2016
// Last Modified: 21 Jan 2019
// Task: The Solver implementation i.e. SOR, AOR, TOR, QOR.
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
    static final double FREE_VALUE = 1.19;
    static final double BOUNDARY_VALUE = 1.2;

    static final double EPSILON = 1.0e-15;

    //Matrix variables
    double[] U;
    double[] V;
    int[] W;

    int[] RB;

    int Nx,Ny;

    public Queue<Point> path;

    public Solver(BufferedImage img, int gx, int gy) {
        Nx = img.getWidth();
        Ny = img.getHeight();

        U = new double[Nx*Ny];
        V = new double[Nx*Ny];
        W = new int[Nx*Ny];

        RB= new int[Nx*Ny];

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

        // Make the outer boundary walls
        for(int x = 0; x < Nx; x++) {
            U[x+0*Nx] = V[x+0*Nx] = BOUNDARY_VALUE;
            W[x+0*Nx] = WALL_VALUE;
            U[x+(Ny-1)*Nx] = V[x+(Ny-1)*Nx] = BOUNDARY_VALUE;
            W[x+(Ny-1)*Nx] = WALL_VALUE;
        }
        for(int y = 0; y < Ny; y++) {
            U[0+y*Nx] = V[0+y*Nx] = BOUNDARY_VALUE;
            W[0+y*Nx] = WALL_VALUE;
            U[(Nx-1)+y*Nx] = V[(Nx-1)+y*Nx] = BOUNDARY_VALUE;
            W[(Nx-1)+y*Nx] = WALL_VALUE;
        }
    }

    public void updateMatrix() {
/*
        double[] Ut = U;
        U = V;
        V = Ut;
*/
        for(int y = 0; y < Ny; y++)
        for(int x = 0; x < Nx; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                U[x+y*Nx] = V[x+y*Nx];
            }
    }

    //
    // The HALF-SWEEP iterative method.
    //
    public void doJacobi() {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]);
            }
    }

    public void doGS() {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (V[x-1+y*Nx] + U[x+1+y*Nx] + V[x+(y-1)*Nx] + U[x+(y+1)*Nx]);
            }
    }

    public void doSOR(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (V[x-1+y*Nx] + U[x+1+y*Nx] + V[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
            }
    }

    public void doAOR(double w, double r) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx] +
                            r*0.25 * (V[x-1+y*Nx] - U[x-1+y*Nx] + V[x+(y-1)*Nx] - U[x+(y-1)*Nx]);
            }
    }

    public void doTOR(double w, double r, double s) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx] +
                            r*0.25 * (V[x-1+y  *Nx] - U[x-1+y  *Nx])+
                            s*0.25 * (V[x+(y-1)*Nx] - U[x+(y-1)*Nx]);
            }
    }
    
    
    
    //
    // The KSOR method
    //     w = R - [-2, 0]; w < -2.0 && w > 0
    //
    public void doKSOR(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = (1./(1.+w)) * (w*0.25 * (V[x-1+y*Nx] + U[x+1+y*Nx] + V[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + U[x+y*Nx]);
                
				//V[x+y*Nx] = (1./(1.+w)) * U[x+y*Nx] + (w/(1.+w)) * 0.25 * (V[(x-1)+y*Nx] + V[x+(y-1)*Nx] + U[(x+1)+y*Nx] + U[x+(y+1)*Nx]);
                
            }
    }

    //
    // The HSKSOR method
    //
    public void doHSKSOR(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(((x+y) % 2) == 0)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = (1./(1.+w)) * (w*0.25 * (V[x-1+(y-1)*Nx] + V[x+1+(y-1)*Nx] + U[x-1+(y+1)*Nx] + U[x+1+(y+1)*Nx]) + U[x+y*Nx]);
            }
    }
    
    //
    // The QSKSOR method
    //
    public void doQSKSOR(double w) {
        for(int y = 2; y < Ny-2; y++)
        for(int x = 2; x < Nx-2; x++)
            if((x%2 == 0) && (y%2 == 0))
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = (1./(1.+w)) * (w*0.25 * (V[x-2+y*Nx] + U[x+2+y*Nx] + V[x+(y-2)*Nx] + U[x+(y+2)*Nx]) + U[x+y*Nx]);
            }
    }
    
    
    
    
    //
    //
    // The KAOR method
    //
    //
    public void doKAOR(double w, double r) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(W[x+y*Nx] != WALL_VALUE)
            {
				V[x+y*Nx] = ((1.+r-w)/(1.+r)) * U[x+y*Nx] + 
							(w /(1.+r)) * 0.25 * (U[(x-1)+y*Nx] + U[x+(y-1)*Nx] + U[(x+1)+y*Nx] + U[x+(y+1)*Nx]) +
							(r /(1.+r)) * 0.25 * (V[(x-1)+y*Nx] - U[(x-1)+y*Nx] + V[x+(y-1)*Nx] - U[x+(y-1)*Nx]);
            }
    }




    

    //
    // The HALF-SWEEP iterative method.
    //
    public void doHSSOR(double w) {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(((x+y) % 2) == 0)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (V[x-1+(y-1)*Nx] + V[x+1+(y-1)*Nx] + U[x-1+(y+1)*Nx] + U[x+1+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
            }
    }

    public void doFillHS() {
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(((x+y) % 2) == 1)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (V[x-1+y*Nx] + V[x+1+y*Nx] + V[x+(y-1)*Nx] + V[x+(y+1)*Nx]);
            }
    }

    //
    // The QUARTER-SWEEP iterative method.
    //
    public void doQSSOR(double w) {
        for(int y = 2; y < Ny-2; y++)
        for(int x = 2; x < Nx-2; x++)
            if((x%2 == 0) && (y%2 == 0))
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (V[x-2+y*Nx] + U[x+2+y*Nx] + V[x+(y-2)*Nx] + U[x+(y+2)*Nx]) + (1-w)*U[x+y*Nx];
            }
    }

    public void doFillQS() {
        // Compute WHITE Square, i.e. x and y are ODD
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x%2 == 1) && (y%2 == 1))
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (V[x-1+(y-1)*Nx] + V[x+1+(y-1)*Nx] + V[x-1+(y+1)*Nx] + V[x+1+(y+1)*Nx]);
            }

        // Compute WHITE Dot, i.e. x ODD and y EVEN or otherwise
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++) {
            if((x%2 == 1) && (y%2 == 1)) continue;
            if((x%2 == 0) && (y%2 == 0)) continue;
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = 0.25 * (V[x-1+y*Nx] + V[x+1+y*Nx] + V[x+(y-1)*Nx] + V[x+(y+1)*Nx]);
            }
        }
    }

    //
    // The Modified SOR
    //
    public void doMSOR(double w, double ww) {
        // Compute the RED nodes
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 0)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
            }

        // Compute the BLACK nodes
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 1)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = ww*0.25 * (V[x-1+y*Nx] + V[x+1+y*Nx] + V[x+(y-1)*Nx] + V[x+(y+1)*Nx]) + (1-ww)*U[x+y*Nx];
            }
    }

    public void doMAOR(double w, double ww, double r) {
        // Compute the RED nodes
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 0)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = ww*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-ww)*U[x+y*Nx];
            }

        // Compute the BLACK nodes
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 1)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] =  w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx] +
                             r*0.25 * (V[x-1+y*Nx] - U[x-1+y*Nx] + V[x+(y-1)*Nx] - U[x+(y-1)*Nx] +
                                       V[x+1+y*Nx] - U[x+1+y*Nx] + V[x+(y+1)*Nx] - U[x+(y+1)*Nx]);
            }
    }




    //
    // The HALF-SWEEP Modified SOR
    //
    public void doHSMSOR(double w, double ww) {
        // Compute the RED nodes
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x%2 == 1) && (y%2 == 1))
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+(y-1)*Nx] + U[x+1+(y-1)*Nx] + U[x-1+(y+1)*Nx] + U[x+1+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
            }

        // Compute the BLACK nodes
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x%2 == 0) && (y%2 == 0))
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = ww*0.25 * (V[x-1+(y-1)*Nx] + V[x+1+(y-1)*Nx] + V[x-1+(y+1)*Nx] + V[x+1+(y+1)*Nx]) + (1-ww)*U[x+y*Nx];
            }
    }


    //
    // The QUARTER-SWEEP Modified SOR.
    //
    public void doInitRB()
    {
        // Let all be WHITE nodes
        for(int y = 0; y < Ny; y++)
        for(int x = 0; x < Nx; x++)
            RB[x+y*Nx] = 0;

        for(int y = 2; y < Ny-2; y+=4)
        for(int x = 2; x < Nx-2; x+=4) {
            RB[x+y*Nx] = 1;
            RB[x+2+(y+2)*Nx] = 1;

            RB[x+2+y*Nx] = 2;
            RB[x+(y+2)*Nx] = 2;
        }
/*
        for(int y = 0; y < Ny; y++) {
            for(int x = 0; x < Nx; x++)
                System.out.print(String.format("%d", RB[x+y*Nx]));
            System.out.println();
        }
*/
    }

    public void doQSMSOR(double w, double ww) {
        // Compute the RED nodes
        for(int y = 2; y < Ny-2; y++)
        for(int x = 2; x < Nx-2; x++)
            if(RB[x+y*Nx] == 1)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-2+y*Nx] + U[x+2+y*Nx] + U[x+(y-2)*Nx] + U[x+(y+2)*Nx]) + (1-w)*U[x+y*Nx];
            }

        // Compute the BLACK nodes
        for(int y = 2; y < Ny-2; y++)
        for(int x = 2; x < Nx-2; x++)
            if(RB[x+y*Nx] == 2)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = ww*0.25 * (V[x-2+y*Nx] + V[x+2+y*Nx] + V[x+(y-2)*Nx] + V[x+(y+2)*Nx]) + (1-ww)*U[x+y*Nx];
            }
    }


    public void doQSMAOR(double w, double ww, double r) {
        // Compute the RED nodes
        for(int y = 2; y < Ny-2; y++)
        for(int x = 2; x < Nx-2; x++)
            if(RB[x+y*Nx] == 1)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = ww*0.25 * (U[x-2+y*Nx] + U[x+2+y*Nx] + U[x+(y-2)*Nx] + U[x+(y+2)*Nx]) + (1-ww)*U[x+y*Nx];
            }

        // Compute the BLACK nodes
        for(int y = 2; y < Ny-2; y++)
        for(int x = 2; x < Nx-2; x++)
            if(RB[x+y*Nx] == 2)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] =  w*0.25 * (U[x-2+y*Nx] + U[x+2+y*Nx] + U[x+(y-2)*Nx] + U[x+(y+2)*Nx]) + (1-w)*U[x+y*Nx] +
                             r*0.25 * (V[x-2+y*Nx] - U[x-2+y*Nx] + V[x+(y-2)*Nx] - U[x+(y-2)*Nx] +
                                       V[x+2+y*Nx] - U[x+2+y*Nx] + V[x+(y+2)*Nx] - U[x+(y+2)*Nx]);
            }
    }



    //
    // The novelty: Quad-Parameter Overrelaxation (also known as Red-Black QOR)
    //
    public void doQOR(double w, double r, double s, double t, double u) {
        int x,y;

        // The RED nodes
        for(y = 1; y < Ny-1; y++)
        for(x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 0)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx];
            }

        // The BLACK nodes
        for(y = 1; y < Ny-1; y++)
        for(x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 1)
            if(W[x+y*Nx] != WALL_VALUE)
            {
                V[x+y*Nx] = w*0.25 * (U[x-1+y*Nx] + U[x+1+y*Nx] + U[x+(y-1)*Nx] + U[x+(y+1)*Nx]) + (1-w)*U[x+y*Nx] +
                            r*0.25 * (V[x-1+y  *Nx] - U[x-1+y  *Nx])+
                            s*0.25 * (V[x+1+y  *Nx] - U[x+1+y  *Nx])+
                            t*0.25 * (V[x+(y-1)*Nx] - U[x+(y-1)*Nx])+
                            u*0.25 * (V[x+(y+1)*Nx] - U[x+(y+1)*Nx]);
            }
    }

    public boolean checkConverge() {
        double err = 0.0;
        int k = 0;
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if(U[x+y*Nx] != V[x+y*Nx])
            if(W[x+y*Nx] != WALL_VALUE) {
                err += Math.abs(1-U[x+y*Nx]/V[x+y*Nx]);
                ++k;
            }
        if(k > 0) err /= k;
        return (err < EPSILON);
    }

    public boolean checkConvergeHS() {
        double err = 0.0;
        int k = 0;
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x+y) % 2 == 0)
            if(U[x+y*Nx] != V[x+y*Nx])
            if(W[x+y*Nx] != WALL_VALUE) {
                err += Math.abs(1-U[x+y*Nx]/V[x+y*Nx]);
                ++k;
            }
        if(k > 0) err /= k;
        return (err < EPSILON);
    }

    public boolean checkConvergeQS() {
        double err = 0.0;
        int k = 0;
        for(int y = 1; y < Ny-1; y++)
        for(int x = 1; x < Nx-1; x++)
            if((x%2 == 0) && (y%2 == 0))
            if(U[x+y*Nx] != V[x+y*Nx])
            if(W[x+y*Nx] != WALL_VALUE) {
                err += Math.abs(1-U[x+y*Nx]/V[x+y*Nx]);
                ++k;
            }
        if(k > 0) err /= k;
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
                System.out.print(String.format("%f ", V[x+y*Nx]));
            System.out.println();
        }
    }
    
    public void saveMatrix(String fname) {
        try {
            PrintWriter writer = new PrintWriter(fname, "UTF-8");
            for(int y = 0; y < Ny; y++) {
                for(int x = 0; x < Nx; x++)
                    writer.print(String.format("%.16f ", V[x+y*Nx]));
                writer.println();
            }
            writer.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void loadMatrix(String fname) {
        try {
            File file = new File(fname);
            Scanner sc = new Scanner(file);
            for(int y = 0; y < Ny; y++) {
                for(int x = 0; x < Nx; x++) {
                    U[x+y*Nx] = V[x+y*Nx] = sc.nextDouble();
                }
            }
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }    
}
