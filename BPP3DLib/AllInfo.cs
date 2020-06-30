/* Copyright: Muamer Hrncic */
/* Original author(s): Muamer Hrncic */
/* License: MIT */
/* Purpose: Algorithm to solve the bin packing problem. */

#region Description

/* The three dimensional bin packing problem belongs to the  */
/* class of NP complete problems and can be formulated as follows: */

/* Let be J={1,...,n} a set of small rectangular items with dimensions w_i,h_i,d_i. */
/* All small items must be packed into bins of dimensions W,H,D. */
/* All dimensions are positive integers and the dimensions of the small */
/* items are smaller than or equal to the corresponding bin dimensions.  */

/* The task is to pack all the small items,  */
/* such that: */

/* 1) All small items are packed into a minimum number of bins */
/* 2) No two items overlap. */

/* The algorithm solves the problem for orthogonal packing, which means the items are packed such that*/
/* the edges of the small items are parallel to the correspoding bin edge. */
/* No rotations of items or bins are allowed. */
/* The algorithm is capable to solve the problem for general packings and robot packings. */

/* For a detailed description of the algorithm and original C-Code refer to: */

/* [1] Silvano Martello, David Pisinger, and Daniele Vigo. "The three-dimensional */
/* bin packing problem." Oper. Res., 48(2):256–267, March */
/* 2000. */

/* [2] Silvano Martello, David Pisinger, Daniele Vigo, Edgar Boef, and Jan Korst. */
/* "Algorithm 864: General and robot-packable variants of the threedimensional */
/* bin packing problem." ACM Trans. Math. Softw., 33:7, 01 */
/* 2007. */

#endregion

/* This C# code is a reimplementation of the original C-Code provided */
/* by Daniel Pisinger at http://hjemmesider.diku.dk/~pisinger/codes.html. */
/* This code comes with no warranty. */

using System;
using System.Collections.Generic;
using System.Text;

namespace BPP3DLib
{
    // AllInfo contains all the information of the problem
    public class AllInfo
    {
        int w;                  // x-size of bin                                    
        int h;                  // y-size of bin                                    
        int d;                  // z-size of bin                                    
        long binVol;            // Volume of the bin                                  
        int n;                  // Number of boxes                                  
        int packType;           // Packing type: General or Robot                   
        int fBox;               // Index of first box in problem                    
        int lBox;               // Index of last box in problem                     
        int fSol;               // Index of first box in current solution                  
        int fOpt;               // Index of first box in optimal solution           
        int lOpt;               // Index of last box in optimal solution            
        bool[] closed;          // Array of bin condition, closed or not closed     
        int fClosed;            // Index of first box in closed bins                
        int lClosed;            // Index of last box in closed bins                 
        int numOfClosed;        // Number of closed bins                            
        long maxFill;           // The best filling found                           
        int mCut;               // how many siblings at each node in b&b            
        Box[] boxes;            // Array of boxes in problem                        
        Box[] boxesCurSol;      // Array of boxes in in current solution            
        Box[] boxesOptSol;      // Array of boxes in optimal solution               
        Box[] boxesClosedBin;   // Array of boxes in closed bins   
        long bound0;            // Bound L_0 at root node                   
        long bound1;            // Bound L_1 at root node                   
        long bound2;            // Bound L_2 at root node                   
        long lb;                // The best of the three bounds "boundX"                        
        int z;                  // Currently best solution                  
        int maxIter;            // Maximum number of iterations in OneBinRobot           
        int miss;               // Number of boxes not packed in OneBinRobot  
        int nodes;              // Nodes in branch-and-bound                
        int iterat;             // Iterations in OneBinDecision            
        int subNodes;           // Sub nodes in branch-and-bound, 1000 subNodes are 1 node                
        int subIterat;          // Iterations in OneBinDecision            
        long callsFitsMore;     // Number of calls to FitsMore
        long callsFitsTwo;      // Number of calls to FitsTwo
        long callsFitsThree;    // Number of calls to FitsThree
        long iter3d;            // Iterations in OneBinRobot or OneBinGeneral   
        int zLayer;             // Heuristic solution               
        long exactTopo;         // Number of topological sorts 
        long exactTopn;         // Number of topological sorts within the for loop             
        int exactCall;          // Number of calls to OneBinGeneral (exact solution)             
        int exactn;             // Largest problem for exact                
        double layHeurTime;     // Layer heuristic computing time           
        double generalTime;     // Time used in OneBinGeneral             
        double robotTime;       // Time used in OneBinRobot                
        double time;            // Overall computing time                           
        int maxClose;           // Maximum number of closed bins at any time    
        int nodeLimit;          // Maximum number of nodes in main tree     
        int iterLimit;          // Maximum number of iterations in OneBin   
        int timeLimit;          // Maximum amount of time to be used        
        bool stopped;           // Variable to indicate if algorithm was stopped   
        double globStartTime;   // Variable to indicate the global start time

        public int W { get => w; set => w = value; }
        public int H { get => h; set => h = value; }
        public int D { get => d; set => d = value; }
        public long BinVol { get => binVol; set => binVol = value; }
        public int N { get => n; set => n = value; }
        public int PackType { get => packType; set => packType = value; }
        public int FBox { get => fBox; set => fBox = value; }
        public int LBox { get => lBox; set => lBox = value; }
        public int FSol { get => fSol; set => fSol = value; }
        public int FOpt { get => fOpt; set => fOpt = value; }
        public int LOpt { get => lOpt; set => lOpt = value; }
        public bool[] Closed { get => closed; set => closed = value; }
        public int FClosed { get => fClosed; set => fClosed = value; }
        public int LClosed { get => lClosed; set => lClosed = value; }
        public int NumOfClosed { get => numOfClosed; set => numOfClosed = value; }
        public long MaxFill { get => maxFill; set => maxFill = value; }
        public int MCut { get => mCut; set => mCut = value; }
        public Box[] Boxes { get => boxes; set => boxes = value; }
        public Box[] BoxesCurSol { get => boxesCurSol; set => boxesCurSol = value; }
        public Box[] BoxesOptSol { get => boxesOptSol; set => boxesOptSol = value; }
        public Box[] BoxesClosedBin { get => boxesClosedBin; set => boxesClosedBin = value; }
        public long Bound0 { get => bound0; set => bound0 = value; }
        public long Bound1 { get => bound1; set => bound1 = value; }
        public long Bound2 { get => bound2; set => bound2 = value; }
        public long Lb { get => lb; set => lb = value; }
        public int Z { get => z; set => z = value; }
        public int MaxIter { get => maxIter; set => maxIter = value; }
        public int Miss { get => miss; set => miss = value; }
        public int Nodes { get => nodes; set => nodes = value; }
        public int Iterat { get => iterat; set => iterat = value; }
        public int SubNodes { get => subNodes; set => subNodes = value; }
        public int SubIterat { get => subIterat; set => subIterat = value; }
        public long CallsFitsTwo { get => callsFitsTwo; set => callsFitsTwo = value; }
        public long CallsFitsThree { get => callsFitsThree; set => callsFitsThree = value; }
        public long CallsFitsMore { get => callsFitsMore; set => callsFitsMore = value; }
        public long IterThreeD { get => iter3d; set => iter3d = value; }
        public int ZLayer { get => zLayer; set => zLayer = value; }
        public long ExactTopo { get => exactTopo; set => exactTopo = value; }
        public long ExactTopn { get => exactTopn; set => exactTopn = value; }
        public int ExactCall { get => exactCall; set => exactCall = value; }
        public int Exactn { get => exactn; set => exactn = value; }
        public double GeneralTime { get => generalTime; set => generalTime = value; }
        public double RobotTime { get => robotTime; set => robotTime = value; }
        public double Time { get => time; set => time = value; }
        public double LayHeurTime { get => layHeurTime; set => layHeurTime = value; }
        public int MaxClose { get => maxClose; set => maxClose = value; }
        public int NodeLimit { get => nodeLimit; set => nodeLimit = value; }
        public int IterLimit { get => iterLimit; set => iterLimit = value; }
        public int TimeLimit { get => timeLimit; set => timeLimit = value; }
        public bool Stopped { get => stopped; set => stopped = value; }
        public double GlobStartTime { get => globStartTime; set => globStartTime = value; }

        public AllInfo(int n, int W, int H, int D, int[] w, int[] h, int[] d, int nodeLimit, int iterLimit, int timeLimit, int packingType)
        {
            this.w = W;
            this.h = H;
            this.d = D;
            this.binVol = W * H * (long)D;
            this.n = n;
            this.packType = packingType;
            this.fBox = 0;
            this.lBox = n - 1;
            this.fSol = 0;
            this.fOpt = 0;
            this.lOpt = n - 1;
            this.closed = new bool[n+1];
            this.fClosed = 0;
            this.lClosed = this.fClosed - 1;
            this.numOfClosed = 0;
            this.maxFill = 0;
            this.mCut = 0;
            this.boxes = new Box[n+1];
            this.boxesCurSol = new Box[n+1];
            this.boxesOptSol = new Box[n+1];
            this.boxesClosedBin = new Box[n+1];

            // Initialize Boxes;
            for (int i = 0; i < n; i++)
            {
                if ((w[i] < 1) || (w[i] > W)) throw new Exception("Bad w!");
                if ((h[i] < 1) || (h[i] > H)) throw new Exception("Bad h!");
                if ((d[i] < 1) || (d[i] > D)) throw new Exception("Bad d!");
                this.boxes[i] = new Box(i + 1, w[i], h[i], d[i]);
            }

            // Sort by nonincreasing volume;
            Array.Sort(this.boxes, 0, this.lBox + 1, new BoxVComparer());

            this.bound0 = 0;
            this.bound1 = 0;
            this.bound2 = 0;
            this.lb = 0;
            this.z = this.n+1;
            this.maxIter = 0;
            this.miss = 0;
            this.nodes = 0;
            this.iterat = 0;
            this.subNodes = 0;
            this.subIterat = 0;
            this.callsFitsMore = 0;
            this.iter3d = 0;
            this.zLayer = 0;
            this.exactTopo = 0;
            this.exactTopn = 0;
            this.exactCall = 0;
            this.exactn = 0;
            this.generalTime = 0;
            this.robotTime = 0;
            this.time = 0;
            this.layHeurTime = 0;
            this.maxClose = 0;
            this.nodeLimit = nodeLimit;
            this.iterLimit = iterLimit;
            this.timeLimit = timeLimit;
            this.packType = packingType;
            this.stopped = false;
            this.callsFitsTwo = 0;
            this.callsFitsThree = 0;
            this.globStartTime = 0;
        }
    }
}
