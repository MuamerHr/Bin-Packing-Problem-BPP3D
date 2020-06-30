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
using System.Linq;


namespace BPP3DLib
{
    public static class BPP3D
    {
        #region Globals
        public static int IUnit = 1000;             // Scaling factor of nodes and iterations;
        public static int MaxBoxes = 101;           // Maximum number of boxes allowed for computations;
        public static int MaxOneBpp = 1000000;      // Maximum number of iterations in the 1DBPP;
        public static int MaxIter = 1000;           // Maximum iterations in heuristics;
        public static int MaxClose = 16;            // Maximum level for which TryCloseBin is applied;

        public const int WDim = 0;                  // Rotations of boxes;
        public const int HDim = 1;
        public const int DDim = 2;

        public const int General = 0;               // Packing type;
        public const int Robot = 1;

        public const int Left = 0;                  // Relative placements of two boxes i and j, used by general packing algorithm;
        public const int Right = 1;
        public const int Under = 2;
        public const int Above = 3;
        public const int Front = 4;
        public const int Behind = 5;
        public const int Undef = 6;
        public const int RelMax = 8;

        public static long StackDepth;      // Depth of the domain stack;

        // Bool variable to indicate time-out situation;
        public static bool Stopped = false;

        // Counter used to ensure that 1D BPP at most performs MaxOneBpp iterations;
        public static int BpIterat;

        // Bool variables to indicate when packing algorithm should terminate;
        public static bool Feasible, Terminate;

        // Stack of domain pairs, used for general packing;
        public static DomainPair[] DomainStack;
        public static int DomPos;

        // Domain of each box;
        public static int[][][] Domain;

        // Current relation between two boxes;
        public static int[][] Relation;

        // Actual level in recursive packing algorithm;
        public static int BBLevel;

        public static DateTime time;            // Variable which hold the actual time.
        public static Point[] foundCorners;     // Array of points, used in robot packing algorithm
        public static Box[] t;                  // Array of boxes
        public static Box[] f;                  // Array of boxes
        public static DomainPair ds;            

        #endregion

        #region small procedures

        public static void CheckTimeLimit(long max, double ts)
        {
            // Test for time limit;

            if (max == 0) return;

            time = DateTime.Now;
            double te = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;
            double t = te - ts;

            if (t >= max)
            {
                Stopped = true;
            }
        }

        public static void CheckNodeLimit(long nodes, long max)
        {
            // Test for node limit;
            if (max == 0) return;
            if (nodes >= max)
            {
                Stopped = true;
            }
        }

        public static void CheckIterLimit(long iterations, long max)
        {
            // Test for iteration limit;
            if (max == 0) return;
            if (iterations >= max)
            {
                Stopped = true;
            }
        }
        public static int Max(int i, int j)
        {
            // Find the maximum of two integers;
            if (i > j)
            {
                return i;
            }
            return j;
        }

        public static int Diff(int i, int j)
        {
            // Difference of indices;
            return j - i + 1;
        }

        #endregion

        #region Handling Solutions
        public static void CheckSolution(AllInfo a, Box[] boxes, int f, int l)
        {
            // Check if solution is feasible;
            int m = l + 1;            

            // Check for overlapping;
            for (int i = f; i != m; i++)
            {
                if (boxes[i].K <= 0) continue;  // Box currently not chosen;

                for (int j = f; j != m; j++)
                {
                    if (i == j) continue;
                    if (boxes[i].No == boxes[j].No) { throw new Exception("Duplicated box " + boxes[i].No.ToString() + ", CheckSolution"); }
                    if (boxes[j].K <= 0) continue;
                    if (boxes[i].Bno != boxes[j].Bno) continue;
                    if ((boxes[i].X + boxes[i].W > boxes[j].X) && (boxes[j].X + boxes[j].W > boxes[i].X) &&
                        (boxes[i].Y + boxes[i].H > boxes[j].Y) && (boxes[j].Y + boxes[j].H > boxes[i].Y) &&
                        (boxes[i].Z + boxes[i].D > boxes[j].Z) && (boxes[j].Z + boxes[j].D > boxes[i].Z))
                    {
                        throw new Exception("Overlap box" + boxes[i].No.ToString() + "," + boxes[j].No.ToString() + ": [" + boxes[i].W.ToString() + "," + boxes[i].H.ToString() + "," + boxes[i].D.ToString() + "] [" + boxes[j].W.ToString() + "," + boxes[j].H.ToString() + "," + boxes[j].D.ToString() + "]" + ", CheckSolution");
                    }
                }
            }

            if (a.PackType != Robot) return;

            bool stillBoxes, foundExtreme, extreme;
            // Check if solution is robot packable;
            for (int b = 1; b <= a.Z; b++)
            {
                while (true)
                {
                    stillBoxes = false;
                    foundExtreme = true;
                    for (int i = f; i < m; i++)
                    {
                        if ((boxes[i].Bno == b) && (boxes[i].K == 1))
                        {
                            stillBoxes = true;
                            break;
                        }
                    }
                    if (!stillBoxes) break;

                    foundExtreme = false;
                    for (int i = f; i < m; i++)
                    {
                        if ((boxes[i].Bno != b) || (boxes[i].K != 1)) continue;
                        extreme = true;
                        for (int j = f; j < m; j++)
                        {
                            if (j == i) continue;
                            if ((boxes[j].Bno != b) || (boxes[j].K != 0)) continue;
                            if ((boxes[i].X < boxes[j].X + boxes[j].W) || (boxes[i].Y < boxes[j].Y + boxes[j].H) || (boxes[i].Z < boxes[j].Z + boxes[j].D))
                            {
                                extreme = false;
                            }
                        }
                        if (extreme) { boxes[i].K = -1; foundExtreme = true; }
                    }
                    if (!foundExtreme) break;
                }

                if (!foundExtreme)
                {                    
                    List<string> info = new List<string>();

                    for (int i = f; i <= m; i++)
                    {
                        if (boxes[i].Bno == b)
                        {
                            string s = "";
                            s += "no " + boxes[i].No.ToString();
                            s += " (" + boxes[i].X.ToString();
                            s += ", " + boxes[i].Y.ToString();
                            s += ", " + boxes[i].Z.ToString();
                            s += ") (" + boxes[i].W.ToString();
                            s += ", " + boxes[i].H.ToString();
                            s += ", " + boxes[i].D.ToString();
                            s += ") bin " + boxes[i].Bno.ToString();
                            s += " k " + boxes[i].W.ToString();
                            info.Add(s);
                        }
                    }
                    throw new Exception("Not robot Packable, CheckSolution.", new NotRobPackableException(info));     
                }
                
                // Restore K values.
                for (int i = f; i < m; i++)
                {
                    if (boxes[i].K == -1)
                    {
                        boxes[i].K = 1;
                    }
                }
            }
        }

        public static void SaveSolution(AllInfo a, Box[] boxes, int f, int l, int z)
        {
            /* ======================================================================
	                                        SaveSolution
            ====================================================================== */
            // Save an updated solution, checking its validity;

            int m = l + 1;

            // First check validity;
            if (z >= a.Z) throw new Exception("Not improved, SaveSolution.");
            for (int i = f; i != m; i++)
            {
                if ((1 <= boxes[i].Bno) && (boxes[i].Bno <= z)) { continue; }

                throw new Exception("Illegal bin " + boxes[i].Bno.ToString() + ", box " + boxes[i].No.ToString() + ", SaveSolution.");
            }

            // Save the solution;
            a.Z = z;
            int k = a.FOpt;
            for (int i = f; i != m; i++, k++)
            {
                a.BoxesOptSol[k] = boxes[i];
            }
            m = a.LClosed + 1;
            for (int i = a.FClosed; i != m; i++, k++)
            {
                a.BoxesOptSol[k] = a.BoxesClosedBin[i];
            }
            m = a.LOpt + 1;
            for (int i = a.FOpt; i != m; i++)
            {
                a.BoxesOptSol[i].K = 1;
            }
            if (Diff(a.FOpt, k - 1) != a.N)
            {
                throw new Exception("Not correct amount of boxes, SaveSolution.");
            }

            // Check Solution;
            CheckSolution(a, a.BoxesOptSol, a.FOpt, a.LOpt);
        }

        #endregion

        #region Lower Bounds
        public static long BoundZero(double binVol, Box[] boxes, int f, int l)
        {
            // The continuous bound L_0;
            /* ======================================================================
                                            BoundZero
            ====================================================================== */

            int i, m;
            long volSum, lb;

            volSum = 0;
            for (i = f, m = l + 1; i != m; i++)
            {
                volSum += boxes[i].Vol;
            }
            lb = (long)Math.Ceiling(volSum / binVol);
            return lb;
        }

        public static void Rotate(AllInfo a, Box[] boxes, int f, int l, bool prob)
        {
            /* ======================================================================
                                        Rotate
            ====================================================================== */
            // Rotates the dimensions by one step;

            int i, m;
            int dim, coord;

            for (i = f, m = l + 1; i != m; i++)
            {
                dim = boxes[i].W; boxes[i].W = boxes[i].H; boxes[i].H = boxes[i].D; boxes[i].D = dim;
                coord = boxes[i].X; boxes[i].X = boxes[i].Y; boxes[i].Y = boxes[i].Z; boxes[i].Z = coord;
            }

            if (prob) // Check if the bin dimensions have to be rotated;
            {
                dim = a.W; a.W = a.H; a.H = a.D; a.D = dim;
            }
        }

        public static void ChooseBoxes(Box[] boxes, Box[] chBox, int f, int l, int W2, int D2, ref int lBox)
        {
            /* ======================================================================
                                            ChooseBoxes
            ====================================================================== */

            // Returns a set of boxes with w > W2 and d > D2. The set is used in BoundOne;

            int m = l + 1;

            for (int i = f; i != m; i++)
            {
                if ((boxes[i].W > W2) && (boxes[i].D > D2))
                {
                    chBox[lBox] = boxes[i];
                    lBox++;
                }
            }
            lBox -= 1;
        }

        public static void FindPlist(Box[] boxes, int[] pl, int f, int l, int M, int dim)
        {
            /* ======================================================================
                                            FindPlist
            ====================================================================== */

            // Returns a zero-terminated list of dimensions;

            int m = l + 1; ;
            int k = 0;
            
            switch (dim)
            {
                case WDim:
                    for (int i = f; i != m; i++)
                    {
                        if (boxes[i].W <= M) { pl[k] = boxes[i].W; k++; }
                    }
                    break;
                case HDim:
                    for (int i = f; i != m; i++)
                    {
                        if (boxes[i].H <= M) { pl[k] = boxes[i].H; k++; }
                    }
                    break;
                case DDim:
                    for (int i = f; i != m; i++)
                    {
                        if (boxes[i].D <= M) { pl[k] = boxes[i].D; k++; }
                    }
                    break;
            }
            if (k == 0) return;

            // Sort the dimensions;
            Array.Sort(pl, 0, k);

        }

        public static int BoundOneX(AllInfo a, Box[] boxes, int f, int l)
        {
            /* ======================================================================
                                        BoundOneX
             ====================================================================== */

            // Derive bound L_1 for a fixed dimension;

            if (l == f - 1) return 0;

            int m;
            int H = a.H, H2 = H / 2, h;
            int j1, j2, j3, j2h, j2hp, j3h;
            int lb = 1, lbOne, alpha, beta;

            Box[] boxesBigLBOne = new Box[a.N+1];

            int lbox = 0;
            int[] pList = new int[a.N+1];

            ChooseBoxes(boxes, boxesBigLBOne, f, l, a.W / 2, a.D / 2, ref lbox);

            if (lbox < 0)
            {
                /* empty */
                return lb;
            }

            FindPlist(boxesBigLBOne, pList, 0, lbox, H2, HDim);
            pList = pList.Distinct().ToArray();// remove duplicates and get list of distinct values
            m = lbox + 1;

            foreach (int p in pList)
            {
                if (p == 0) break;

                j1 = j2 = j3 = j2h = j2hp = j3h = 0;

                for (int i = 0; i != m; i++)
                {
                    h = boxesBigLBOne[i].H;
                    if (h > H - p)
                    {
                        // Largest boxes;
                        j1++;
                        continue;
                    }
                    if ((H - p >= h) && (h > H2))
                    {
                        // Large boxes;
                        j2++;
                        j2h += h;
                        j2hp += (H - h) / p;
                        continue;
                    }
                    if ((H2 >= h) && (h >= p))
                    {
                        // Small boxes;
                        j3++;
                        j3h += h;
                        continue;
                    }
                }
                alpha = (int)Math.Ceiling((j3h - (j2 * H - j2h)) / (double)H);
                beta = (int)Math.Ceiling((j3 - j2hp) / (double)(H / p));

                if (alpha < 0) alpha = 0;
                if (beta < 0) beta = 0;

                lbOne = j1 + j2 + Max(alpha, beta);
                if (lbOne > lb) lb = lbOne;
            }
            return lb;
        }

        public static int BoundOne(AllInfo a, Box[] boxes, int f, int l)
        {
            // Derive bound L_1 as the best of all L_1 bounds for three rotations;

            int lb, lbx;

            lb = 0;
            for (int i = WDim; i <= DDim; i++)
            {
                lbx = BoundOneX(a, boxes, f, l);
                if (lbx > lb) lb = lbx;
                Rotate(a, boxes, f, l, true);
            }
            return lb;
        }

        public static int BoundTwoX(AllInfo a, Box[] boxes, int f, int l)
        {
            /* ======================================================================
                                            BoundTwoX
            ====================================================================== */

            // Derive bound L_2 for a fixed dimension;

            int m = l + 1;
            int W = a.W, H = a.H, D = a.D, W2 = W / 2, D2 = D / 2;
            long k1h, k23v;
            int lb, lb1, lbx, fract;
            int[] plist = new int[a.N+1];
            int[] qlist = new int[a.N+1];
            double WD = W * D;

            // First derive bound_one;
            lb = lb1 = BoundOneX(a, boxes, f, l);
            int hlb1 = H * lb1;

            FindPlist(boxes, plist, f, l, W2, WDim);        // Run through all threshold values of p;
            plist = plist.Distinct().ToArray();             // Remove duplicates to get list of distinct values;

            FindPlist(boxes, qlist, f, l, D2, DDim);        // Run through all threshold values of q;
            qlist = qlist.Distinct().ToArray();             // Remove duplicates to get list of distinct values;

            foreach (int p in plist)
            {
                if (p == 0) break;

                foreach (int q in qlist)
                {
                    if (q == 0) break;
                    k1h = k23v = 0;
                    for (int i = f; i != m; i++)
                    {
                        if ((boxes[i].W > W - p) && (boxes[i].D > D - q)) { k1h += boxes[i].H; continue; }
                        if ((boxes[i].W >= p) && (boxes[i].D >= q)) { k23v += boxes[i].Vol; }
                    }
                    fract = (int)Math.Ceiling((k23v - (hlb1 - k1h) * WD) / (double)a.BinVol);
                    if (fract < 0) fract = 0;

                    lbx = lb1 + fract;
                    if (lbx > lb) lb = lbx;
                }
            }
            return lb;
        }

        public static int BoundTwo(AllInfo a, Box[] boxes, int f, int l)
        {
            // Derive bound L_2 as the best of all L_2 bounds for three rotations;

            int i, lb, lbx;

            lb = 0;
            for (i = WDim; i <= DDim; i++)
            {
                lbx = BoundTwoX(a, boxes, f, l);
                if (lbx > lb) lb = lbx;
                Rotate(a, boxes, f, l, true);
            }
            return lb;
        }

        #endregion

        #region Heuristic Filling

        public static void FillOneLayer(AllInfo a, Box[] boxes, int f, int m, int l, int d)
        {
            /* ======================================================================
                                                FillOneLayer
               ====================================================================== */

            // Fill a layer of depth d (d depth of box f) by arranging the 
            // boxes in a number of vertical shelfs. 
            // The boxes i packed are assigned coordinates (x, y)
            // and the field K is set to the argument d (layer no).

            int i, s, t;
            int r; // Remaining width;
            int[] width = new int[a.N + 1];
            int[] height = new int[a.N + 1];
            int[] x = new int[a.N + 1];

            Array.Sort(boxes, f, Diff(f, m), new BoxHComparer());   // Sort boxes by height;

            r = a.W;
            x[0] = 0; width[0] = 0; height[0] = 0;

            for (s = 1, i = f; i != l + 1; s++)
            {
                x[s] = x[s - 1] + width[s - 1];
                height[s] = 0;
                width[s] = boxes[i].W;
                if (width[s] > r)
                {
                    width[s] = r;
                }

                r -= width[s];
                for (; i != l + 1; i++)
                {
                    for (t = s; t != 0; t--)
                    {
                        if (boxes[i].W <= width[t] && height[t] + boxes[i].H <= a.H)
                        {
                            boxes[i].Y = height[t];
                            boxes[i].X = x[t];
                            boxes[i].K = d;
                            height[t] += boxes[i].H;
                            break;
                        }
                    }
                    if ((t == 0) && (r > 0)) break; // Strip is full, create a new one;
                }
            }
        }

        public static int CountArea(Box[] boxes, int f, int l, long binArea)
        {
            /* ======================================================================
				                            CountArea
            ====================================================================== */

            // Select a subset of the boxes such that the selected boxes have a total
            // area of two times the face of a bin (the parameter: binArea).


            int i;
            long area, d;

            for (area = 0, i = f; i != l + 1; i++)
            {
                d = boxes[i].H * boxes[i].W;
                area += d;
                if (area > 2 * binArea)
                {
                    return i - 1;
                }
            }
            return l;
        }

        public static int RemoveBoxes(Box[] boxes, int f, int l, ref int depth)
        {
            /* ======================================================================
                                                RemoveBoxes
            ====================================================================== */

            // Remove the boxes which were chosen for a layer, i.e., where the field "box".K != 0.
            // The depth of the layer is set equal to the deepest box chosen.

            int i, j;
            int d;

            for (i = f, j = l, d = 0; i != j + 1;)
            {
                if (boxes[i].K != 0)
                {
                    if (boxes[i].D > d)
                    {
                        d = boxes[i].D;
                    }
                    i++;
                }
                else
                {
                    Box t = boxes[i];
                    boxes[i] = boxes[j];
                    boxes[j] = t;
                    j--;
                }
            }
            depth = d;
            return i;
        }

        public static void AssignBoxes(HeuristicPair[] t, int u, int maxBin, Box[] boxes, int f, int l)
        {
            /* ======================================================================
                                        AssignBoxes
            ====================================================================== */

            // Assign z-coordinates to the boxes, once they layers have been combined
            // to individual bins by solving a 1D-BPP.

            int i, m;
            int h;
            int b, z;

            // Derive z-coordinates for each layer;
            for (b = 1; b <= maxBin; b++)
            {
                z = 0;
                for (h = 0; h <= u; h++)
                {
                    if (t[h].BinNoLay == b)
                    {
                        t[h].Z = z; z += t[h].D;
                    }
                }
            }

            for (i = f, m = l + 1; i != m; i++)
            {
                h = boxes[i].K - 1;
                boxes[i].Z = t[h].Z; boxes[i].Bno = t[h].BinNoLay;
            }
        }

        public static void BinPackOneDim(HeuristicPair[] t, int i, int f, int l, int[] b, int bno, ref int z)
        {
            /* ======================================================================
                                        BinPackOneDim
            ====================================================================== */

            // One-dimensional bin-packing algorithm. In each iteration, the next
            // box/slice is assigned to every open bin as well as to a new bin. The
            // algorithm terminates when maxbpp iterations have been performed,
            // returning the heuristic solution found.

            int j;
            int k;

            BpIterat++;
            if (BpIterat > MaxOneBpp || bno >= z) return;

            if (i > l)
            {
                z = bno;
                for (k = f; k <= l; k++) t[k].BinNoLay = t[k].B;
            }
            else
            {
                for (j = 0; j < bno; j++)
                {
                    if (t[i].D <= b[j])
                    {
                        b[j] -= t[i].D;
                        t[i].B = j + 1;
                        BinPackOneDim(t, i + 1, f, l, b, bno, ref z);
                        b[j] += t[i].D;
                    }
                }
                b[bno] -= t[i].D;
                t[i].B = bno + 1;
                BinPackOneDim(t, i + 1, f, l, b, bno + 1, ref z);
                b[bno] += t[i].D;
            }
        }

        public static void HeuristicFillOneDim(AllInfo a)
        {
            /* ======================================================================
                                    HeuristicFillOneDim
            ====================================================================== */

            // Heuristic algorithm for the 3D BPP. A number of layers are constructed
            // using the shelf approach to pack every layer. Then the individual layers
            // are combined to bins by solving a 1D BPP defined in the layer depths.

            int j, f, l, m;
            int i, n, h;

            HeuristicPair[] t = new HeuristicPair[a.N+1];
            Box[] boxes = new Box[a.N];
            for (int i1 = 0; i1 < a.N; i1++)
            {
                boxes[i1] = a.Boxes[i1];
            }
            int d, z;
            d = 0;

            for (j = 0; j < a.N; j++)
            {
                boxes[j].Bno = boxes[j].X = boxes[j].Y = boxes[j].Z = 0;
                boxes[j].K = 0;
            }

            // Fill layer one by one;
            for (f = 0, l = boxes.Length - 1, h = 0; ; h++)
            {
                n = Diff(f, l); if (n == 0) break;

                Array.Sort(boxes, f, n, new BoxDComparer()); // Sort boxes by depth;

                m = CountArea(boxes, f, l, a.W * a.H);
                FillOneLayer(a, boxes, f, m, l, h + 1);
                f = RemoveBoxes(boxes, f, l, ref d);
                t[h].D = d; t[h].BinNoLay = h + 1;          // Initially put layers into separate bins */
                t[h].Z = 0; t[h].LayNo = h + 1;             // This ensures feas. solution if terminate */
            }

            // Split into bins by solving 1-dim binpacking;
            int[] b = new int[h];
            for (i = 0; i < h; i++) b[i] = a.D;             // All bins are empty;

            Array.Sort(t, 0, h, new HeurPairXComparer());

            z = h + 1; BpIterat = 0;
            BinPackOneDim(t, 0, 0, h - 1, b, 0, ref z);

            Array.Sort(t, 0, h, new HeurPairLComparer());   // Order according to layer number;

            // Assign bin number to each box
            AssignBoxes(t, h - 1, z, boxes, 0, boxes.Length - 1);

            if (z < a.ZLayer) a.ZLayer = z;
            if (a.ZLayer < a.Z)
            {
                // All boxes went into closed bins;
                SaveSolution(a, boxes, a.FBox, a.LBox, a.ZLayer);
            }
        }

        public static void HeuristicFillThreeDim(AllInfo a)
        {
            /* ======================================================================
                                    HeuristicFillThreeDim
            ====================================================================== */

            // Call the heuristic HeuristicFillOneDim, for three different rotations
            // of the problem.

            int i;
            int f = 0;
            int l = a.Boxes.Length - 1;

            // Store the time;
            time = DateTime.Now;
            double ts = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;


            a.ZLayer = a.N;  // Very bad incumbent solution;  n boxes require at most n layers;
            for (i = WDim; i <= DDim; i++)
            {
                HeuristicFillOneDim(a);
                Rotate(a, a.BoxesOptSol, f, l, false); // Rotates the solution;
                Rotate(a, a.Boxes, f, l, true);
            }

            time = DateTime.Now;
            double te = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;

            // Store the elapsed time
            a.LayHeurTime = te - ts;
        }
        #endregion

        #region General Packings

        /* **********************************************************************
        **********************************************************************
                        fill one 3D bin using GENERAL packing
        **********************************************************************
        ********************************************************************** */

        public static void ModifyAndPushRel(int i, int j, int rel, bool dom)
        {
            /* ======================================================================
                                    ModifyAndPush
            ====================================================================== */

            // Push the relation "rel" between box i and box j to a stack. If "dom"
            // is true, the relation is removed from the domain. If "dom" is false,
            // the relation "rel" is imposed between boxes "i" and "j".

            ds = DomainStack[DomPos];
            ds.I = i;
            ds.J = j;
            ds.Domain = dom;
            if (dom)
            {
                ds.Relation = rel;
                Domain[i][j][rel] = 0;
            }
            else
            {
                ds.Relation = Relation[i][j];
                Relation[i][j] = rel;
            }
            DomainStack[DomPos]= ds;
            DomPos++;
            if (DomPos == StackDepth) throw new Exception("Stack filled, ModifyAndPush.");

        }

        public static void PopDomains(int pos)
        {
            /* ======================================================================
                                PopDomains
            ====================================================================== */

            // Pop all relations between boxes from the stack. The stack is emptied
            // down to the depth given by "pos".

            while (DomPos != pos)
            {

                DomPos--;
                ds = DomainStack[DomPos];
                if (ds.Domain)
                {
                    Domain[ds.I][ds.J][ds.Relation] = 1;
                }
                else
                {
                    Relation[ds.I][ds.J] = ds.Relation;
                }
            }
        }

        public static int FindCoordinates(AllInfo a, Box[] t, int n, int f)
        {
            /* ======================================================================
                                        FindCoordinates
             ====================================================================== */

            // Find coordinates of boxes according to the currently pending relations.
            // In principle this can be done by topologically sorting the boxes according
            // to e.g. the left-right relations, and then assigning coordinates from
            // the left-most box to the right-most box.
            // The following implementation is a simplified version, which runs through
            // all pairs of boxes, and checks whether they satisfy the relation, otherwise
            // moving one of the boxes right, up or behind, according to the given
            // relation. The process is repeated until no pairs of boxes violate a
            // relation. Computational experiments have shown that the present approach
            // for the considered instances is faster than a topological sorting followed
            // by a critical path calculation.
            // If a box during the process gets moved outsides of the bin, then
            // the algorithm terminates with FALSE. Otherwise TRUE is returned, saying
            // that a feasible packing exists which respects the current relations.

            int g, h;
            int sum;
            int i, j, k, W, H, D;
            bool changed;

            // Check if feasible, i.e., at least one choice for each relation;
            W = a.W; H = a.H; D = a.D;

            for (i = 0; i < n; i++)
            {
                j = i + 1;

                for (; j < n; j++)
                {
                    if (Relation[i][j] != Undef) continue;

                    int dom = 0;
                    bool check = false;
                    
                    for (; dom < Undef; dom++)
                    {
                        if (Domain[i][j][dom] != 0)
                        {
                            check = true;
                            break;
                        }
                    }
                    if (check == true) continue;
                    return 0;
                }
            }

            // Initialize coordinates;
            for (i = 0; i < n; i++)
            {
                g = f + i;
                t[g].X = t[g].Y = t[g].Z = 0;
            }

            // Determine the coordinates;
            a.ExactTopo++;
            for (k = 0; k < n; k++)
            {
                a.ExactTopn++;
                changed = false;
                for (i = 0; i < n; i++)
                {
                    g = f + i;
                    j = i + 1;

                    for (; j < n; j++)
                    {
                        h = f + j;
                        switch (Relation[i][j])
                        {
                            case Undef:
                                break; // Do nothing;
                            case Left:
                                sum = t[g].X + t[g].W;
                                if (t[h].X < sum)
                                {
                                    t[h].X = sum; changed = true;
                                    if (sum + t[h].W > W) return 0;
                                }
                                break;
                            case Right:
                                sum = t[h].X + t[h].W;
                                if (t[g].X < sum)
                                {
                                    t[g].X = sum; changed = true;
                                    if (sum + t[g].W > W) return 0;
                                }
                                break;
                            case Under:
                                sum = t[g].Y + t[g].H;
                                if (t[h].Y < sum)
                                {
                                    t[h].Y = sum; changed = true;
                                    if (sum + t[h].H > H) return 0;
                                }
                                break;
                            case Above:
                                sum = t[h].Y + t[h].H;
                                if (t[g].Y < sum)
                                {
                                    t[g].Y = sum; changed = true;
                                    if (sum + t[g].H > H) return 0;
                                }
                                break;
                            case Front:
                                sum = t[g].Z + t[g].D;
                                if (t[h].Z < sum)
                                {
                                    t[h].Z = sum; changed = true;
                                    if (sum + t[h].D > D) return 0;
                                }
                                break;
                            case Behind:
                                sum = t[h].Z + t[h].D;
                                if (t[g].Z < sum)
                                {
                                    t[g].Z = sum; changed = true;
                                    if (sum + t[g].D > D) return 0;
                                }
                                break;
                        }
                    }
                }
                if (!changed) { return 1; }
            }
            return 0;
        }

        public static void CheckDomain(AllInfo a, Box[] t, int i, int j, int n, int f, int value)
        {
            /* ======================================================================
                                        CheckDomain
            ====================================================================== */

            // Temporarily impose the relation "value" between boxes "i" and "j",
            // and check whether a feasible assignment of coordinates exists which
            // respects all currently imposed relations. 
            // If the relation cannot be satisfied, it is removed from the domain
            // and pushed to a stack, so that it can be restored upon backtracking.

            if (Domain[i][j][value] == 0) return;    // Not allowed in any case;
            Relation[i][j] = value;
            if (FindCoordinates(a, t, n, f) == 0)
            {
                ModifyAndPushRel(i, j, value, true);
            }
        }

        public static int ReduceDomain(AllInfo a, Box[] t, int n, int f)
        {
            /* ======================================================================
                                 ReduceDomain
               ====================================================================== */

            // Constraint propagation algorithm. For each relation in the domain of
            // boxes "i" and "j", check if the relation has the posibility of being
            // satisfied. If some of the relations cannot be satisfied any more, they
            // are removed from the domain (and pushed to a stack, so that they can
            // be restored when the master search algorithm backtracks). If only one
            // relation remains in the domain, the relation is imposed at this node
            // and all descendant nodes.

            int i, j, k, l, m;
            m = 0;

            for (i = 0; i < n - 1; i++)
            {
                for (j = i + 1; j < n - 1; j++)
                {
                    if (Relation[i][j] == Undef)
                    {
                        CheckDomain(a, t, i, j, n, f, Left);
                        CheckDomain(a, t, i, j, n, f, Right);
                        CheckDomain(a, t, i, j, n, f, Under);
                        CheckDomain(a, t, i, j, n, f, Above);
                        CheckDomain(a, t, i, j, n, f, Front);
                        CheckDomain(a, t, i, j, n, f, Behind);
                        Relation[i][j] = Undef;
                        for (k = Left, l = 0; k < Undef; k++)
                        {
                            if (Domain[i][j][k] != 0) { l++; m = k; }
                        }
                        if (l == 0) return 0;
                        if (l == 1) { ModifyAndPushRel(i, j, m, false); }
                    }
                }
            }
            return 1;
        }

        public static void RecursiveGeneralPack(AllInfo a, Box[] t, int i, int j, int n, int f, int rel)
        {
            /* ======================================================================
                                 RecursiveGeneralPack
               ====================================================================== */

            // Recursive algorithm based on constraint programming used for assigning
            // relative positions to each pair of boxes. Each pair of boxes initially 
            // has an associated relation with domain LEFT, RIGHT, UNDER, ABOVE, FRONT, 
            // BEHIND. In each iteration of the algorithm a pair of boxes "i" and "j"
            // is assigned the relation "rel". Constraint propagation is then used to 
            // decrease the domains of remaining relations. 
            //   If it is not possible to assign coordinates to the boxes such that the 
            // currently imposed relations between pairs of boxes are respected, we 
            // backtrack.
            //   If each pair of boxes has been assigned a relation, and it is possible
            // to assign coordinates to the boxes such that the currently imposed 
            // relations between pairs of boxes are respected, we save the solution 
            // and return.
            //   Otherwise, constraint propagation is used to decrease the domains
            // of relations corresponding to each pairs of boxes. If a domain only
            // contains a single relation, the relation is fixed.
            //   The recursive step selects the next pair of boxes following "i" and "j"
            // and repeatedly assigns each relation from the domain to the relation 
            // variable.

            if (Stopped) return;
            a.IterThreeD++;

            if ((a.IterThreeD == a.MaxIter) && (a.MaxIter != 0)) Terminate = true;
            a.SubIterat++;

            if (a.SubIterat == IUnit)
            {
                a.SubIterat = 0;
                a.Iterat++;
                CheckIterLimit(a.Iterat, a.IterLimit);
                CheckTimeLimit(a.TimeLimit, a.GlobStartTime);
            }

            if (Terminate) return;

            Relation[i][j] = rel;

            for (int i1 = 0, j1 = 0; i1 != i && j1 != j;)
            {
                i1++;
                if (i1 >= j1) { i1 = 0; j1++; }
                if (Relation[i1][j1] == Undef)
                {
                    throw new Exception("Relation error " + i1.ToString() + " " + j1.ToString() + " , RecGeneralPack.");
                }
            }

            int feas = FindCoordinates(a, t, n, f);
            if (feas == 0) return;

            if ((i == n - 2) && (j == n - 1))
            {
                Feasible = true;
                Terminate = true;
                t.CopyTo(a.BoxesCurSol, 0);
                return;
            }

            int pos = DomPos;
            feas = ReduceDomain(a, t, n, f);

            if (feas == 1)
            {
                i++; if (i >= j) { i = 0; j++; }
                BBLevel++;
                rel = Relation[i][j];
                if (Domain[i][j][Left] == 1) RecursiveGeneralPack(a, t, i, j, n, f, Left);
                if (Domain[i][j][Right] == 1) RecursiveGeneralPack(a, t, i, j, n, f, Right);
                if (Domain[i][j][Under] == 1) RecursiveGeneralPack(a, t, i, j, n, f, Under);
                if (Domain[i][j][Above] == 1) RecursiveGeneralPack(a, t, i, j, n, f, Above);
                if (Domain[i][j][Front] == 1) RecursiveGeneralPack(a, t, i, j, n, f, Front);
                if (Domain[i][j][Behind] == 1) RecursiveGeneralPack(a, t, i, j, n, f, Behind);
                Relation[i][j] = rel;
            }
            PopDomains(pos);
        }

        public static bool OneBinGeneralPacking(AllInfo a, Box[] t, int f, int l, bool fast)
        {
            /* ======================================================================
                                    OneBinGeneralPacking
             ====================================================================== */

            // Check if boxes f..l can be packed into a single bin using general 
            // packing. If "fast" is TRUE, the problem is only solved heuristically,
            // hence if the algorithm returns FALSE we cannot rule out the posibility
            // of packing all boxes into the bin.
            // The algorithm is based on constraint programming, where
            // each pair of boxes initially has an associated relation with domain LEFT, 
            // RIGHT, UNDER, ABOVE, FRONT, BEHIND. Then the recursive algorithm "recpack" 
            // is called, which repeatedly tries to assign the relation a value, using
            // constraint propagation to decrease the domains of remaining boxes.

            // Check time limit;
            if (Stopped) return false;

            int n = Diff(f, l);
            int m = n * n * RelMax;
            a.IterThreeD = 0;
            a.MaxIter = (fast ? MaxIter : 0); // Limited or infinitly many;
            a.ExactCall++;


            // Store the time;
            time = DateTime.Now;
            double ts = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;

            #region INIT

            for (int i = 0; i < m; i++)
            {
                DomainStack[i] = new DomainPair();
            }

            DomPos = 0;
            Feasible = false;
            Terminate = false;
            BBLevel = 1;

            if (n > a.Exactn) a.Exactn = n;

            // Init relations and domains;
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    Relation[i][j] = Undef;
                    for (int k = Left; k < Undef; k++)
                    {
                        Domain[i][j][k] = 1;
                    }
                }
            }

            Domain[0][1][Right] = 0;
            Domain[0][1][Above] = 0;
            Domain[0][1][Behind] = 0;

            #endregion

            // Calling general pack procedure;
            RecursiveGeneralPack(a, t, 0, 0, n, f, Undef);

            if (Feasible) CheckSolution(a, t, f, l);

            time = DateTime.Now;
            double te = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;
            // Store the elapsed time;
            a.GeneralTime += te - ts;

            return Feasible;
        }

        #endregion

        #region Robot Packings
        /* **********************************************************************
           **********************************************************************
                            fill one 3D bin using ROBOT packing
           **********************************************************************
           ********************************************************************** */

        public static long FindEnvelope(Point[] fc, Point[] s1, ref int st,
            int W, int H, int D, int RW, int RH, int cz, ref int lc, ref int zn)
        {
            /* ======================================================================
                            Envelope
               ====================================================================== */

            // Find the two-dimensional envelope of the boxes given by extreme
            // points in the array fc up to index l.

            int last = lc + 1;
            int x, xx, y, z, ix, iy, iz, mz;
            long area;

            // Find corner points and area;
            x = xx = z = 0; y = H; area = 0; zn = D;
            lc = 0;

            for (int i = 0; i != last; i++)
            {
                iz = fc[i].Z;
                if (iz <= cz) continue;
                if (iz < zn) zn = iz; // Find minimum next z coordinate;
                ix = fc[i].X;

                if (ix <= x)
                {
                    if (iz > z)
                    {
                        fc[lc] = fc[i];
                        lc++;
                    }
                    continue;
                }
                iy = fc[i].Y;

                if ((x <= RW) && (iy <= RH))
                {
                    s1[st].X = x; 
                    s1[st].Y = iy; 
                    s1[st].Z = cz; 
                    st++;
                    area += (x - xx) * y;
                    y = iy; xx = x;
                }
                x = ix; z = iz;
                fc[lc] = fc[i];
                lc++;
            }
            if (y != 0) area += (W - xx) * y;
            st--;
            lc--;
            return area;
        }

        public static void CheckDominance(Point[] s1, int sk, int sl, int st)
        {
            /* ======================================================================
                                        CheckDominance
               ====================================================================== */

            // The 3D envelope is found by deriving a number of 2D envelopes. This
            // may however introduce some "false" corner points, which are marked
            // by the following algorithm.

            int t, u = st + 1;
            int stx;

            for (t = sl + 1; t != u; t++)
            {
                stx = s1[t].X;
                while (s1[sk].X < stx)
                {
                    sk++;
                    if (sk > sl) return;
                }
                if ((s1[sk].X == stx) && (s1[sk].Y == s1[t].Y)) s1[t].Z = 0;
            }
        }

        public static void RemoveDominated(Point[] s1, int s0, int sl, ref int sm)
        {
            /* ======================================================================
                                    RemoveDom
               ====================================================================== */

            // Remove "false" corner points marked by algorithm "CheckDominance".

            int m = sl + 1;
            sm = s0;
            for (; s0 != m; s0++)
            {
                if (s1[s0].Z == 0) continue;
                s1[sm] = s1[s0];
                sm++;
            }
            sm--;
        }

        public static long FindPlaces(Box[] t, Point[] s1, AllInfo a, int f, int l, ref int sm)
        {
            /* ======================================================================
                              FindPlaces
               ====================================================================== */

            // Find all corner points, where a new box may be placed as well as the
            // volume of the "envelope" occupied by already placed boxes. Already
            // placed boxes are given by the indices f,..,l, while a list of possible placings
            // is returned in the array s1 up to index sm. The return value of the procedure is an upper
            // bound on the possible filling of this bin.

            int W = a.W, H = a.H, D = a.D;
            int m = l + 1;
            int mind, RW, RH;
            int z = 0, zn = 0;
            int sk = 0, lc = 0, sl = 0, st = 0, s0 = 0;
            long vol = 0, area = 0;
            for(int i = 0; i < l + 1; i++)
            {
                foundCorners[i] = new Point();
            }            

            RW = W; RH = H; mind = D;

            // Select boxes which are chosen, and find minimum dimensions of the unchosen boxes;

            for (int j = f; j != m; j++)
            {
                if (t[j].K != 0)
                { // Defines an extreme box;
                    foundCorners[lc].X = t[j].X + t[j].W;
                    foundCorners[lc].Y = t[j].Y + t[j].H;
                    foundCorners[lc].Z = t[j].Z + t[j].D;
                    lc++;
                }
                else
                { // Free box;
                    if (t[j].W < RW) RW = t[j].W;
                    if (t[j].H < RH) RH = t[j].H;
                    if (t[j].D < mind) mind = t[j].D;
                }
            }

            // Sort the boxes according to max y (first) max x (second);
            if (lc >= 1)
            {
                Array.Sort(foundCorners, 0, Diff(f, l), new PointComparer());
            }

            // For each z-coordinate find the 2D envelope;
            vol = 0; sl = -1; sk = 0; s0 = 0;
            RW = W - RW; RH = H - RH;

            foundCorners[lc].X = W + 1;
            foundCorners[lc].Y = 0;
            foundCorners[lc].Z = D + 1;

            while (z != D)
            {
                // Find 2D envelope for all boxes which cover *z;
                st = sl + 1;
                area = FindEnvelope(foundCorners, s1, ref st, W, H, D, RW, RH, z, ref lc, ref zn);
                if (zn + mind > D) zn = D;                          // Nothing fits between zn and D;
                vol += area * (long)(zn - z);                       // Update volume;
                if (sl != sk - 1) CheckDominance(s1, sk, sl, st);   // Check for dominance;
                sk = sl + 1; sl = st;
                if (z == 0) s0 = st;
                z = zn;
            }
            RemoveDominated(s1, s0 + 1, sl, ref sm);                // Remove "false" corner points;
            return vol;                                             // Bound is current filling + all free vol;
        }

        public static void RecursiveRobotPacking(AllInfo a, Box[] t, int f, int l, int miss, long fill)
        {
            /* ======================================================================
                            RecursiveRobotPacking
               ====================================================================== */

            // Recursive algorithm for solving a knapsack filling of a single bin.
            // In each iteration, the set of feasible positions for placing a new
            // box is found, and an upper bound on the filling is derived. If the
            // bound indicates that an improved solution still may be obtained, every
            // box is tried to be placed at every feasible position, before calling
            // the algorithm recursively.

            if (Stopped) return;
            a.IterThreeD++;

            if ((a.IterThreeD == a.MaxIter) && (a.MaxIter != 0)) Terminate = true;
            if (a.IterThreeD % IUnit == 0) CheckTimeLimit(a.TimeLimit, a.GlobStartTime);

            a.SubIterat++;
            if (a.SubIterat == IUnit)
            {
                a.SubIterat = 0;
                a.Iterat++;
                CheckIterLimit(a.Iterat, a.IterLimit);
            }
            if (Terminate) return;


            int m = Diff(f, l);

            // Find min/max dimensions of remaining boxes;
            if (miss == 0)
            {
                // None left, wich is good. => Save solution.
                for (int i1 = 0; i1 < m; i1++)
                {
                    a.BoxesCurSol[a.FSol + i1] = t[i1];
                }
                a.MaxFill = a.BinVol;
                Terminate = true;
                a.Miss = miss;
                return;
            }
            else
            {
                int d;
                long bound = fill + a.BinVol;
                int sl = 0;
                Point[] s1 = new Point[3 * a.N];

                // Check if actual filling is a better filling;
                if (fill > a.MaxFill)
                {
                    for (int i = 0; i < m; i++)
                    {
                        a.BoxesCurSol[a.FSol + i] = t[i];
                    }
                    a.MaxFill = fill;
                    a.Miss = miss;
                }                

                // Find bound and positions to place new boxes;
                bound -= FindPlaces(t, s1, a, f, l, ref sl);
                
                if (bound > a.MaxFill)
                {
                    // For each position in S, try to place an box there;
                    for (int s = 0; s != sl + 1; s++)
                    {
                        d = 0;
                        for (int i = f; i != l + 1; i++)
                        {
                            if (t[i].K != 0) continue; // Already chosen;

                            // See if box fits at position s;
                            if ((s1[s].X) + t[i].W > a.W) continue;
                            if ((s1[s].Y) + t[i].H > a.H) continue;
                            if ((s1[s].Z) + t[i].D > a.D) continue;

                            // Place box and call the algorithm recursively;
                            t[i].K = 1;
                            t[i].X = s1[s].X;
                            t[i].Y = s1[s].Y;
                            t[i].Z = s1[s].Z;

                            RecursiveRobotPacking(a, t, f, l, miss - 1, fill + t[i].Vol);

                            t[i].K = 0;
                            t[i].X = t[i].Y = t[i].Z = 0;
                            d++;
                            if (d == a.MCut || Terminate) break;
                        }
                        if (d == a.MCut || Terminate) break;
                    }
                }
            }
        }

        public static bool OneBinRobotPacking(AllInfo a, Box[] boxes, int f, int l, bool fast)
        {
            /* ======================================================================
                            OneBinRobotPacking
               ====================================================================== */

            // Knapsack filling of a single bin. The following procedure initializes
            // some variables before calling the recursive branch-and-bound algorithm.

            int m = l + 1;
            long vol = 0;
            int n = Diff(f, l);

            // Initialize the boxes;
            for (int i = f; i != m; i++)
            {
                boxes[i].X = 0;
                boxes[i].Y = 0;
                boxes[i].Z = 0;
                boxes[i].K = 0;
                vol += boxes[i].Vol;
            }

            // Try to fill one bin with boxes f..l
            a.IterThreeD = 0;
            a.MaxFill = vol - 1;                // Lower bound equal to all minus one, only packings where all boxes fit into one bin are of interest;
            a.Miss = n;
            a.MaxIter = (fast ? MaxIter : 0);   // Limited or infinitly many iterations;
            a.MCut = 0;                         // Try all branches;
            Terminate = false;

            // store the time;
            time = DateTime.Now;
            double ts = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;

            // Calling branch-and-bound algorithm;
            RecursiveRobotPacking(a, boxes, f, l, n, 0);

            time = DateTime.Now;
            double te = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;
            // Store the elapsed time;
            a.RobotTime += te - ts;


            // Copy solution;
            if (a.MaxFill == a.BinVol)
            { // NB: maxfill = BVOL, when optimal found;
                for (int i = a.FSol, j = f; j != m; i++, j++)
                {
                    boxes[j] = a.BoxesCurSol[i];
                }
                return true;
            }
            return false;
        }

        #endregion

        #region branch-and-bound for 3DBPP

        public static bool TwoBoxesFit(Box fi, ref Box fj, int W, int H, int D)
        {
            // Specialized algorithm for case with two boxes;
            // All coordinates are initialized to zero, so just adjust changes!;
            // The 2-box solution is always guillotine cuttable;

            if (fi.W + fj.W <= W) { fj.X = fi.W; return true; }
            if (fi.H + fj.H <= H) { fj.Y = fi.H; return true; }
            if (fi.D + fj.D <= D) { fj.Z = fi.D; return true; }
            return false;
        }

        public static bool ThreeBoxesFit(ref Box fi, ref Box fj, ref Box fk, int W, int H, int D)
        {
            //  Specialized algorithms for case with three boxes;

            int w = W - fk.W; int h = H - fk.H; int d = D - fk.D;
            int bijw = fi.W + fj.W; int bijh = fi.H + fj.H; int bijd = fi.D + fj.D;

            // All coordinates are initialized to zero, so just adjust changes!;
            // The 3-box solution can either be cut by guillotine cuts;

            
            if ((fi.W <= w) && (fj.W <= w))
            {
                if (bijw <= w) { fj.X = fi.W; fk.X = w; return true; }
                if (bijh <= H) { fj.Y = fi.H; fk.X = w; return true; }
                if (bijd <= D) { fj.Z = fi.D; fk.X = w; return true; }
            }
            if ((fi.H <= h) && (fj.H <= h))
            {
                if (bijw <= W) { fj.X = fi.W; fk.Y = h; return true; }
                if (bijh <= h) { fj.Y = fi.H; fk.Y = h; return true; }
                if (bijd <= D) { fj.Z = fi.D; fk.Y = h; return true; }
            }
            if ((fi.D <= d) && (fj.D <= d))
            {
                if (bijw <= W) { fj.X = fi.W; fk.Z = d; return true; }
                if (bijh <= H) { fj.Y = fi.H; fk.Z = d; return true; }
                if (bijd <= d) { fj.Z = fi.D; fk.Z = d; return true; }
            }
            w = W - fi.W; h = H - fi.H; d = D - fi.D;
            int bjkw = fj.W + fk.W; int bjkh = fj.H + fk.H; int bjkd = fj.D + fk.D;            
            if ((fj.W <= w) && (fk.W <= w))
            {
                if (bjkw <= w) { fk.X = fj.W; fi.X = w; return true; }
                if (bjkh <= H) { fk.Y = fj.H; fi.X = w; return true; }
                if (bjkd <= D) { fk.Z = fj.D; fi.X = w; return true; }
            }
            if ((fj.H <= h) && (fk.H <= h))
            {
                if (bjkw <= W) { fk.X = fj.W; fi.Y = h; return true; }
                if (bjkh <= h) { fk.Y = fj.H; fi.Y = h; return true; }
                if (bjkd <= D) { fk.Z = fj.D; fi.Y = h; return true; }
            }
            if ((fj.D <= d) && (fk.D <= d))
            {
                if (bjkw <= W) { fk.X = fj.W; fi.Z = d; return true; }
                if (bjkh <= H) { fk.Y = fj.H; fi.Z = d; return true; }
                if (bjkd <= d) { fk.Z = fj.D; fi.Z = d; return true; }
            }
            w = W - fj.W; h = H - fj.H; d = D - fj.D;
            int bkiw = fk.W + fi.W; int bkih = fk.H + fi.H; int bkid = fk.D + fi.D;
            if ((fk.W <= w) && (fi.W <= w))
            {
                if (bkiw <= w) { fi.X = fk.W; fj.X = w; return true; }
                if (bkih <= H) { fi.Y = fk.H; fj.X = w; return true; }
                if (bkid <= D) { fi.Z = fk.D; fj.X = w; return true; }
            }
            if ((fk.H <= h) && (fi.H <= h))
            {
                if (bkiw <= W) { fi.X = fk.W; fj.Y = h; return true; }
                if (bkih <= h) { fi.Y = fk.H; fj.Y = h; return true; }
                if (bkid <= D) { fi.Z = fk.D; fj.Y = h; return true; }
            }
            if ((fk.D <= d) && (fi.D <= d))
            {
                if (bkiw <= W) { fi.X = fk.W; fj.Z = d; return true; }
                if (bkih <= H) { fi.Y = fk.H; fj.Z = d; return true; }
                if (bkid <= d) { fi.Z = fk.D; fj.Z = d; return true; }
            }
            // (xi,yi,zi) = (0,0,0); (xj,yj,zj) = (wi,0,0); (xk,yk,zk) = (0,hi,dj)
            if ((bijw <= W) && (bkih <= H) && (bjkd <= D))
            {
                fj.X = fi.W; fk.Y = fi.H; fk.Z = fj.D; return true;
            }
            // (xi,yi,zi) = (0,0,0); (xj,yj,zj) = (wk,0,di); (xk,yk,zk) = (0,hi,0)
            if ((bjkw <= W) && (bkih <= H) && (bijd <= D))
            {
                fj.X = fk.W; fj.Z = fi.D; fk.Y = fi.H; return true;
            }
            // (xi,yi,zi) = (0,0,0); (xj,yj,zj) = (0,hi,dk); (xk,yk,zk) = (wi,0,0)
            if ((bkiw <= W) && (bijh <= H) && (bjkd <= D))
            {
                fj.Y = fi.H; fj.Z = fk.D; fk.X = fi.W; return true;
            }
            // (xi,yi,zi) = (0,0,0); (xj,yj,zj) = (0,hi,0); (xk,yk,zk) = (wj,0,di)
            if ((bjkw <= W) && (bijh <= H) && (bkid <= D))
            {
                fj.Y = fi.H; fk.X = fj.W; fk.Z = fi.D; return true;
            }
            // (xi,yi,zi) = (0,0,0); (xj,yj,zj) = (wi,0,0); (xk,yk,zk) = (0,hj,di)
            if ((bijw <= W) && (bjkh <= H) && (bkid <= D))
            {
                fj.X = fi.W; fk.Y = fj.H; fk.Z = fi.D; return true;
            }
            // (xi,yi,zi) = (0,0,0); (xj,yj,zj) = (0,0,di); (xk,yk,zk) = (wi,hj,0)
            if ((bkiw <= W) && (bjkh <= H) && (bijd <= D))
            {
                fj.Z = fi.D; fk.X = fi.W; fk.Y = fj.H; return true;
            }
            return false;
        }

        public static bool ManyBoxesFit(AllInfo a, Box[] t, int k, bool fast)
        {
            // The routine ManyBoxesFit checks whether a given subset of boxes fits into a single bin;

            bool lb = (BoundTwo(a, t, 0, k) > 1 ? true : false);
            if (lb) return false;
            a.CallsFitsMore++;
            if (a.PackType == General)
            {
                return OneBinGeneralPacking(a, t, 0, k, fast);
            }
            if (a.PackType == Robot)
            {
                return OneBinRobotPacking(a, t, 0, k, fast);
            }
            return false;
        }

        public static bool OneBinDecision(AllInfo a, int j, int bno)
        {
            /* ======================================================================
                            OneBinDecision
               ====================================================================== */

            // The following procedure checks whether a new box "j" fits into the the
            // bin "bno" (together with already placed boxes in the bin). If the answer
            // is "no" then definitly it is not possible to place the box into the bin.

            int k = -1;
            bool fits;
            for(int i = 0; i < j + 1; i++)
            {
                t[i] = new Box();
            }

            for (int i = a.FBox; i != j; i++)
            {
                if (a.Boxes[i].Bno == bno)
                {
                    k++;
                    t[k] = a.Boxes[i];
                    if (k <= 3) t[k].X = t[k].Y = t[k].Z = 0;
                    t[k].RefBoxInd = i;
                }
            }

            k++;
            t[k] = a.Boxes[j]; ;
            t[k].X = t[k].Y = t[k].Z = 0; t[k].K = 1;
            t[k].RefBoxInd = j;

            int size = Diff(0, k);

            switch (size)
            {
                case 0: throw new Exception("No boxes in OneBinDecision.");
                case 1: fits = true; t[k].X = t[k].Y = t[k].Z = 0; break;
                case 2:
                    {
                        fits = TwoBoxesFit(t[0], ref t[k], a.W, a.H, a.D);
                        a.CallsFitsTwo++;
                        break;
                    }
                case 3:
                    {
                        fits = ThreeBoxesFit(ref t[0], ref t[1], ref t[k], a.W, a.H, a.D);
                        a.CallsFitsThree++;
                        break;
                    }
                default:
                    {
                        fits = ManyBoxesFit(a, t, k, false);
                        break;
                    }

            }
            if (size <= 3)
            {
                a.SubIterat++;
                if (a.SubIterat == IUnit)
                {
                    a.SubIterat = 0;
                    a.Iterat++;
                    CheckIterLimit(a.Iterat, a.IterLimit);
                }
            }

            if (fits)
            {
                int r;
                // Copy solution back;
                for (int i = 0; i < k + 1; i++)
                {
                    r = t[i].RefBoxInd;
                    a.Boxes[r].X = t[i].X;
                    a.Boxes[r].Y = t[i].Y;
                    a.Boxes[r].Z = t[i].Z;
                    a.Boxes[r].K = 1;
                }
            }
            return fits;
        }

        public static bool OneBinHeuristic(AllInfo a, Box[] f, int l)
        {

            /* ======================================================================
                            OneBinHeuristic
               ====================================================================== */

            // This is a heuristic version of "OneBinDecision". If "fits = true"
            // then a heuristic solution has been found where boxes f,..,l fit into a
            // bin. Otherwise a filling may still be possible, but it
            // was not found by the heuristic.

            int m = l + 1;

            if (l <= 3) for (int i = 0; i != m; i++) { f[i].X = f[i].Y = f[i].Z = 0; }

            switch (Diff(0, l))
            {
                case 0: throw new Exception("No boxes in OneBinHeuristic.");
                case 1: return true;
                case 2:
                    {
                        a.CallsFitsTwo++;
                        return TwoBoxesFit(f[0], ref f[l], a.W, a.H, a.D);

                    }
                case 3:
                    {
                        a.CallsFitsThree++;
                        return ThreeBoxesFit(ref f[0], ref f[1], ref f[l], a.W, a.H, a.D);
                    }
                default:
                    {
                        return ManyBoxesFit(a, f, l, true);
                    }
            }
        }

        public static bool TryCloseBin(AllInfo a, Box[] boxes, ref int curr, int bno,
                        Box[] oldBoxes, ref int oldLast, ref int oldLastClosed, ref int oldNoOfClosed,
                        bool[] oldClosed, int level)
        {

            /* ======================================================================
                            TryCloseBin
               ====================================================================== */

            // A bin may be closed if no more boxes fit into it. The present version
            // uses a more advanced criterion: First, the set of boxes which fit into
            // bin "bno" is derived. This is done, by testing whether each box (alone)
            // fits together with the already placed boxes in the bin. Having derived
            // the set of additional boxes that (individually) fits into the bin, it is
            // tested whether a solution exists where all the additional boxes are
            // placed in the bin. If this is the case, we have found a optimal placing
            // of the additional boxes, and thus we may close the bin.

            if (level > MaxClose) return false;

            int j, m, k, i, s;
            int r;
            int n, b;
            long vol;
            bool didClose, fits;

            for (j = a.FBox; j < curr + 1; j++)
            {
                f[j] = new Box();
            }
            i = curr;
            didClose = false;

            for (b = 1; b <= bno; b++)
            {
                if (i > a.LBox) break;
                if (a.Closed[b]) continue;

                k = 0; n = 0; vol = 0;
                for (j = a.FBox; j != i; j++)
                {
                    if (boxes[j].Bno == b)
                    {
                        f[k] = boxes[j];
                        f[k].RefBoxInd = j;
                        k++;
                        vol += boxes[j].Vol;
                    }
                }
                n = k;

                if (n == 0)
                {
                    throw new Exception("Bin with no boxes, TryClose.");
                }
                if (vol < a.BinVol / 2) continue;

                for (j = i, m = a.LBox + 1; j != m; j++)
                {
                    if ((boxes[j].No > a.N) || (boxes[j].No < 1))
                    {
                        throw new Exception("Bad Number, TryClose.");
                    }

                    fits = OneBinDecision(a, j, b); // Start the inner branching scheme;

                    if (fits)
                    {
                        f[k] = boxes[j];
                        f[k].RefBoxInd = j;
                        k++;
                        vol += boxes[j].Vol;
                    }

                    if (vol > a.BinVol) break;
                }
                if (vol > a.BinVol) continue;
                if (OneBinHeuristic(a, f, k - 1))
                {
                    if (!didClose)
                    {
                        // Take backup of table when first bin closed;

                        for (int i1 = 0; i1 < bno + 1; i1++)
                        {
                            oldClosed[i1] = a.Closed[i1];
                        }
                        for (int i1 = a.FBox; i1 < Diff(a.FBox, a.LBox); i1++)
                        {
                            oldBoxes[i1] = boxes[i1];
                        }
                        oldLast = a.LBox;
                        oldLastClosed = a.LClosed;
                        oldNoOfClosed = a.NumOfClosed;
                    }

                    a.Closed[b] = true;
                    s = a.LClosed;
                    didClose = true;
                    a.NumOfClosed++;

                    if (a.NumOfClosed > a.MaxClose) a.MaxClose = a.NumOfClosed;

                    for (j = 0; j != k; j++)
                    {
                        r = f[j].RefBoxInd;     // Get index of original box;
                        boxes[r].Bno = b;
                        boxes[r].K = 1;
                        boxes[r].X = f[j].X;
                        boxes[r].Y = f[j].Y;
                        boxes[r].Z = f[j].Z;
                    }
                    m = a.LBox + 1;

                    for (j = k = a.FBox; j != m; j++)
                    {
                        if (boxes[j].Bno == b)
                        {
                            s++;
                            a.BoxesClosedBin[s] = boxes[j];
                        }
                        else
                        {
                            boxes[k] = boxes[j];
                            k++;
                        }
                    }
                    a.LBox = k - 1;
                    a.LClosed = s;
                    i -= n;                     // Reposition current box;
                }
            }
            curr = i;
            return didClose;
        }

        public static void RecursiveBinPack(AllInfo a, Box[] boxes, int i, int bno, long lb, int level)
        {
            /* ======================================================================
                                    RecursiveBinPack
            ====================================================================== */
            // The outer branching scheme;
            // Recursive algorithm for 3D Bin-packing Problem. In each iteration, the
            // next box "i" is assigned to every open bin, as well as to a new bin.

            if (bno >= a.Z) return;                    // Used too many bins;
            if (a.Z == a.Lb) return;        // Optimal solution found

            a.SubNodes++;

            if (a.SubNodes == IUnit) { a.SubNodes = 0; a.Nodes++; }
            CheckTimeLimit(a.TimeLimit, a.GlobStartTime);
            CheckNodeLimit(a.Nodes, a.NodeLimit);
            CheckIterLimit(a.Iterat, a.IterLimit);

            if (Stopped) return;

            Box[] oldBoxes = new Box[a.N+1];

            int oldLast = 0, oldLastClosed = 0, oldNoOfClosed = 0; ;
            bool[] oldClosed = new bool[a.N+1];
            
            if (i == a.LBox + 1)
            {
                // All boxes assigned, must be a better solution;
                SaveSolution(a, boxes, a.FBox, a.LBox, bno);
            }
            else
            {
                bool more = TryCloseBin(a, boxes, ref i, bno, oldBoxes, ref oldLast, ref oldLastClosed, ref oldNoOfClosed, oldClosed, level);

                if (i == a.LBox + 1)
                {
                    // All boxes went into closed bins;
                    SaveSolution(a, boxes, a.FBox, a.LBox, bno);
                }
                else
                {
                    if (more) lb = a.NumOfClosed + BoundTwo(a, boxes, a.FBox, a.LBox);
                    if (lb < a.Z)
                    {
                        for (int b = 1; b <= bno; b++)
                        {
                            if (a.Closed[b]) continue;          // Cannot add to closed bin;
                            if (OneBinDecision(a, i, b))
                            {
                                boxes[i].Bno = b;
                                RecursiveBinPack(a, boxes, i + 1, bno, lb, level + 1);
                                boxes[i].Bno = 0;
                            }
                        }

                        boxes[i].Bno = bno + 1;
                        boxes[i].X = boxes[i].Y = boxes[i].Z = 0;
                        a.Closed[boxes[i].Bno] = false;

                        RecursiveBinPack(a, boxes, i + 1, bno + 1, lb, level + 1);

                        boxes[i].Bno = 0;
                    }
                }
                // Restore;
                if (more)
                {
                    // Reopen a closed bin when backtracking.
                    a.LBox = oldLast;
                    a.LClosed = oldLastClosed;
                    a.NumOfClosed = oldNoOfClosed;

                    Array.Copy(oldBoxes, 0, boxes, a.FBox, Diff(a.FBox, oldLast));

                    for (int i1 = 0; i1 < bno + 1; i1++)
                    {
                        a.Closed[i1] = oldClosed[i1];
                    }

                }
            }
        }
        #endregion

        #region Main procedure

        public static void ClearBoxes(AllInfo a, Box[] boxes)
        {
            //  Reset fields x,y,z and k
            int i, m = a.LBox + 1;

            for (i = a.FBox; i != m; i++)
            {
                boxes[i].X = boxes[i].Y = boxes[i].Z = boxes[i].Bno = 0; boxes[i].K = 0;
            }
            // Sort by nonincreasing volume;
            Array.Sort(boxes, 0, a.LBox + 1, new BoxVComparer());
        }

        public static void ReturnBoxes(Box[] boxOptS, int[] x, int[] y, int[] z, int[] bno)
        {
            // Write solutions to the x,y,z and bno arrays;

            int m = boxOptS.Length - 1;
            int k;

            for (int i = 0; i != m; i++)
            {
                k = boxOptS[i].No - 1;
                x[k] = boxOptS[i].X;
                y[k] = boxOptS[i].Y;
                z[k] = boxOptS[i].Z;
                bno[k] = boxOptS[i].Bno;
            }
        }

        public static AllInfo BinPack3D(int n, int W, int H, int D, int[] w, int[] h, int[] d,
               int[] x, int[] y, int[] z, int[] bno, int nodeLimit, int iterLimit, int timeLimit, int packingType)
        {
            // First check;

            if ((packingType != Robot) && (packingType != General))
            {
                throw new Exception("Bad packtype, BinPack3D.");
            }
            if (n + 1 > MaxBoxes)
            {
                throw new Exception("Instance to big, BinPack3D.");
            }

            if (packingType == 0)
            {
                // General packing
                StackDepth = n * n * 8;
                DomainStack = new DomainPair[StackDepth];
                Domain = Array.ConvertAll(new int[n], x1 => Array.ConvertAll(new int[n], y1 => new int[8]));
                Relation = Array.ConvertAll(new int[n], x1 => new int[n]);
            }

            // Copy the instance information to an object;
            AllInfo a = new AllInfo(n, W, H, D, w, h, d, nodeLimit, iterLimit, timeLimit, packingType);
            t = new Box[a.N + 1];
            f = new Box[a.N + 1];
            foundCorners = new Point[a.N + 1];

            // store the time;
            time = DateTime.Now;
            a.GlobStartTime = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;

            Stopped = false;

            // Find bounds;
            a.Bound0 = BoundZero(a.BinVol, a.Boxes, a.FBox, a.LBox);
            a.Bound1 = BoundOne(a, a.Boxes, a.FBox, a.LBox);
            a.Bound2 = BoundTwo(a, a.Boxes, a.FBox, a.LBox);
            a.Lb = a.Bound2;

            // Initialize search limits for exact search;
            a.NodeLimit = nodeLimit;
            a.IterLimit = iterLimit;
            a.TimeLimit = timeLimit;     

            // Outer tree enummeration;
            RecursiveBinPack(a, a.Boxes, a.FBox, 0, a.Lb, 1);
            a.Stopped = Stopped;
            if (a.Stopped)
            {
                ClearBoxes(a, a.Boxes);
                // Find heuristic solution;
                HeuristicFillThreeDim(a);
            }

            time = DateTime.Now;
            double te = time.Hour * 3600 + time.Minute * 60 + time.Second + (double)time.Millisecond / 1000;
            // Get the elapsed time as a TimeSpan value and store it.
            a.Time = te - a.GlobStartTime;
            a.Stopped = Stopped;

            // Copy boxes back to arrays;
            ReturnBoxes(a.BoxesOptSol, x, y, z, bno);

            return a;
        }
        #endregion
    }
}