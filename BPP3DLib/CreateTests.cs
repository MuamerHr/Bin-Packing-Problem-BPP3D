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

namespace BPP3DLib
{
    public static class CreateTests
    {
        public static void MakeTests(int n, int[] w, int[] h, int[] d, int W, int H, int D, int equalBinDim, ref int instanceType)
        {
            // Main procedure to create random instances
            // If parameter equalBinDim > 0 then all bin dimensions are set to be equal creating a cube of dimensions W*H*D = equalBinDim^3.
            // The parameter instanceType can have values from 1-12, where:
            // 1) instance type 1-11 creates classical instances with bin dimensions W=H=D=100
            // 2) instance type 12  creates instances for packing and loading wood packages

            int i, k, m;

            int f = 0;
            int l = n - 1;
            int no;

            // Set the bin dimensions;
            if (equalBinDim > 0 && instanceType < 12)
            {
                // Create a cube with side length "equalBinDim";
                W = equalBinDim; H = equalBinDim; D = equalBinDim;
            }

            // Create maxtypes box types;
            for (i = f, m = l + 1; i != m; i++)
            {
                randomtype(w, h, d, i, W, H, D, ref instanceType);
            }

            // Guillotine cut three bins;
            if (instanceType == 9)
            {
                no = BPP3D.Diff(f, l) / 3;
                k = f + no; m = k + no;
                while (true)
                {
                    SpecialBin(w, h, d, f, k, W, H, D);
                    SpecialBin(w, h, d, k + 1, m, W, H, D);
                    SpecialBin(w, h, d, m + 1, l, W, H, D);
                    if (InstanceCheck(w, h, d, 3 * equalBinDim * equalBinDim * equalBinDim, f, l)) break;
                }
            }
        }

        public static void randomtype(int[] w, int[] h, int[] d, int i, int W, int H, int D, ref int instanceType)
        {
            // Procedure to create random instances according to specific instance type

            int t;
            Random rand = new Random();

            if (instanceType < 1 || instanceType > 13)
            {   // Instances Martello, Vigo;
                t = rand.Next(1, 12);
                if (t <= 5) instanceType = t;
            }

            switch (instanceType)
            {
                // Instances Martello, Vigo;
                case 1:
                    {
                        w[i] = rand.Next(1, W / 2);
                        h[i] = rand.Next(2 * H / 3, H);
                        d[i] = rand.Next(2 * D / 3, D);
                        break;
                    }
                case 2:
                    {
                        w[i] = rand.Next(2 * W / 3, W);
                        h[i] = rand.Next(1, H / 2);
                        d[i] = rand.Next(2 * D / 3, D);
                        break;
                    }
                case 3:
                    {
                        w[i] = rand.Next(2 * W / 3, W);
                        h[i] = rand.Next(2 * H / 3, H);
                        d[i] = rand.Next(1, D / 2);
                        break;
                    }
                case 4:
                    {
                        w[i] = rand.Next(W / 2, H);
                        h[i] = rand.Next(H / 2, H);
                        d[i] = rand.Next(D / 2, D);
                        break;
                    }
                case 5:
                    {
                        w[i] = rand.Next(1, W / 2);
                        h[i] = rand.Next(1, H / 2);
                        d[i] = rand.Next(1, D / 2);
                        break;
                    }

                // Instances Berkey, Wang;
                case 6:
                    {
                        w[i] = rand.Next(1, 10);
                        h[i] = rand.Next(1, 10);
                        d[i] = rand.Next(1, 10);
                        break;
                    }
                case 7:
                    {
                        w[i] = rand.Next(1, 35);
                        h[i] = rand.Next(1, 35);
                        d[i] = rand.Next(1, 35);
                        break;
                    }
                case 8:
                    {
                        w[i] = rand.Next(1, 100);
                        h[i] = rand.Next(1, 100);
                        d[i] = rand.Next(1, 100);
                        break;
                    }
                //Guillotine cut three bins are created in MakeTests
                case 9:
                    {                        
                        break;
                    }

                // Instances 2D cases;
                case 10:
                    {
                        w[i] = rand.Next(1, W);
                        h[i] = rand.Next(1, H);
                        d[i] = D; break;
                    }
                // Instances 1D cases;
                case 11:
                    {
                        w[i] = rand.Next(1, W / 2);
                        h[i] = H;
                        d[i] = D; break;
                    }

                // Instances Hrncic, truck dimensions 3D;
                case 12:
                    {
                        w[i] = rand.Next(2000, 5000);
                        h[i] = rand.Next(500, 900);
                        d[i] = rand.Next(500, 800);
                        break;
                    }
            }
        }
        public static void SpecialBin(int[] w, int[] h, int[] d, int f, int l, int W, int H, int D)
        {
            // Procedure to create hard random instances whith known solution
            int m, i, j, k;
            int wh, hh, dh;
            Random rand = new Random();

            if (f == l) { w[f] = W; h[f] = H; d[f] = D; return; }
            if (BPP3D.Diff(f, l) == 5)
            {
                wh = W / 3; hh = H / 3; i = f + 1; j = f + 2; k = f + 3;
                w[i] = W - wh; h[i] = hh; d[i] = D;
                w[j] = wh; h[j] = H - hh; d[j] = D;
                w[k] = W - wh; h[k] = hh; d[k] = D;
                w[l] = wh; h[l] = H - hh; d[l] = D;
                w[f] = W - 2 * wh; h[f] = H - 2 * hh; d[f] = D;
                return;
            }

            m = f + (l - f) / 2;
            while (true)
            {
                switch (rand.Next(1, 3))
                {
                    case 1:
                        if (W < 2) break;

                        wh = rand.Next(1, W - 1);

                        SpecialBin(w, h, d, f, m, wh, H, D);
                        SpecialBin(w, h, d, m + 1, l, W - wh, H, D);
                        return;
                    case 2:
                        if (H < 2) break;

                        hh = rand.Next(1, H - 1);

                        SpecialBin(w, h, d, f, m, W, hh, D);
                        SpecialBin(w, h, d, m + 1, l, W, H - hh, D);
                        return;
                    case 3:
                        if (D < 2) break;

                        dh = rand.Next(1, D - 1);

                        SpecialBin(w, h, d, f, m, W, H, dh);
                        SpecialBin(w, h, d, m + 1, l, W, H, D - dh);
                        return;
                }
            }
        }

        public static bool InstanceCheck(int[] w, int[] h, int[] d, long totvol, int f, int l)
        {
            int j, m;
            long vol = 0;

            for (j = f, m = l + 1; j != m; j++)
            {
                if ((w[j] < 1) || (h[j] < 1) || (d[j] < 1)) return false;
                vol += w[j] * h[j] * d[j];
            }
            return (vol == totvol);
        }
    }
}
