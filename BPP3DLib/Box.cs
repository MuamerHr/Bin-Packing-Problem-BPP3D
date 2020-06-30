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

namespace BPP3DLib
{
    public struct Box
    {
        int no;           /* box number                             */
        int w;            /* box width  (x-size)                    */
        int h;            /* box height (y-size)                    */
        int d;            /* box depth  (z-size)                    */
        int x;            /* optimal x-position                     */
        int y;            /* optimal y-position                     */
        int z;            /* optimal z-position                     */
        int bno;          /* bin number                             */
        int k;            /* is the box chosen?                     */
        long vol;         /* volume of box                          */
        int refBoxInd;    /* index of the original box              */

        public int No { get => no; set => no = value; }
        public int W { get => w; set => w = value; }
        public int H { get => h; set => h = value; }
        public int D { get => d; set => d = value; }
        public int X { get => x; set => x = value; }
        public int Y { get => y; set => y = value; }
        public int Z { get => z; set => z = value; }
        public int Bno { get => bno; set => bno = value; }
        public int K { get => k; set => k = value; }
        public long Vol { get => vol; set => vol = value; }
        public int RefBoxInd { get => refBoxInd; set => refBoxInd = value; }

        public Box(int no, int w, int h, int d)
        {
            this.no = no;
            this.w = w;
            this.h = h;
            this.d = d;
            this.x = 0;
            this.y = 0;
            this.z = 0;
            this.bno = 0;
            this.k = 0;
            this.vol = w * h * (long) d;
            this.refBoxInd = no;
        }
    }
}
