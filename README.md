# BPP3D-Bin-Packing-Problem

This repository contains procedures to solve the bin packing problem for 
one, two, or three dimensions.

The three dimensional bin packing problem belongs to the 
class of NP complete problems and can be formulated as follows:

Let be J={1,...,n} a set of small rectangular items with dimensions w_i,h_i,d_i.
All small items must be packed into bins of dimensions W,H,D.
All dimensions are positive integers and the dimensions of the small
items are smaller than or equal to the corresponding bin dimensions. 

The task is to pack all the small items, 
such that:

1) All small items are packed into a minimum number of bins
2) No two items overlap.

The algorithm solves the problem for orthogonal packings, which means the items are packed,
such that the edges of the small items are parallel to the correspoding bin edge.
No rotations of items or bins are allowed. The algorithm is capable to solve the problem 
for general packings and robot packings.

For a detailed description of the algorithm and original C-Code refer to:

[1] Silvano Martello, David Pisinger, and Daniele Vigo. "The three-dimensional
bin packing problem." Oper. Res., 48(2):256–267, March
2000.

[2] Silvano Martello, David Pisinger, Daniele Vigo, Edgar Boef, and Jan Korst.
"Algorithm 864: General and robot-packable variants of the threedimensional
bin packing problem." ACM Trans. Math. Softw., 33:7, 01
2007.

This C# code is a reimplementation of the original C-Code provided
by Daniel Pisinger at http://hjemmesider.diku.dk/~pisinger/codes.html.
This code comes with no warranty.
