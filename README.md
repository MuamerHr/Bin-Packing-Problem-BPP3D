# BPP3D-Bin-Packing-Problem

This repository contains procedures to solve the bin packing problem for 
one, two, or three dimensions.

The three dimensional bin packing problem belongs to the 
class of NP complete problems and can be formulated as follows:

Let be J={1,...,n} a set of small rectangular items with dimensions w_i, h_i, d_i.
All small items must be packed into bins of dimensions W,H,D.
All dimensions are positive integers and the dimensions of the small
items are smaller than or equal to the corresponding bin dimensions. 

The task is to pack all the small items, 
such that:

1) All small items are packed into a minimum number of bins
2) No two items overlap.

##################################################################################################

This algorithm solves the problem for orthogonal packings, which means the items are packed,
such that the edges of the small items are parallel to the correspoding bin edge.
No rotations of items or bins are allowed. The algorithm is capable to solve the problem 
for general packings and robot packings.

The algorithm is a combination of two branch and bound schemes where:
- the outer branching scheme assigns small items to bins without specifying the actual positions
- the inner branching scheme packs a given subset of items into one bin depending on the chosen
packing pattern (general, robot).

##################################################################################################

The main method of the algorithm is the method BinPack3D(...). It
returns an object of type AllInfo which contains all the information
and results of the executed algorithm.

The parameter description is the following
int n	...			Amount of small items
int W...			Width of the bin
int H...			Height of the bin
int D...			Depth of the bin
int[] w...			Widths of the small items
int[] h...			Heights of the small items
int[] d...			Depths of the small items
int[] x...			Resulting x-coordinates of the small items
int[] y...			Resulting y-coordinates of the small items
int[] z...			Resulting z-coordinates of the small items
int[] bno...		Resulting bin number of the small items
int nodeLimit...	Limit for  number of exploration nodes in the outer branching scheme.
					If set to zero, the algorithm tries to find an optimal
					solution, or terminates if "iterLimit" or "timeLimit" is reached.
					"nodeLimit" is a multiple of thousand, e.g. if nodeLimit=1000,
					the explores at most "nodeLimit * IUnit" nodes (IUnit = 1000).
int iterLimit...	Limit for number of iterations in the inner branching schemes.
					If set to zero, the algorithm tries to find an optimal
					solution, or terminates if "nodeLimit" or "timeLimit" is reached.
					"iterLimit" is a multiple of thousand, e.g. if iterLimit=1000,
					the explores at most "iterLimit * IUnit" nodes (IUnit = 1000).
int timeLimit...	Time limit for algorithm. If set to zero, the algorithm tries to find an optimal
					solution, or terminates if "nodeLimit" or "iterLimit" is reached.
					If the timelimit is reached the algorithm return a heuristic solution.				
int packingType...	If set to "0" the algorithm tries to find an optimal solution
					using general packing patterns.
					If set to "1" the algorithm tries to find an optimal solution
					using robot packing pattern.

##################################################################################################

For a detailed description of the algorithm and original C-Code refer to:

[1] Silvano Martello, David Pisinger, and Daniele Vigo. "The three-dimensional
bin packing problem." Oper. Res., 48(2):256–267, March
2000.

[2] Silvano Martello, David Pisinger, Daniele Vigo, Edgar Boef, and Jan Korst.
"Algorithm 864: General and robot-packable variants of the threedimensional
bin packing problem." ACM Trans. Math. Softw., 33:7, 01
2007.

##################################################################################################

This C# code is a reimplementation of the original C-Code provided
by Daniel Pisinger at http://hjemmesider.diku.dk/~pisinger/codes.html.
The code comes with no warranty.
