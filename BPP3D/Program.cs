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
using System.IO;
using BPP3DLib;
using System.Threading;

namespace BPP3DApp
{
    public class Program
    {
        // Main Programm to test the bin packing algorithm.
        // The procedure contains examples how to use the algorithm.
        static void Main(string[] args)
        {
            
            Console.WriteLine("######################################################################################");
            Console.WriteLine("The algorithm is tested with fixed values.");
            RunStaticTest();

            Console.ReadKey();
            Console.WriteLine("######################################################################################");
            Console.WriteLine("The algorithm is tested with random values.");
            {
                int n = 10;
                int instanceType = 1;
                int timeLimit = 600;
                int instLimit = 1;
                string outputPath = @"C:\temp\";
                try
                {
                    RunRandomTest(n, instanceType, timeLimit, instLimit, outputPath);
                }
                catch(Exception e)
                {
                    Console.WriteLine("Some error occured.");
                    Console.WriteLine(e.Message);
                }
            }
            Console.ReadKey();
            Console.WriteLine("######################################################################################");
            Console.WriteLine("A file is read and the algorithm is tested.");
            {
                string instancePath = @"C:\temp\myTest.txt";
                try { 
                    RunReadTest(instancePath);
                }
                catch (Exception e)
                {
                    Console.WriteLine("Some error occured.");
                    Console.WriteLine(e.Message);
                }
            }
            Console.ReadKey();
        }
        public static void RunStaticTest()
        {
            // Run the algorithm for the bin and box dimensions given down below.

            int n = 10;
            int W = 100;
            int H = 100;
            int D = 100;
            int[] w;
            int[] h;
            int[] d;

            //n=10;
            w = new int[] { 93,79,86,79,72,91,84,90,66,95};
            h = new int[] { 82,84,93,70,87,69,84,72,66,67};
            d = new int[] { 8,26,47,47,8,26,13,14,19,2};

            //n=10;
            //w = new int[] { 93, 99, 95, 69, 78, 72, 93, 75, 86, 62 };
            //h = new int[] { 59, 97, 58, 74, 94, 57, 90, 60, 87, 83 };
            //d = new int[] { 69, 82, 90, 60, 58, 96, 73, 75, 84, 71 };

            //n=20;
            //w = new int[] { 35,34,49,47,44,6,29,38,10,33,17,33,22,22,44,46,24,12,25,29};
            //h = new int[] { 92,83,86,88,70,90,75,83,69,86,67,82,91,69,80,81,77,77,73,74};
            //d = new int[] { 96,90,75,67,77,87,71,71,98,68,78,96,93,92,75,66,87,85,82,84};

            //n=30
            //w = new int[] { 27, 31, 19, 24, 46, 39, 20, 37, 38, 6, 44, 7, 11, 34, 30, 16, 2, 5, 45, 45, 18, 49, 20, 32, 7, 46, 31, 48, 24, 35 };
            //h = new int[] { 10, 8, 2, 30, 16, 11, 19, 1, 35, 40, 12, 38, 8, 33, 26, 42, 39, 2, 30, 20, 12, 43, 43, 5, 23, 6, 45, 39, 29, 35 };
            //d = new int[] { 47, 24, 10, 22, 12, 47, 4, 38, 1, 18, 21, 21, 26, 45, 32, 49, 26, 46, 14, 36, 26, 7, 28, 8, 36, 31, 20, 17, 1, 41 };
            //n=60;
            //w = new int[] { 89, 77, 68, 81, 76, 96, 78, 72, 80, 68, 89, 88, 95, 86, 80, 86, 96, 96, 74, 92, 86, 89, 81, 67, 78, 76, 69, 69, 80, 72, 97, 79, 69, 74, 68, 97, 93, 68, 95, 91, 78, 71, 68, 84, 83, 71, 92, 97, 96, 99, 93, 94, 71, 93, 71, 91, 87, 91, 94, 91 };
            //h = new int[] { 89, 94, 67, 69, 70, 96, 70, 82, 92, 90, 87, 89, 89, 80, 91, 88, 68, 81, 69, 88, 72, 77, 96, 88, 90, 90, 86, 97, 97, 74, 68, 95, 86, 84, 97, 86, 88, 80, 95, 81, 77, 86, 82, 77, 85, 82, 78, 86, 90, 86, 85, 98, 89, 79, 68, 72, 91, 72, 72, 74 };
            //d = new int[] { 27, 38, 35, 29, 36, 46, 8, 29, 9, 45, 36, 1, 20, 4, 19, 37, 47, 26, 6, 30, 47, 9, 22, 8, 15, 48, 14, 26, 11, 44, 44, 49, 29, 35, 30, 2, 1, 4, 11, 45, 49, 42, 46, 45, 27, 27, 34, 10, 27, 18, 40, 45, 13, 28, 12, 7, 6, 15, 4, 26 };

            //n=70;
            //w = new int[] { 67, 74, 95, 73, 57, 81, 6, 79, 2, 2, 99, 5, 51, 35, 67, 13, 49, 43, 40, 22, 72, 48, 13, 19, 90, 31, 12, 37, 70, 71, 67, 58, 86, 92, 55, 53, 61, 35, 6, 23, 52, 34, 17, 24, 1, 59, 95, 24, 58, 51, 83, 78, 47, 6, 86, 91, 34, 50, 55, 99, 43, 41, 82, 3, 7, 15, 90, 32, 53, 86 };
            //h = new int[] { 2, 8, 48, 53, 85, 4, 73, 19, 27, 9, 27, 25, 83, 64, 45, 62, 75, 63, 2, 99, 98, 93, 18, 55, 32, 25, 55, 61, 53, 77, 58, 77, 33, 59, 9, 21, 13, 87, 93, 13, 93, 51, 89, 63, 54, 28, 61, 7, 25, 11, 28, 59, 97, 10, 45, 89, 71, 74, 3, 42, 31, 93, 47, 31, 32, 31, 73, 6, 38, 6 };
            //d = new int[] { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

            //n=70;
            //w = new int[] { 35, 83, 53, 44, 29, 54, 99, 91, 28, 76, 91, 4, 21, 74, 3, 27, 34, 20, 26, 78, 58, 83, 54, 35, 25, 69, 38, 26, 27, 71, 54, 79, 76, 82, 16, 77, 89, 23, 59, 24, 76, 21, 9, 51, 22, 2, 57, 9, 92, 78, 80, 99, 51, 61, 64, 24, 3, 90, 91, 33, 88, 47, 38, 29, 94, 84, 99, 11, 32, 97 };
            //h = new int[] { 36, 65, 70, 78, 38, 3, 40, 81, 95, 2, 97, 58, 6, 20, 4, 43, 21, 58, 86, 37, 48, 96, 49, 3, 51, 6, 58, 85, 35, 24, 13, 78, 61, 23, 46, 77, 59, 96, 50, 91, 86, 75, 29, 61, 51, 95, 11, 65, 49, 53, 59, 33, 11, 8, 79, 21, 97, 49, 9, 14, 27, 37, 28, 53, 70, 1, 78, 64, 55, 8 };
            //d = new int[] { 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100 };

            int[] x = new int[n];
            int[] y = new int[n];
            int[] z = new int[n];
            int[] bno = new int[n];

            AllInfo a;
            int packingtype = 1;
            int nodelimit = 0;
            int iterlimit = 0;
            int timelimit = 0;

            a = BPP3D.BinPack3D(n, W, H, D, w, h, d, x, y, z, bno, nodelimit, iterlimit, timelimit, packingtype);
            ShowResultsOnConsole(a, w, h, d, x, y, z, bno);
        }

        public static void RunRandomTest(int n = 10, int instanceType = 1, int timeLimit = 600, int instLimit = 1, string outputPath = @"C:\temp\")
        {
            // Create random instance with for the three dimensional bin packing problem. 
            // The bin dimensions are assigned in procedure "CreateRandomTest".
            // For n and the instance Type "instLimit" instances are created.
            // The algorithm is run twice for every instance in order to find a robot packing and a general packing.

            for (int actInstance = 1; actInstance <= instLimit; actInstance++)
            {
                Console.WriteLine("#########################################################");
                Console.WriteLine("Actual instance:");
                Console.WriteLine("n = " + n.ToString());
                Console.WriteLine("Instance type = " + instanceType.ToString());
                Console.WriteLine("Instance number= " + actInstance.ToString());
                CreateRandomTest(n, instanceType, actInstance, timeLimit, outputPath);
                Console.WriteLine("Actual instance finished. See output files for details.");
                Thread.Sleep(1000);
            }
            Console.WriteLine("#########################################################");
            Console.WriteLine("Finished all!");
            Console.ReadKey();
        }

        public static void CreateRandomTest(int n, int instanceType, int actInstance, int timeLimit, string outputPath)
        {
            int W, H, D, equalBinDim;
            equalBinDim = 100;
            int[] w = new int[n];
            int[] h = new int[n];
            int[] d = new int[n];

            if (instanceType != 12)
            {
                W = H = D = 100;
            }
            else
            {
                // Standard dimensions of a truckcontainer half.
                W = 13000;
                H = 3000;
                D = 1200; // The whole container has 2400mm depth.
            }

            // Create a random instance
            BPP3DLib.CreateTests.MakeTests(n, w, h, d, W, H, D, equalBinDim, ref instanceType);

            // Name of the file, which holds the input data of the instance
            string instanceName = "INST" + "_N" + n.ToString() + "_IT" + instanceType.ToString() + "NR" + actInstance.ToString() + ".txt";
            string path = outputPath + instanceName;
            string res;

            // Store the instance data.
            StreamWriter swDat = new StreamWriter(path);
            swDat.WriteLine(n.ToString() + "\t" + W.ToString() + "\t" + H.ToString() + "\t" + D.ToString());
            for (int i = 0; i < n; i++)
            {
                swDat.WriteLine(w[i].ToString() + "\t" + h[i].ToString() + "\t" + d[i].ToString());
            }
            swDat.Close();

            // Start the algorithm
            int[] x;
            int[] y;
            int[] z;
            int[] bno;

            AllInfo aRob, aGen;
            int nodeLimit = 0;
            int iterLimit = 0;
            int packingType = 1;

            #region Robot packing

            // Try to find a robot packing.
            x = new int[n];
            y = new int[n];
            z = new int[n];
            bno = new int[n];

            Console.WriteLine("Actual packing: Robot packing.");
            Console.WriteLine("Started at: " + DateTime.Now.ToString());

            // Execute the algorithm
            aRob = BPP3D.BinPack3D(n, W, H, D, w, h, d, x, y, z, bno, nodeLimit, iterLimit, timeLimit, packingType);

            Console.WriteLine("Finished at: " + DateTime.Now.ToString());

            // Name of the file, which holds the result of the instance
            instanceName = "INST" + "_N" + n.ToString() + "_IT" + instanceType.ToString() + "NR" + actInstance.ToString() +
                           "_PT" + packingType.ToString() + "_TL" + timeLimit.ToString() + "TU" + ((int)aRob.Time).ToString() + ".txt";

            // Path of the output file for robot packing.
            res = outputPath + instanceName;

            // Store the results for the robot packing.
            WriteResultsToFile(res, aRob, w, h, d, x, y, z, bno);

            #endregion

            #region General Packing

            // Try to find a general packing.
            x = new int[n];
            y = new int[n];
            z = new int[n];
            packingType = 0;

            Console.WriteLine("Actual packing: General packing.");
            Console.WriteLine("Started at: " + DateTime.Now.ToString());

            // Execute the algorithm
            aGen = BPP3D.BinPack3D(n, W, H, D, w, h, d, x, y, z, bno, nodeLimit, iterLimit, timeLimit, packingType);

            Console.WriteLine("Finished at: " + DateTime.Now.ToString());

            // Name of the file, which holds the result of the instance
            instanceName = "INST" + "_N" + n.ToString() + "_IT" + instanceType.ToString() + "NR" + actInstance.ToString() +
                           "_PT" + packingType.ToString() + "_TL" + timeLimit.ToString() + "TU" + ((int)aGen.Time).ToString() + ".txt";

            // Path of the output file for general packing.
            res = outputPath + instanceName;

            // Store the results for the robot packing.
            WriteResultsToFile(res, aGen, w, h, d, x, y, z, bno);

            #endregion

        }

        public static void RunReadTest(string instancePath = @"C:\temp\")
        {
            // Run the algorithm for an instance from a file located in the path "instancePath".
            // Read the instance from a file.
            // The file to read should look like the following template:
            // n    W   H   D
            // w_1   h_1  d_1
            // ...
            // w_n   h_n  d_n
            // The entries n, W, H, D, w_i, h_i, d_i must be seperated by a tabstop "\t" and must
            // be positive integers. 
            // The box dimensions must satisfy 1 <= w_i <= W, 1<= h_i <= H, 1 <= d_i <= D.

            #region Read Test

            int n, W, H, D, i;          
            string[] lineValues;
            string actLine;
            char[] separator = new char[] { '\t' };
            StreamReader sr = new StreamReader(instancePath);
            actLine = sr.ReadLine();

            lineValues = actLine.Split(separator);
            n = Int32.Parse(lineValues[0]);
            W = Int32.Parse(lineValues[1]);
            H = Int32.Parse(lineValues[2]);
            D = Int32.Parse(lineValues[3]);

            int[] w = new int[n];
            int[] h = new int[n];
            int[] d = new int[n];
            i = 0;
            while (sr.Peek() != -1)
            {
                actLine = sr.ReadLine();
                lineValues = actLine.Split(separator);
                w[i] = Int32.Parse(lineValues[0]);
                h[i] = Int32.Parse(lineValues[1]);
                d[i] = Int32.Parse(lineValues[2]);
                i++;
            }

            sr.Close();

            #endregion

            #region Run algorithm

            int[] x = new int[n];
            int[] y = new int[n];
            int[] z = new int[n];
            int[] bno = new int[n];

            AllInfo a;
            int packingtype = 1;
            int nodelimit = 0;
            int iterlimit = 0;
            int timelimit = 0;

            a = BPP3D.BinPack3D(n, W, H, D, w, h, d, x, y, z, bno, nodelimit, iterlimit, timelimit, packingtype);

            #endregion

            ShowResultsOnConsole(a, w, h, d, x, y, z, bno);

        }

        public static void ShowResultsOnConsole(AllInfo a, int[] w, int[] h, int[] d, int[] x, int[] y, int[] z, int[] bno)
        {
            //Show the results of the test on the console window.

            Console.WriteLine("######################################################################################");
            Console.WriteLine("n" + "\t" + "W" + "\t" + "H" + "\t" + "D");
            Console.WriteLine(a.N.ToString() + "\t" + a.W.ToString() + "\t" + a.H.ToString() + "\t" + a.D.ToString());

            Console.WriteLine("w" + "\t" + "h" + "\t" + "d" + "\t" + "x" + "\t" + "y" + "\t" + "z" + "\t" + "bno");
            for (int i = 0; i < a.N; i++)
            {
                Console.WriteLine(w[i].ToString() + "\t" + h[i].ToString() + "\t" + d[i].ToString() + "\t" +
                                x[i].ToString() + "\t" + y[i].ToString() + "\t" + z[i].ToString() + "\t" +
                                bno[i].ToString());
            }
            Console.WriteLine("######################################################################################");

            Console.WriteLine("Lower bound:" + "\t" + (a.Stopped ? a.Lb : a.Z).ToString());
            Console.WriteLine("Upper bound:" + "\t" + a.Z.ToString());
            Console.WriteLine("######################################################################################");
            Console.WriteLine("Nodes:" + "\t" + a.Nodes.ToString());
            Console.WriteLine("Subnodes:" + "\t" + a.SubNodes.ToString());
            Console.WriteLine("Iterations:" + "\t" + a.Iterat.ToString());
            Console.WriteLine("Subiterations:" + "\t" + a.SubIterat.ToString());
            Console.WriteLine("######################################################################################");
            Console.WriteLine("Calls to FitsTwo:" + "\t" + a.CallsFitsTwo.ToString());
            Console.WriteLine("Calls to FitsThree:" + "\t" + a.CallsFitsThree.ToString());
            Console.WriteLine("Calls to FitsMore:" + "\t" + a.CallsFitsMore.ToString());
            Console.WriteLine("######################################################################################");
            Console.WriteLine("Overall time:" + "\t" + a.Time.ToString());
            Console.WriteLine("Heuristic time:" + "\t" + a.LayHeurTime.ToString());
            if (a.PackType == 1)
            {
                Console.WriteLine("Robot pack time:" + "\t" + a.RobotTime.ToString());
            }
            else
            {
                Console.WriteLine("General pack time:" + "\t" + a.GeneralTime.ToString());
            }
            Console.WriteLine("######################################################################################");
        }

        public static void WriteResultsToFile(string path, AllInfo a, int[] w, int[] h, int[] d, int[] x, int[] y, int[] z, int[] bno)
        {
            // Write the results into a text file originated at "path".
            // Ensure to have sufficient rights for writing files into the directory given by path.

            StreamWriter swRes = new StreamWriter(path);
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("n" + "\t" + "W" + "\t" + "H" + "\t" + "D");
            swRes.WriteLine(a.N.ToString() + "\t" + a.W.ToString() + "\t" + a.H.ToString() + "\t" + a.D.ToString());

            swRes.WriteLine("w" + "\t" + "h" + "\t" + "d" + "\t" + "x" + "\t" + "y" + "\t" + "z" + "\t" + "bno");
            for (int i = 0; i < a.N; i++)
            {
                swRes.WriteLine(w[i].ToString() + "\t" + h[i].ToString() + "\t" + d[i].ToString() + "\t" +
                                x[i].ToString() + "\t" + y[i].ToString() + "\t" + z[i].ToString() + "\t" +
                                bno[i].ToString());
            }
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("Node limit:" + "\t" + a.NodeLimit.ToString());
            swRes.WriteLine("Iteration limit:" + "\t" + a.IterLimit.ToString());
            swRes.WriteLine("Time limit:" + "\t" + a.TimeLimit.ToString());

            swRes.WriteLine("######################################################################################");
            if (a.Lb == a.Z)
            {
                swRes.WriteLine("Solution: Exact");
            }
            else
            {
                swRes.WriteLine("Solution: Heuristic");
            }
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("Lower bound:" + "\t" + (a.Stopped ? a.Lb : a.Z).ToString());
            swRes.WriteLine("Upper bound:" + "\t" + a.Z.ToString());
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("Nodes:" + "\t" + a.Nodes.ToString());
            swRes.WriteLine("Subnodes:" + "\t" + a.SubNodes.ToString());
            swRes.WriteLine("Iterations:" + "\t" + a.Iterat.ToString());
            swRes.WriteLine("Subiterations:" + "\t" + a.SubIterat.ToString());
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("Calls to FitsTwo:" + "\t" + a.CallsFitsTwo.ToString());
            swRes.WriteLine("Calls to FitsThree:" + "\t" + a.CallsFitsThree.ToString());
            swRes.WriteLine("Calls to FitsMore:" + "\t" + a.CallsFitsMore.ToString());
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("Overall time:" + "\t" + a.Time.ToString());
            swRes.WriteLine("Heuristic time:" + "\t" + a.LayHeurTime.ToString());
            if (a.PackType == 1)
            {
                swRes.WriteLine("Robot pack time:" + "\t" + a.RobotTime.ToString());
            }
            else
            {
                swRes.WriteLine("General pack time:" + "\t" + a.GeneralTime.ToString());
            }
            swRes.WriteLine("######################################################################################");
            swRes.WriteLine("######################################################################################");
            swRes.Close();
        }

    }
}
