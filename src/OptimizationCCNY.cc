/**
 *  @file OptimizationCCNY.cc
 *  @author Ejup Hoxha <ejup.hoxha@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2019, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "OptimizerE.h"


int main(int argc, char **argv)
{
	OptimizerE graph;
	graph.AddVertex(1);	// Add vertex 1; Estimate is set as: [Eigen::Isometry3d::Identity()]
	graph.AddVertex(2);  // Add vertex 2
	
	//Inverse of the Covariance matrix.
	Eigen::Matrix<double, 6, 6> infoMatrix = Eigen::Matrix<double, 6, 6>::Identity();
	for (size_t i = 0; i < 6; i++)
	{
	   infoMatrix(i,i) = 100;
	}

	Eigen::Isometry3d T;   //Homogenous Transformation matrix
	T << 1, 0, 0, 0,
     	     0, 1, 0, 1,
             0, 0, 1, 0,
	     0, 0, 0, 1;       // Translated from the origin on y direction for 1 unit.

	graph.AddEdge(1, 2, infoMatrix, T)
	int iterations =10;
	graph.Optimize(iterations);

	cout << "Optimization Project!\n";
	return 0;
}
