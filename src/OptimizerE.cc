/**
 *  @file OptimizerE.cc
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

OptimizerE::OptimizerE()
{
	std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver (new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());
	std::unique_ptr<g2o::BlockSolver_6_3> blockSolver (new g2o::BlockSolver_6_3(std::move(linearSolver)));
	g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
	globalOptimizer.setAlgorithm(solver);
}

void OptimizerE::AddVertex(int _vertexID)
{
	g2o::VertexSE3 *vertex = new g2o::VertexSE3();
        vertex->setId(_vertexID);
        vertex->setEstimate(Eigen::Isometry3d::Identity());
	globalOptimizer.addVertex(vertex);
}

void OptimizerE::SetEstimate()
{

}

void OptimizerE::GetEstimate(int _vertexID)
{

}

void OptimizerE::AddEdge(int _vertex_1_ID, int _vertex_2_ID, Eigen::Matrix<double, 6, 6> _information, Eigen::Isometry3d _T)
{
 	g2o::EdgeSE3 *edge = new g2o::EdgeSE3();
        edge->vertices()[0] = globalOptimizer.vertex(_vertex_1_ID);
        edge->vertices()[1] = globalOptimizer.vertex(_vertex_2_ID);
        edge->setInformation(_information); 
        edge->setMeasurement(_T);
        globalOptimizer.addEdge(edge);
	
}
void OptimizerE::Optimize(int _iterations)
{
	cout <<GREEN"\nOptimizing the graph with: "<<globalOptimizer.vertices().size()<<" nodes\n";
	cout <<YELLOW"Iterations: "<<_iterations<<"\nSaving nonoptimized graph as: 'nonoptimizedGraph.g2o'\n";
    	globalOptimizer.save("nonoptimizedGraph.g2o");
    	globalOptimizer.initializeOptimization();
	globalOptimizer.optimize(_iterations); 
	cout <<YELLOW"Optimization Done!\nSaving optimized graph as: 'optimizedGraph.g2o'\n";
	globalOptimizer.save("optimizedGraph.g2o");
}

void OptimizerE::SaveGraph(string _name)
{
	//globalOptimizer.save(_name);
	//cout <<GREEN"\nGraph saved: "<<_name<<endl;
}

void OptimizerE::ClearOptimizer()
{
     globalOptimizer.clear();
     cout <<YELLOW"\nCleared!";
}
