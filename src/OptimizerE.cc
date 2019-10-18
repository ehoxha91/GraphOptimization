
#include "OptimizerE.h"


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

void OptimizerE::SaveGraph(string name)
{
	globalOptimizer.save(name);
	cout <<GREEN"\nGraph saved: "<<name<<endl;
}

void OptimizerE::ClearOptimizer()
{
     globalOptimizer.clear();
     cout <<YELLOW"\nGraph saved: "<<name<<endl;
}
