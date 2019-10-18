#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

using namespace std;



//We should check which ones we need!!!
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>

//Eigen
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>

class OptimizerE
{
	public:		
		std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver (new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>());
		std::unique_ptr<g2o::BlockSolver_6_3> blockSolver (new g2o::BlockSolver_6_3(std::move(linearSolver)));
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
		g2o::SparseOptimizer globalOptimizer;
		void AddVertex(int _vertexID);		//Add vertex to the graph
		void SetEstimate(); 						//Set vertex estimate
		void GetEstimate(int _vertexID);				//Get vertex estimate
		//Add edge between two vertices, set 1/Covariance matrix and set relationship between those two vertices (T-Homogenous transformation matrix)
		void AddEdge(int _vertex_1_ID, int _vertex_2_ID, Eigen::Matrix<double, 6, 6> _information, Eigen::Isometry3d _T);
		void Optimize(int _iterations);					//Optimize!
		void SaveGraph(string name);					//Save current graph;
		void ClearOptimizer();						//Clear graph!
}

//Define some colors for terminal...

#define RESET "\033[0m"
#define BLACK "\033[30m" /* Black */
#define RED "\033[31m" /* Red */
#define GREEN "\033[32m" /* Green */
#define YELLOW "\033[33m" /* Yellow */
#define BLUE "\033[34m" /* Blue */
#define MAGENTA "\033[35m" /* Magenta */
#define CYAN "\033[36m" /* Cyan */
#define WHITE "\033[37m" /* White */
#define BOLDBLACK "\033[1m\033[30m" /* Bold Black */
#define BOLDRED "\033[1m\033[31m" /* Bold Red */
#define BOLDGREEN "\033[1m\033[32m" /* Bold Green */
#define BOLDYELLOW "\033[1m\033[33m" /* Bold Yellow */
#define BOLDBLUE "\033[1m\033[34m" /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m" /* Bold Magenta */
#define BOLDCYAN "\033[1m\033[36m" /* Bold Cyan */
#define BOLDWHITE "\033[1m\033[37m" /* Bold White */

