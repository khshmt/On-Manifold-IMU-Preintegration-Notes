#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>

/*
	Modeling Robot Motion Example
	=============================
	Prior
	|
	v
	(x1)  ----Between---- > (x2)  ----Between---- > (x3)
	|                          |                         |
	pose1                      pose2                     pose3

*/

int main() {
	// empty nonlinear factor graph
	gtsam::NonlinearFactorGraph graph;

	// prior at origin x=0, y=0, theta=0
	gtsam::Pose2 priorMean(0.0, 0.0, 0.0);
	// noise model with standard deviations 0.1 for x, y, and theta
	gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
		gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1, 0.1, 0.1));
	// add prior factor on key 1
	graph.add(gtsam::PriorFactor<gtsam::Pose2>(1, priorMean, priorNoise));


	// odometry measurement: move 1.0 meter forward and rotate 90 degrees
	gtsam::Pose2 odometry(1.0, 0.0, M_PI / 2);
	// noise model with standard deviations 0.2 for x, y, and 0.1 for theta
	gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
		gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));
	// add odometry factors between consecutive poses
	graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, odometry, odometryNoise));
	graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, odometry, odometryNoise));

	gtsam::Values initialEstimate;
	initialEstimate.insert(1, gtsam::Pose2(0.5, 0.0, 0.2));
	initialEstimate.insert(2, gtsam::Pose2(2.3, 0.1, -0.2));
	initialEstimate.insert(3, gtsam::Pose2(4.1, 0.1, 0.1));

	// optimize using Levenberg-Marquardt optimization
	gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

	gtsam::Marginals marginals(graph, result);

	graph.print("Factor Graph:\n");

	initialEstimate.print("Initial Estimate:\n");

	std::cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << "\n\n";
	std::cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << "\n\n";
	std::cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << "\n\n";

	result.print("Final Result:\n");

	return 0;
}