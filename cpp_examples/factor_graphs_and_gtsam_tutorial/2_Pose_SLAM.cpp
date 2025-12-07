#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// PoseSLAM Example
// ==========================
// https://gtsam.org/tutorials/intro-images/8_Users_dellaert_git_github_doc_images_FactorGraph3.png

int main() {
    gtsam::NonlinearFactorGraph graph;
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.3, 0.3, 0.1));
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(1, gtsam::Pose2(0.0, 0.0, 0.0), priorNoise));

    // Add Odometry factors
    gtsam::noiseModel::Diagonal::shared_ptr model =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.2, 0.2, 0.1));

    // https://gtsam.org/tutorials/intro-images/9_Users_dellaert_git_github_doc_images_example1.png
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(1, 2, gtsam::Pose2(2, 0, 0), model));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(2, 3, gtsam::Pose2(2, 0, M_PI / 2), model));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(3, 4, gtsam::Pose2(2, 0, M_PI / 2), model));
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(4, 5, gtsam::Pose2(2, 0, M_PI / 2), model));
    // Add the loop closure constraint
    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(5, 2, gtsam::Pose2(2, 0, M_PI / 2), model));

    gtsam::Values initialEstimate;
	initialEstimate.insert(1, gtsam::Pose2(0.0, 0.0, 0.0));
	initialEstimate.insert(2, gtsam::Pose2(2.3, 0.1, 0.0));
	initialEstimate.insert(3, gtsam::Pose2(4.1, 0.1, M_PI / 2));
    initialEstimate.insert(4, gtsam::Pose2(4.1, 2.1, M_PI));
    initialEstimate.insert(5, gtsam::Pose2(2.2, 2.1, -M_PI));

	// optimize using Levenberg-Marquardt optimization
	gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
    result.print("Final result:\n");

    return 0;
}