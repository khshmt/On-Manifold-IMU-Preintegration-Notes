#include <iostream>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>


/*
	Robot Localization Example
	==========================

	(x1)---------------(x2)----------------(x3)
	|                   |                   |
	pose1              pose2               pose2
	|                   |                   |
	+--UnaryFactor      +--UnaryFactor      +--UnaryFactor
*/

class UnaryFactor : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
public:
	UnaryFactor(gtsam::Key j, double x, double y, const gtsam::SharedNoiseModel& model) :
		NoiseModelFactor1<gtsam::Pose2>(model, j), mx_(x), my_(y) {}

	gtsam::Vector evaluateError(const gtsam::Pose2& q,
		boost::optional<gtsam::Matrix&> H = boost::none) const
	{
		const gtsam::Rot2& R = q.rotation();
		if (H) (*H) = (gtsam::Matrix(2, 3) <<
			R.c(), -R.s(), 0.0,
			R.s(), R.c(), 0.0).finished();
		return (gtsam::Vector(2) << q.x() - mx_, q.y() - my_).finished();
	}

private:
	double mx_, my_; ///< X and Y measurements
};

int main() {
	gtsam::NonlinearFactorGraph graph;

	// add unary measurement factors, like GPS, on all three poses
	gtsam::noiseModel::Diagonal::shared_ptr unaryNoise =
		gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.1, 0.1)); // 10cm std on x,y

	graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
	graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
	graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));


	gtsam::Values initialEstimate;
	initialEstimate.insert(1, gtsam::Pose2(0.5, 0.0, 0.0));
	initialEstimate.insert(2, gtsam::Pose2(2.3, 0.1, 0.0));
	initialEstimate.insert(3, gtsam::Pose2(4.1, 0.1, 0.0));

	// optimize using Levenberg-Marquardt optimization
	gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();

	result.print("Final result:\n");
	return 0;
}