#include <Eigen/Dense>

namespace hebi {
namespace arm {

// A class that specifies a goal position of one or more waypoint and/or auxilary
// states.
// Constructors are provided for various cases; in the case that velocities or
// accelerations are omitted, the default behavior is to leave these unconstrained
// except for a final "0" state.  For aux states, this is left as unchanged.
// For times, this is left to the Arm object to fill in a heuristic.
class Goal {

public:
  //////////////////////////////////////////////////////////////////////////////
  // Single waypoint constructors
  //////////////////////////////////////////////////////////////////////////////

  // Single waypoints, default vel/accel, no time
  Goal(const Eigen::VectorXd& positions)
    : positions_(toMatrix(positions)),
      velocities_(nanWithZeroRight(positions.size(), 1)),
      accelerations_(nanWithZeroRight(positions.size(), 1)) {}

  // Single waypoints, default vel/accel
  Goal(double time,
       const Eigen::VectorXd& positions)
    : times_(toVector(time)),
      positions_(toMatrix(positions)),
      velocities_(nanWithZeroRight(positions.size(), 1)),
      accelerations_(nanWithZeroRight(positions.size(), 1)) {}

  // Single waypoints, no time
  Goal(const Eigen::VectorXd& positions,
       const Eigen::VectorXd& velocities,
       const Eigen::VectorXd& accelerations)
    : positions_(toMatrix(positions)),
      velocities_(toMatrix(velocities)),
      accelerations_(toMatrix(accelerations)) {}

  // Single waypoints
  Goal(double time,
       const Eigen::VectorXd& positions,
       const Eigen::VectorXd& velocities,
       const Eigen::VectorXd& accelerations)
    : times_(toVector(time)),
      positions_(toMatrix(positions)),
      velocities_(toMatrix(velocities)),
      accelerations_(toMatrix(accelerations)) {}

  // Single waypoints + aux state, no time
  Goal(const Eigen::VectorXd& positions,
       const Eigen::VectorXd& velocities,
       const Eigen::VectorXd& accelerations,
       const Eigen::VectorXd& aux)
    : positions_(toMatrix(positions)),
      velocities_(toMatrix(velocities)),
      accelerations_(toMatrix(accelerations)),
      aux_(aux) {}

  // Single waypoints + aux state
  Goal(double time,
       const Eigen::VectorXd& positions,
       const Eigen::VectorXd& velocities,
       const Eigen::VectorXd& accelerations,
       const Eigen::VectorXd& aux)
    : times_(toVector(time)),
      positions_(toMatrix(positions)),
      velocities_(toMatrix(velocities)),
      accelerations_(toMatrix(accelerations)),
      aux_(aux) {}

  //////////////////////////////////////////////////////////////////////////////
  // Multiple waypoint constructors
  //////////////////////////////////////////////////////////////////////////////

  // Multiple waypoints, default vel/accel, no time
  Goal(const Eigen::MatrixXd& positions)
    : positions_(positions),
      velocities_(nanWithZeroRight(positions.rows(), positions.cols())),
      accelerations_(nanWithZeroRight(positions.rows(), positions.cols())) {}

  // Multiple waypoints, default vel/accel
  Goal(const Eigen::VectorXd& times,
       const Eigen::MatrixXd& positions)
    : times_(times),
      positions_(positions),
      velocities_(nanWithZeroRight(positions.rows(), positions.cols())),
      accelerations_(nanWithZeroRight(positions.rows(), positions.cols())) {}

  // Multiple waypoints, no time
  Goal(const Eigen::MatrixXd& positions,
       const Eigen::MatrixXd& velocities,
       const Eigen::MatrixXd& accelerations)
    : positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations) {}

  // Multiple waypoints
  Goal(const Eigen::VectorXd& times,
       const Eigen::MatrixXd& positions,
       const Eigen::MatrixXd& velocities,
       const Eigen::MatrixXd& accelerations)
    : times_(times),
      positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations) {}

  // Multiple waypoints + aux state, no time
  Goal(const Eigen::MatrixXd& positions,
       const Eigen::MatrixXd& velocities,
       const Eigen::MatrixXd& accelerations,
       const Eigen::MatrixXd& aux)
    : positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations),
      aux_(aux) {}

  // Multiple waypoints + aux state
  Goal(const Eigen::VectorXd& times,
       const Eigen::MatrixXd& positions,
       const Eigen::MatrixXd& velocities,
       const Eigen::MatrixXd& accelerations,
       const Eigen::MatrixXd& aux)
    : times_(times),
      positions_(positions),
      velocities_(velocities),
      accelerations_(accelerations),
      aux_(aux) {}

  const Eigen::VectorXd& times() const { return times_; }
  const Eigen::MatrixXd& positions() const { return positions_; }
  const Eigen::MatrixXd& velocities() const { return velocities_; }
  const Eigen::MatrixXd& accelerations() const { return accelerations_; }
  const Eigen::MatrixXd& aux() const { return aux_; }

private:
  // Helper function to create unconstrained points along a motion, with nan at the right side.
  static Eigen::MatrixXd nanWithZeroRight(size_t num_joints, size_t num_waypoints)
  {
    double nan = std::numeric_limits<double>::quiet_NaN();
    Eigen::MatrixXd matrix(num_joints, num_waypoints);
    matrix.setConstant(nan);
    matrix.rightCols<1>().setZero();
    return matrix;
  }

  static Eigen::VectorXd toVector(double scalar)
  {
    Eigen::VectorXd vector(1);
    vector[0] = scalar;
    return vector;
  }

  static Eigen::MatrixXd toMatrix(const Eigen::VectorXd& vector)
  {
    Eigen::MatrixXd matrix(vector.size(), 1);
    matrix.col(0) = vector;
    return matrix;
  }

  const Eigen::VectorXd times_{0};
  const Eigen::MatrixXd positions_{0, 0};
  const Eigen::MatrixXd velocities_{0, 0};
  const Eigen::MatrixXd accelerations_{0, 0};
  const Eigen::MatrixXd aux_{0, 0};
};

} // namespace arm
} // namespace hebi