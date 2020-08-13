#include <Eigen/Eigen>
#include "dart/dart.hpp"
#include "MPCSolver.hpp"
#include "Utility.hpp"
#include "Observer.hpp"
#include  <fstream>

class Controller
{
public:
  Controller(dart::dynamics::SkeletonPtr _robot,
			 dart::simulation::WorldPtr _world);
  virtual ~Controller();

  std::pair<Eigen::Vector3d, double> getZmpFromWrench();
  std::pair<Eigen::Vector3d, Eigen::Vector2d> getZmpFromExternalForces();
  Eigen::Vector3d getZmpFromAngularMomentum();

  void update();

  dart::dynamics::SkeletonPtr getRobot() const;
  mpcSolver::MPCSolver* getSolver();
  dart::dynamics::BodyNode* getSupportFoot();

  Eigen::MatrixXd getTorsoAndSwfJacobian();
  Eigen::MatrixXd getComAndSwfJacobian();

  Eigen::VectorXd getEndEffector() const;

  Eigen::VectorXd getOmnidirectionalSwingFootTrajectoryMPC(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, double, int);

  Eigen::VectorXd getJointVelocitiesStacked(Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd);

  Eigen::VectorXd getJointVelocitiesStacked(Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd);

  Eigen::VectorXd getJointVelocitiesPrioritized(Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd,
		  Eigen::VectorXd, Eigen::VectorXd);

  Eigen::VectorXd getJointVelocitiesQp(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

  void setInitialConfiguration();
  void setObservers(double, Eigen::Vector3d&, double);

  Eigen::VectorXd generateWalking();
  Eigen::VectorXd generateBalance();

  Eigen::Vector3d getRPY(dart::dynamics::BodyNode*, dart::dynamics::BodyNode*);

  void storeData();

  virtual void keyboard(unsigned char _key, int _x, int _y);

  void setComTargetHeight(double);
  void setReferenceVelocityX(double);
  void setReferenceVelocityY(double);
  void setReferenceVelocityOmega(double);
  void setBalancePointCom();
  void setBalancePointTorso();
  void setBeheaviorBalance();
  void setBeheaviorFreeBalance();
  void setBeheaviorWalk();
  void setVelGain(double);
  void setZmpGain(double);
  void setBalanceBasePos(Eigen::VectorXd);
  void setBalanceFootPos(Eigen::VectorXd);

  Eigen::VectorXd getBalanceBasePos();
  Eigen::VectorXd getBalanceFootPos();

  // External forces
  void setExternalForceConstant();
  void setExternalForcePeriodic();
  void setExternalForceStartFrame(int);
  void setExternalForceX(float);
  void setExternalForceY(float);
  void setExternalForceZ(float);
  void setExternalForcePeriodicPhase(float);
  void setExternalForcePeriodicFrequency(float);

  // Observers
  void addLuenbergerObserver();
  void addKalmanObserver();
  void addStephensObserver();
  void removeLuenbergerObserver();
  void removeKalmanObserver();
  void removeStephensObserver();
  void setObserverDelay(int);

  Eigen::VectorXd shift(Eigen::VectorXd v);
  Eigen::Vector3d getExternalForce();
  Eigen::Vector3d getExternalForceDerivative();

  std::string getLogPath();

private:
  dart::dynamics::SkeletonPtr mRobot;

  dart::dynamics::BodyNode* mTorso;

  dart::simulation::WorldPtr mWorld;

  int indInitial;

  int footstepCounter = 0;

  double stepHeight = 0.02;

	double g = 9.81;	     // gravity
	double Mc = 35.1954;   // [kg] mass of the robot
  double comTargetHeight = 0.33; //0.33;
  double ni = sqrt(g/comTargetHeight);
  double dt = 1.0 / 100.0;
  Eigen::Vector3d comInitialPosition;

  bool supportFoot;
  bool LEFT = false;
  bool RIGHT = true;

  double ikGain;

  bool balancePoint;
  bool TORSO = false;
  bool COM = true;

  int beheavior;
  int BALANCE = 0;
  int WALK = 1;
  int FREE_BALANCE = 2;

  double singleSupportDuration, doubleSupportDuration;

  Eigen::VectorXd swingFootStartingPosition;

  mpcSolver::MPCSolver* solver;

  dart::dynamics::BodyNode* leftFoot;
  dart::dynamics::BodyNode* rightFoot;
  dart::dynamics::BodyNode* mSupportFoot;
  dart::dynamics::BodyNode* mSwingFoot;
  dart::dynamics::BodyNode* mBase;

  Eigen::VectorXd initialConfiguration = Eigen::VectorXd::Zero(30);

  Eigen::VectorXd qDotOld = Eigen::VectorXd::Zero(24);

  // Balance
  Eigen::VectorXd balanceBasePos;
  Eigen::VectorXd balanceFootPos;

  // Observer
  CompositeObserver* observers;
  int obsDelay;

  // ZMP Filter 
  int FIROrder = 31;   // order of the filter
  Eigen::Vector3d measZmp;
  Eigen::Vector3d lastMeasZmp;

	Eigen::VectorXd ZMPPastMeasuresX;
	Eigen::VectorXd ZMPPastMeasuresY;
	Eigen::RowVectorXd FIRCoeffs;
	double ZMPCurrentX, ZMPCurrentY;  //current value of ZMP
	double ZMPFilteredX, ZMPFilteredY; //filtered value of ZMP tp feed the observer with and to log

  // External  force
  int EXTFORCE_CONST = 0;
  int EXTFORCE_PERIODIC = 1;

  Eigen::Vector3d externalForce = Eigen::Vector3d(0.0, 0.0, 0.0);
  int externalForceMode = 0;
  float externalForcePeriodicPhase = 0.0;
  float externalForcePeriodicFrequency = 1.0;
  int externalForceStartFrame = 250;
  double externalForceXOffset = 26.0;

  // Log path
  std::string logPath = "../data/";
};
