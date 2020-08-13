#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "qpOASES/qpOASES.hpp"
#include <dart/dart.hpp>

namespace mpcSolver{

    class MPCSolver{
	public:
        MPCSolver(double, double, double, Eigen::Vector3d, double, double, double, double, double, double, double, double, double, double);

        // Main method
        void solve(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Eigen::Affine3d, bool, double, double, double, double);

        // Get stuff
        Eigen::VectorXd getOptimalCoMPosition();
        Eigen::VectorXd getOptimalCoMVelocity();
        Eigen::VectorXd getOptimalZMPPosition();
        Eigen::VectorXd getOptimalFootsteps();
        Eigen::MatrixXd getPredictedZmp();
        Eigen::MatrixXd getZMPDot();
        bool supportFootHasChanged();
	double getOmega();

	// Set stuff
	void setComTargetHeight(double);
	void setReferenceVelocityX(double);
	void setReferenceVelocityY(double);
	void setReferenceVelocityOmega(double);
	void setVelGain(double);
	void setZmpGain(double);

        // Generate matrices
        void genCostFunction();
        void genStabilityConstraint();
        void genBalanceConstraint();
        void genFeasibilityConstraint();
        void genSwingFootConstraint(Eigen::Affine3d);

        // Solve
        void computeOrientations();
        Eigen::VectorXd solveQP();
        Eigen::VectorXd solveQPdart();

        // Update the state
        Eigen::Vector3d updateState(double,int,double);
        void changeReferenceFrame(Eigen::Affine3d);

        // Log
        void logToFile();

	private:

        // Constant parameters
        int N,S,D,M, footstepCounter;
        double singleSupportDuration, doubleSupportDuration, thetaMax;
        double footConstraintSquareWidth;
        double deltaXMax;
        double deltaYIn;
        double deltaYOut;
        double mpcTimeStep;
        double controlTimeStep;
        double comTargetHeight;
        double omega;
        double measuredComWeight = 0;
        double measuredZmpWeight = 0;

        // Parameters for the current iteration
        bool supportFoot;
        double simulationTime;
        double vRefX=0;
        double vRefY=0;
        double omegaRef=0;
        int mpcIter,controlIter;

        // Matrices for prediction
        Eigen::VectorXd p;
        Eigen::MatrixXd P;
        Eigen::MatrixXd Vu;
        Eigen::MatrixXd Vs;

        // Matrices for cost function
        Eigen::MatrixXd costFunctionH;
        Eigen::VectorXd costFunctionF;

        // Matrices for stability constraint
        Eigen::MatrixXd Aeq;
        Eigen::VectorXd beq;

        //Matrices for balance constraint
        Eigen::MatrixXd AZmp;
        Eigen::VectorXd bZmpMax;
        Eigen::VectorXd bZmpMin;

        // Matrices for feasibility constraints
        Eigen::MatrixXd AFootsteps;
        Eigen::VectorXd bFootstepsMax;
        Eigen::VectorXd bFootstepsMin;

        // Matrices for swing foot constraints
        Eigen::MatrixXd ASwingFoot;
        Eigen::VectorXd bSwingFoot;

        // Matrices for the stacked constraints
        Eigen::MatrixXd AConstraint;
        Eigen::VectorXd bConstraintMax;
        Eigen::VectorXd bConstraintMin;

        // Solution of the QP for determining orientations
        Eigen::VectorXd predictedOrientations;

        // Cost function weights
        double qZd = 1;
        double qVx = 0;//100;
        double qVy = 0;//100;
        double qZ = 1000;

        // State
        Eigen::Vector3d comPos;
        Eigen::Vector3d comVel;
        Eigen::Vector3d zmpPos;
        Eigen::Vector4d predictedFootstep;

	// Quadratic problem
	qpOASES::QProblem qp;

	// Some vectors to plot
	Eigen::MatrixXd predictedZmp;

        // ZMP dot for observers
        Eigen::MatrixXd zmpDot;

   };

}

#endif
