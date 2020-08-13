#include "MPCSolver.hpp"
#include "utils.cpp"
#include "qpOASES/qpOASES.hpp"
#include  <stdlib.h>
#include <iostream>
#include <fstream>
#include  <Eigen/Cholesky>

using namespace mpcSolver;

MPCSolver::MPCSolver(double mpcTimeStep, double controlTimeStep, double predictionTime, Eigen::Vector3d initialComPosition,
                     double comTargetHeight, double singleSupportDuration, double doubleSupportDuration, double thetaMax,
					 double footConstraintSquareWidth, double deltaXMax, double deltaYIn, double deltaYOut, double measuredComWeight, double measuredZmpWeight ){

	// Set up parameters
	this->mpcTimeStep = mpcTimeStep;
	this->controlTimeStep = controlTimeStep;

	this->footConstraintSquareWidth = footConstraintSquareWidth;
	this->deltaXMax = deltaXMax;
	this->deltaYIn = deltaYIn;
	this->deltaYOut = deltaYOut;
	this->thetaMax = thetaMax;


    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(-15,15); // distribution in range [1, 6]


    this->comTargetHeight = comTargetHeight;
    //this->omega=sqrt(9.81/(comTargetHeight+1.1*comTargetHeight*cos(2*3.14*controlTimeStep/100))); // to only fall with that +1.4*comTargetHeight
    this->omega=sqrt(9.81/(comTargetHeight+1.5*comTargetHeight));


    this->singleSupportDuration=singleSupportDuration;
    this->doubleSupportDuration=doubleSupportDuration;

    this->measuredComWeight = measuredComWeight;
    this->measuredZmpWeight = measuredZmpWeight;

    N = round(predictionTime/mpcTimeStep);
    S = round(singleSupportDuration/mpcTimeStep);
    D = round(doubleSupportDuration/mpcTimeStep);
    M = ceil(N/(S+D));
    mpcIter = 0;
    controlIter = 0;

    // Matrices for cost function
    costFunctionH = Eigen::MatrixXd::Zero(2*(N+M),2*(N+M));
    costFunctionF = Eigen::VectorXd::Zero(2*(N+M));

    // Matrices for stability constraint
    Aeq = Eigen::MatrixXd::Zero(2,(N*2)+(M*2));
    beq = Eigen::VectorXd::Zero(2);

    // Matrices for ZMP constraint
    AZmp = Eigen::MatrixXd::Zero(2*N,2*(N+M));
    bZmpMax = Eigen::VectorXd::Zero(2*N);
    bZmpMin = Eigen::VectorXd::Zero(2*N);

    // Matrices for feasibility constraint
    AFootsteps = Eigen::MatrixXd::Zero(2*M,2*(M+N));
    bFootstepsMax = Eigen::VectorXd::Zero(2*M);
    bFootstepsMin = Eigen::VectorXd::Zero(2*M);

    // Matrices for swing foot constraint
    ASwingFoot = Eigen::MatrixXd::Zero(4,(N*2)+(M*2));
    bSwingFoot = Eigen::VectorXd::Zero(4);

    // Matrices for all constraints stacked
    AConstraint = Eigen::MatrixXd::Zero(2*(N+M)+2,2*(N+M));
    bConstraintMin = Eigen::VectorXd::Zero(2*(N+M)+2);
    bConstraintMax = Eigen::VectorXd::Zero(2*(N+M)+2);

    // Matrices for ZMP prediction
    p =  Eigen::VectorXd::Ones(N);
    P =  Eigen::MatrixXd::Ones(N,N)*mpcTimeStep;

    for(int i=0; i<N;++i){
    	for(int j=0;j<N;++j){
            if (j>i) P(i,j)=0;
        }
    }

    // Matrices for CoM velocity prediction
    Vu = Eigen::MatrixXd::Zero(N,N);
    Vs = Eigen::MatrixXd::Zero(N,3);

    double ch = cosh(omega*mpcTimeStep);
    double sh = sinh(omega*mpcTimeStep);

    Eigen::MatrixXd A_upd = Eigen::MatrixXd::Zero(3,3);
    Eigen::VectorXd B_upd = Eigen::VectorXd::Zero(3);
    A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
    B_upd<<mpcTimeStep-sh/omega,1-ch,mpcTimeStep;

    Eigen::RowVectorXd Vu_newline(N);
    Eigen::RowVectorXd Vs_newline(3);
    Eigen::RowVectorXd A_midLine(3);

    A_midLine<<omega*sh,ch,-omega*sh;

    for(int i=1;i<=N;++i) {
        Vu_newline.setZero();
        Vs_newline.setZero();
        Vu_newline(i-1) = 1-ch;
        if(i>1) {
            for(int j=0;j<=i-2;++j) {
                Vu_newline.segment(j,1) = A_midLine*(matrixPower(A_upd,(i-j-2)))*B_upd;
            }
        }
        Vs_newline = A_midLine*(matrixPower(A_upd,(i-1)));
        Vu.row(i-1) = Vu_newline;
        Vs.row(i-1) = Vs_newline;
    }

    // Vector that contains predicted rotations
    predictedOrientations = Eigen::VectorXd::Zero(M+1);

    // Initialize CoM, ZMP and predicted footstep




    comPos = initialComPosition;
    comPos(2) = comTargetHeight;
    comVel = Eigen::Vector3d::Zero(3);
    zmpPos = Eigen::Vector3d(comPos(0),comPos(1),0.0);
   // std::cout<<"ZMP"<<zmpPos<<std::endl;
    predictedFootstep = Eigen::Vector4d::Zero();

    // Initialize footstep counter
    footstepCounter=0;

    // Stuff for plotting
    predictedZmp = Eigen::MatrixXd::Zero(3,N);

	zmpDot = Eigen::MatrixXd::Zero(3,1);
}

void MPCSolver::solve(Eigen::Vector3d measuredComPos, Eigen::Vector3d measuredComVel, Eigen::Vector3d measuredComAcc,
					  Eigen::Affine3d swingFootTransform, bool supportFoot, double simulationTime, double vRefX, double vRefY, double omegaRef){

    // Save iteration parameters
    //this->vRefX = vRefX;
    //this->vRefY = vRefY;
    //this->omegaRef = omegaRef;
    this->supportFoot = supportFoot;
    this->simulationTime = simulationTime;

    // If new footstep is starting, change reference frame
    if(supportFootHasChanged()) changeReferenceFrame(swingFootTransform);

    // Adjust the state based on measures
    if(footstepCounter > 1) measuredComWeight = 0.1;
    else measuredComWeight = 0;

    comPos(0) = (1-measuredComWeight)*comPos(0) + measuredComWeight*measuredComPos(0);
    comPos(1) = (1-measuredComWeight)*comPos(1) + measuredComWeight*measuredComPos(1);

    comVel(0) = (1-measuredComWeight)*comVel(0) + measuredComWeight*measuredComVel(0);
    comVel(1) = (1-measuredComWeight)*comVel(1) + measuredComWeight*measuredComVel(1);

    Eigen::Vector3d measuredZmpPos = measuredComPos - measuredComAcc/(omega*omega);
    zmpPos(0) = (1-measuredComWeight)*zmpPos(0) + measuredComWeight*measuredZmpPos(0);
    zmpPos(1) = (1-measuredComWeight)*zmpPos(1) + measuredComWeight*measuredZmpPos(1);

    // Compute footsteps orientations
    computeOrientations();

    // Compute the cost function
    genCostFunction();

    // Compute the matrices for the constraints
    genStabilityConstraint();
    genBalanceConstraint();
    genFeasibilityConstraint();
    genSwingFootConstraint(swingFootTransform);

    // Stack the matrices for the constraints
    // If both feet are on the floor also add the swing foot constraint

    if (footstepCounter == 0 || mpcIter >= 0) {
    	int nConstraints = Aeq.rows() + AFootsteps.rows() + AZmp.rows() + ASwingFoot.rows();
    	AConstraint.resize(nConstraints, 2*(N+M));
    	bConstraintMin.resize(nConstraints);
    	bConstraintMax.resize(nConstraints);

    	AConstraint 	 << Aeq, AFootsteps, AZmp, ASwingFoot;
    	bConstraintMin  << beq, bFootstepsMin, bZmpMin, bSwingFoot;
    	bConstraintMax << beq, bFootstepsMax, bZmpMax, bSwingFoot;
    } else {
    	int nConstraints = Aeq.rows() + AFootsteps.rows() + AZmp.rows();
    	AConstraint.resize(nConstraints, 2*(N+M));
    	bConstraintMin.resize(nConstraints);
    	bConstraintMax.resize(nConstraints);

    	AConstraint 	 << Aeq, AFootsteps, AZmp;
    	bConstraintMin  << beq, bFootstepsMin, bZmpMin;
    	bConstraintMax << beq, bFootstepsMax, bZmpMax;
    }

    // Solve QP
    Eigen::VectorXd decisionVariables = solveQP();
    
    // Split the QP solution in ZMP dot and footsteps
    Eigen::VectorXd zDotOptimalX(N);
    Eigen::VectorXd zDotOptimalY(N);
    Eigen::VectorXd footstepsOptimalX(M);
    Eigen::VectorXd footstepsOptimalY(M);

    zDotOptimalX = (decisionVariables.head(N));
    zDotOptimalY = (decisionVariables.segment(N+M,N));
    footstepsOptimalX = decisionVariables.segment(N,M);
    footstepsOptimalY = decisionVariables.segment(2*N+M,M);

    // Save some stuff for plotting
    predictedZmp.row(0) = (P*zDotOptimalX + p*zmpPos(0)).transpose();
    predictedZmp.row(1) = (P*zDotOptimalY + p*zmpPos(1)).transpose();
    predictedZmp.row(2) = Eigen::RowVectorXd::Zero(N);

    // Update the state based on the result of the QP
    Eigen::Vector3d nextStateX = updateState(zDotOptimalX(0),0,controlTimeStep);
    Eigen::Vector3d nextStateY = updateState(zDotOptimalY(0),1,controlTimeStep);

	zmpDot << zDotOptimalX(0), zDotOptimalY(0), 0.0;

    comPos << nextStateX(0),nextStateY(0),comTargetHeight;
    comVel << nextStateX(1),nextStateY(1),0.0;
    zmpPos << nextStateX(2),nextStateY(2),0.0;
    predictedFootstep << footstepsOptimalX(0),footstepsOptimalY(0),0.0,predictedOrientations(1);

    logToFile();

    ++controlIter;
    mpcIter = floor(controlIter*controlTimeStep/mpcTimeStep);
    if(mpcIter>=S+D){
    	controlIter = 0;
    	mpcIter = 0;
        footstepCounter++;
    }

    // std::cout << "Iteration " << controlIter << " Footstep " << footstepCounter << std::endl;
}

void MPCSolver::logToFile() {
}

void MPCSolver::changeReferenceFrame(Eigen::Affine3d swingFootTransform) {

	Eigen::Vector3d swingFootPos = swingFootTransform.translation();

	// apply rotation before translation because this transform
	// is expressed in the new support foot frame
	/*comPos = swingFootTransform.rotation()*comPos;
	comVel = swingFootTransform.rotation()*comVel;
	zmpPos = swingFootTransform.rotation()*zmpPos;*/

	comPos(0)-= -swingFootPos(0);
	comPos(1)-= -swingFootPos(1);
	zmpPos(0)-= -swingFootPos(0);
	zmpPos(1)-= -swingFootPos(1);
}

void MPCSolver::genStabilityConstraint() {
	Eigen::VectorXd b(N);

	for(int i=0;i<N;++i){
		b(i) = pow(exp(-omega*mpcTimeStep),i);
	}

	Aeq.block(0,0,1,N)   = ((1/omega)*((1-exp(-omega*mpcTimeStep))/(1+pow(exp(-omega*mpcTimeStep),N))))*b.transpose();
	Aeq.block(1,N+M,1,N) = ((1/omega)*((1-exp(-omega*mpcTimeStep))/(1+pow(exp(-omega*mpcTimeStep),N))))*b.transpose();

	beq<<comPos(0) + comVel(0)/omega - zmpPos(0), comPos(1) + comVel(1)/omega - zmpPos(1);
}

void MPCSolver::genSwingFootConstraint(Eigen::Affine3d swingFootTransform) {

	Eigen::Vector3d swingFootPos = swingFootTransform.translation();

	ASwingFoot(0,N) = 1;
	ASwingFoot(1,2*N+M) = 1;
	ASwingFoot(2,N+1) = 1;
	ASwingFoot(3,2*N+M+1) = 1;

	double step = 0.05;
	if (footstepCounter == 0) step = 0;

	int sign = 1;
	if (footstepCounter%2 == 0) sign = -1;

	bSwingFoot(0) = step;//predictedFootstep(0);//swingFootPos(0);
	bSwingFoot(1) = sign*0.1;//predictedFootstep(1);//swingFootPos(1);
	bSwingFoot(2) = 2*step;
	bSwingFoot(3) = 0;
}

void MPCSolver::genCostFunction() {
	qVx = 0;
	qVy = 0;

	Eigen::MatrixXd Cc = Eigen::MatrixXd::Zero(N,M);
	Eigen::VectorXd Ccf = Eigen::VectorXd::Ones(S+D);

	for(int i=0; i<M-1; ++i){
		Cc.block(S+D-mpcIter+(i*(S+D)),i,S+D,1) = Ccf;
	}

	Cc.block(S+D-mpcIter+((M-1)*(S+D)),M-1,mpcIter,1) = Ccf.block(0,0,mpcIter,1);

	costFunctionH.block(0,0,N,N) = qZd*Eigen::MatrixXd::Identity(N,N) + qVx*Vu.transpose()*Vu + qZ*P.transpose()*P;
	costFunctionH.block(N+M,N+M,N,N) = qZd*Eigen::MatrixXd::Identity(N,N) + qVy*Vu.transpose()*Vu + qZ*P.transpose()*P;

	costFunctionH.block(0,N,N,M) = -qZ*P.transpose()*Cc;
	costFunctionH.block(N,0,M,N) = -qZ*Cc.transpose()*P;
	costFunctionH.block(N,N,M,M) = qZ*Cc.transpose()*Cc;

	costFunctionH.block(N+M,2*N+M,N,M) = -qZ*P.transpose()*Cc;
	costFunctionH.block(2*N+M,N+M,M,N) = -qZ*Cc.transpose()*P;
	costFunctionH.block(2*N+M,2*N+M,M,M) = qZ*Cc.transpose()*Cc;

	Eigen::VectorXd vArcX = Eigen::VectorXd::Zero(N);
	Eigen::VectorXd vArcY = Eigen::VectorXd::Zero(N);

	Eigen::VectorXd costFunctionF1 = Eigen::VectorXd::Zero(N+M);
	Eigen::VectorXd costFunctionF2 = Eigen::VectorXd::Zero(N+M);

	for(int i=0;i<N;++i){
		vArcX(i) = vRefX*cos(i*omegaRef*mpcTimeStep) - vRefY*sin(i*omegaRef*mpcTimeStep);
		vArcY(i) = vRefX*sin(i*omegaRef*mpcTimeStep) + vRefY*cos(i*omegaRef*mpcTimeStep);
	}

    Eigen::Vector3d stateX = Eigen::Vector3d(comPos(0),comVel(0),zmpPos(0));
    Eigen::Vector3d stateY = Eigen::Vector3d(comPos(1),comVel(1),zmpPos(1));

    costFunctionF1.block(0,0,N,1) = (qVx*Vu.transpose())*((Vs*stateX)-vArcX) + qZ*P.transpose()*p*zmpPos(0);
    costFunctionF2.block(0,0,N,1) = (qVy*Vu.transpose())*((Vs*stateY)-vArcY) + qZ*P.transpose()*p*zmpPos(1);

    costFunctionF1.block(N,0,M,1) = -qZ*Cc.transpose()*p*zmpPos(0);
    costFunctionF2.block(N,0,M,1) = -qZ*Cc.transpose()*p*zmpPos(1);

    costFunctionF<<costFunctionF1,costFunctionF2;
}

void MPCSolver::computeOrientations() {

	// Difference matrix (theta_j - theta_{j-1})
	Eigen::MatrixXd differenceMatrix = Eigen::MatrixXd::Identity(M,M);
	for (int i=0; i<M-1; ++i) {
		differenceMatrix(i+1,i) = -1;
	}

	// Rotation cost function
	Eigen::MatrixXd footstepsH = (differenceMatrix.transpose()*differenceMatrix);
	Eigen::VectorXd footstepsF = -omegaRef*(singleSupportDuration+doubleSupportDuration)*differenceMatrix.transpose()*Eigen::VectorXd::Ones(M);

	// Constraint on maximum variation of orientation
	Eigen::MatrixXd AOrientations = differenceMatrix;
	Eigen::VectorXd bOrientationsMin = -thetaMax*Eigen::VectorXd::Ones(M);
	Eigen::VectorXd bOrientationsMax =  thetaMax*Eigen::VectorXd::Ones(M);

	// Constraint on first step (no change in orientation)
	Eigen::MatrixXd AFirstOrientation = Eigen::MatrixXd::Zero(1,M);
	AFirstOrientation(0,0) = 1;
	Eigen::VectorXd bFirstOrientation = Eigen::VectorXd::Zero(M);

	// Some QP Options
	qpOASES::Options optionsOrientations;
	optionsOrientations.setToMPC();
	optionsOrientations.printLevel=qpOASES::PL_NONE;
	qpOASES::int_t nWSR = 300;

	qpOASES::QProblem qpRotations(M,M);
	qpRotations.setOptions(optionsOrientations);

	qpOASES::real_t thetaOpt[M];

	qpOASES::real_t footstepsHqpOASES[M*M];
	qpOASES::real_t footstepsFqpOASES[M];

	for(int i=0;i<M;++i){
		for(int j=0;j<M;++j){
			footstepsHqpOASES[i*M+j] = footstepsH(i,j);
		}
		footstepsFqpOASES[i] = footstepsF(i);
	}

	qpOASES::real_t AOrientationsQP[M*M];
	qpOASES::real_t bOrientationsMinQP[M];
	qpOASES::real_t bOrientationsMaxQP[M];

	for(int i=0;i<M;++i){
		for(int j=0;j<M;++j){
			AOrientationsQP[i*M+j] = AOrientations(i,j);
		}
		bOrientationsMinQP[i] = bOrientationsMin(i);
		bOrientationsMaxQP[i] = bOrientationsMax(i);
	}

	qpRotations.init(footstepsHqpOASES,footstepsFqpOASES,
			AOrientationsQP,0,0,bOrientationsMinQP,bOrientationsMaxQP,nWSR,NULL,NULL,NULL,NULL,NULL,NULL);

	qpRotations.getPrimalSolution(thetaOpt);

	predictedOrientations(0)=0;

	if(footstepCounter==0){
		for(int i=1;i<M;++i){
			predictedOrientations(1) = 0;
			predictedOrientations(i+1) = thetaOpt[i-1];
		}
	}
	else{
		for(int i=1;i<M+1;++i){
			predictedOrientations(i) = thetaOpt[i-1];
		}
	}
}

void MPCSolver::genBalanceConstraint(){
	AZmp.setZero();

	Eigen::MatrixXd Icf = Eigen::MatrixXd::Zero(S+D,S+D);
	Eigen::MatrixXd Ic = Eigen::MatrixXd::Zero(N,N);
	Eigen::MatrixXd Cc = Eigen::MatrixXd::Zero(N,M);
	Eigen::VectorXd Ccf = Eigen::VectorXd::Ones(S+D);

	for(int i=0;i<S;++i){
		Icf(i,i)=1;
	}

	if((int)simulationTime/mpcTimeStep<S+D){
		Ic.block(0,0,S+D-mpcIter,S+D-mpcIter) = Eigen::MatrixXd::Zero(S+D-mpcIter,S+D-mpcIter);
	}
	else{
		Ic.block(0,0,S+D-mpcIter,S+D-mpcIter) = Icf.block(mpcIter,mpcIter,S+D-mpcIter,S+D-mpcIter);
	}

	for(int i=0; i<M-1; ++i){
		Ic.block(S+D-mpcIter+(i*(S+D)),S+D-mpcIter+(i*(S+D)),S+D,S+D) = Icf;
	}

	Ic.block(S+D-mpcIter+((M-1)*(S+D)),S+D-mpcIter+((M-1)*(S+D)),mpcIter,mpcIter) = Icf.block(0,0,mpcIter,mpcIter);

	for(int i=0; i<M-1; ++i){
		Cc.block(S+D-mpcIter+(i*(S+D)),i,S+D,1) = Ccf;
	}

	Cc.block(S+D-mpcIter+((M-1)*(S+D)),M-1,mpcIter,1) = Ccf.block(0,0,mpcIter,1);

	Eigen::MatrixXd rCosZmp = Eigen::MatrixXd::Zero(N,N);
	Eigen::MatrixXd rSinZmp = Eigen::MatrixXd::Zero(N,N);

	rCosZmp.block(0,0,S+D-mpcIter,S+D-mpcIter) = Eigen::MatrixXd::Identity(S+D-mpcIter,S+D-mpcIter);
	rSinZmp.block(0,0,S+D-mpcIter,S+D-mpcIter) = Eigen::MatrixXd::Zero(S+D-mpcIter,S+D-mpcIter);

	for(int i=0; i<M-1; ++i){
		rCosZmp.block(S+D-mpcIter+(i*(S+D)),S+D-mpcIter+(i*(S+D)),S+D,S+D) = Eigen::MatrixXd::Identity(S+D,S+D)*cos(predictedOrientations(i+1));
		rSinZmp.block(S+D-mpcIter+(i*(S+D)),S+D-mpcIter+(i*(S+D)),S+D,S+D) = Eigen::MatrixXd::Identity(S+D,S+D)*sin(predictedOrientations(i+1));
	}

	rCosZmp.block(S+D-mpcIter+((M-1)*(S+D)),S+D-mpcIter+((M-1)*(S+D)),mpcIter,mpcIter) = Eigen::MatrixXd::Identity(S+D,S+D).block(0,0,mpcIter,mpcIter)*cos(predictedOrientations(M));
	rSinZmp.block(S+D-mpcIter+((M-1)*(S+D)),S+D-mpcIter+((M-1)*(S+D)),mpcIter,mpcIter) = Eigen::MatrixXd::Identity(S+D,S+D).block(0,0,mpcIter,mpcIter)*sin(predictedOrientations(M));

	Eigen::MatrixXd zmpRotationMatrix(2*N,2*N);
	zmpRotationMatrix << rCosZmp,rSinZmp,
						-rSinZmp,rCosZmp;

	AZmp.block(0,0,N,N) = Ic*P;
	AZmp.block(0,N,N,M) = -Ic*Cc;
	AZmp.block(N,N+M,N,N) = Ic*P;
	AZmp.block(N,2*N+M,N,M) = -Ic*Cc;

	AZmp = zmpRotationMatrix * AZmp;

	Eigen::VectorXd bZmpLeftTerm = Eigen::VectorXd::Zero(2*N);
	Eigen::VectorXd bZmpRightTerm = Eigen::VectorXd::Zero(2*N);

	bZmpLeftTerm << Ic*p*(footConstraintSquareWidth/2-0*0.0153-0.0193), Ic*p*(footConstraintSquareWidth/2-0*0.0153-0.0193);

	bZmpRightTerm << Ic*p*zmpPos(0), Ic*p*zmpPos(1);
	bZmpRightTerm = zmpRotationMatrix * bZmpRightTerm;

	bZmpMax =   bZmpLeftTerm - bZmpRightTerm;
	bZmpMin = - bZmpLeftTerm - bZmpRightTerm;
        // std::cout <<"Before restriction"<<std::endl; 
        // std::cout <<"counter   "<<footstepCounter<<std::endl; 
        // std::cout <<"max "<<bZmpMax<<std::endl;       
        // std::cout <<" min   "  <<bZmpMin<<std::endl;
        double a, c, b;
        int r = 40;
        int is, e;
        double rest_max[r];
        double rest_min[r];
        c = 1000000000;
        b = 0;
        e = 0;

double restriction_A[] = { 0.0046, 0.0097, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153, 0.0153};

double restriction_B[] = { 0.0009, 0.0019, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031, 0.0031};
   


if ((footstepCounter % 2) == 0){ b=1;}
else {b = 0;}

b = 3;

if (b==0){
//std::cout <<"enters"<<b<<std::endl;
   for (float x = 0; x<(r/2); x++){
//std::cout <<"computes"<<b<<std::endl;
        //  std::cout <<"A"<<std::endl; 
        //  std::cout <<bZmpMax[x]<<std::endl;
  if (bZmpMin(x) < 0.001 && bZmpMin(x) > -0.001 ) {is = 0;}
else {is = 1;}



//x
          bZmpMax[x] =  bZmpMax[x] - 0*is*0.005*(x/r) - is*restriction_A[e]*(x/r); // 0.005
          bZmpMin[x] =  bZmpMin[x] - 0*is*0.02*(x/r) - is*restriction_A[e]*(x/r);
//y
          bZmpMax[x+(r/2)] = bZmpMax[x+(r/2)] - 0*is*0.03*(x/r) - is*restriction_B[e]*(x/r);
          bZmpMin[x+(r/2)] = bZmpMin[x+(r/2)] - 0*is*0.008*(x/r) - is*restriction_A[e]*(x/r);// 0.007

e++;

          // std::cout <<"B"<<std::endl; 
          // std::cout <<bZmpMax[x] - is*0.01*(x*2/r)<<std::endl;
 double w = is*0.01*(x*2/r);
          // std::cout <<w<<std::endl;

       }
}

e = 0;
if (b==1) {
// std::cout <<"enters"<<b<<std::endl;
   for (float x = 0; x<(r/2); x++){
// std::cout <<"computes"<<b<<std::endl;
//          std::cout <<"A"<<std::endl; 
if (bZmpMin(x) < 0.001 && bZmpMin(x) > -0.001 ) {is = 0;}
else {is = 1;}
        //  std::cout <<bZmpMax(x)<<std::endl;
          bZmpMax[x] =  bZmpMax[x] - 0*is*0.005*(x/r) - is*restriction_A[e]*(x/r);
          bZmpMin[x] =  bZmpMin[x] - 0*is*0.02*(x/r) - is*restriction_A[e]*(x/r);
          bZmpMax[x+(r/2)] = bZmpMax[x+(r/2)] - 0*is*0.008*(x/r) - is*restriction_A[e]*(x/r); // 0.005
          bZmpMin[x+(r/2)] = bZmpMin[x+(r/2)] - 0*is*0.01*(x/r) - is*restriction_B[e]*(x/r);   // 0.012
e++;
        //  std::cout <<"B"<<std::endl; 
        //  std::cout <<bZmpMax[x] - is*0.01*(x*2/r)<<std::endl;
 double w = is*0.01*(x*2/r);
        //  std::cout <<w<<std::endl;
       }

}
       // std::cout <<"After restriction"<<std::endl; 
       // std::cout <<"max "<<bZmpMax<<std::endl;       
       // std::cout <<" min   "  <<bZmpMin<<std::endl;
}

void MPCSolver::genFeasibilityConstraint(){
	AFootsteps.setZero();

	// Difference matrix (x_j - x_{j-1})
	Eigen::MatrixXd differenceMatrix = Eigen::MatrixXd::Identity(M,M);
	for (int i=0; i<M-1; ++i){
		differenceMatrix(i+1,i) = -1;
	}

	Eigen::VectorXd pFr = Eigen::VectorXd::Zero(M);
	Eigen::VectorXd pFl = Eigen::VectorXd::Zero(M);
	Eigen::VectorXd pF = Eigen::VectorXd::Ones(M);
	Eigen::MatrixXd footstepsRotationMatrix(2*M,2*M);
	Eigen::MatrixXd rCosFootsteps = Eigen::MatrixXd::Identity(M,M);
	Eigen::MatrixXd rSinFootsteps = Eigen::MatrixXd::Zero(M,M);

	AFootsteps.block(0,N,M,M) = differenceMatrix;
	AFootsteps.block(M,2*N+M,M,M) = differenceMatrix;

	rCosFootsteps(0,0) = 1;
	rSinFootsteps(0,0) = 0;

	for(int i=1; i<M; ++i){
		rCosFootsteps(i,i) = cos(predictedOrientations(i));
		rSinFootsteps(i,i) = sin(predictedOrientations(i));
	}

	footstepsRotationMatrix <<  rCosFootsteps, rSinFootsteps,
							   -rSinFootsteps, rCosFootsteps;

	AFootsteps = footstepsRotationMatrix * AFootsteps;

	if(supportFoot==true){
		for(int i=0;i<M;++i){
			if(i%2==0) pFr(i) = 1;
			else pFl(i) = 1;
		}
	}
	else{
		for(int i=0;i<M;++i){
			if(i%2==0) pFl(i) = 1;
			else pFr(i) = 1;
		}
	}

	bFootstepsMax <<  pF*deltaXMax, -pFl*deltaYIn  + pFr*deltaYOut;
	bFootstepsMin << -pF*deltaXMax, -pFl*deltaYOut + pFr*deltaYIn;
}

Eigen::VectorXd MPCSolver::solveQP() {
	int nVariables = costFunctionH.rows();
	int nConstraints = AConstraint.rows();

	qpOASES::real_t H[nVariables*nVariables];
	qpOASES::real_t g[nVariables];

	qpOASES::real_t A[nConstraints*nVariables];
	qpOASES::real_t lb[nConstraints];
	qpOASES::real_t ub[nConstraints];

	for(int i=0;i<nVariables;++i){
		for(int j=0;j<nVariables;++j){
			H[i*nVariables+j] = costFunctionH(i,j);
		}
		g[i] = costFunctionF(i);
	}

	for(int i=0;i<nConstraints;++i){
		for(int j=0;j<nVariables;++j){
			A[i*nVariables+j] = AConstraint(i,j);
		}
		lb[i] = bConstraintMin(i);
		ub[i] = bConstraintMax(i);
	}

	qpOASES::real_t xOpt[nVariables];

	qpOASES::Options options;
	options.setToMPC();
	options.printLevel=qpOASES::PL_NONE;
	qpOASES::int_t nWSR = 300;

	qp = qpOASES::QProblem(nVariables, nConstraints);
	qp.setOptions(options);
	qp.init(H,g,A,0,0,lb,ub,nWSR,NULL,NULL,NULL,NULL,NULL,NULL);

	qp.getPrimalSolution(xOpt);

	Eigen::VectorXd decisionVariables(2*(N+M));

	for(int i=0;i<2*(N+M);++i){
		decisionVariables(i) = xOpt[i];
	}

	return decisionVariables;

}

Eigen::Vector3d MPCSolver::updateState(double zmpDot, int dim, double timeStep) {
	// Update the state along the dim-th direction (0,1,2) = (x,y,z)

    double ch = cosh(omega*timeStep);
    double sh = sinh(omega*timeStep);

	Eigen::Matrix3d A_upd = Eigen::MatrixXd::Zero(3,3);
	Eigen::Vector3d B_upd = Eigen::VectorXd::Zero(3);
	A_upd<<ch,sh/omega,1-ch,omega*sh,ch,-omega*sh,0,0,1;
	B_upd<<timeStep-sh/omega,1-ch,timeStep;

    Eigen::Vector3d currentState =
    	Eigen::Vector3d( comPos(dim),comVel(dim),zmpPos(dim));

    return A_upd*currentState + B_upd*zmpDot;
}

bool MPCSolver::supportFootHasChanged(){
    if (controlIter==0 && footstepCounter>0) return true;
    else return false;
}

Eigen::VectorXd MPCSolver::getOptimalCoMPosition(){
    return comPos;
}

Eigen::VectorXd MPCSolver::getOptimalCoMVelocity(){
    return comVel;
}

Eigen::VectorXd MPCSolver::getOptimalFootsteps(){
    return predictedFootstep;
}

Eigen::VectorXd MPCSolver::getOptimalZMPPosition(){
    return zmpPos;
}

Eigen::MatrixXd MPCSolver::getPredictedZmp(){
	return predictedZmp;
}

void MPCSolver::setComTargetHeight(double height) {
	comTargetHeight = height;
}

void MPCSolver::setReferenceVelocityX(double ref) {
	vRefX = ref;
}

void MPCSolver::setReferenceVelocityY(double ref) {
	vRefY = ref;
}

void MPCSolver::setReferenceVelocityOmega(double ref) {
	omegaRef = ref;
}

void MPCSolver::setVelGain(double value) {
	qVx = value;
	qVy = value;
}

void MPCSolver::setZmpGain(double value) {
	qZ = value;
}

double MPCSolver::getOmega() {
	return omega;
}

Eigen::MatrixXd MPCSolver::getZMPDot() {
	return zmpDot;
}

