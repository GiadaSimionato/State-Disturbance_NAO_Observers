#pragma once
#include "Eigen/Dense"
#include <list>
#include <map>
#include <string>


class Observer{
    public:
        Observer(std::string name) { this->name = name; }
        virtual ~Observer() {}
        virtual void update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y) = 0;
        virtual std::map<std::string, Eigen::VectorXd> state() = 0;
        virtual std::map<std::string, Eigen::MatrixXd> uncertainty() = 0;
        std::string getName() { return name; }
    protected:
        std::string name;
};

class CompositeObserver : public Observer {
    public:
        CompositeObserver(std::string name): Observer(name) {};
        ~CompositeObserver();

        void add(Observer *obs);
        void rem(Observer *obs);
        Observer* getChild(std::string name);

        virtual void update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y);

        std::map<std::string, Eigen::MatrixXd> uncertainty();
        std::map<std::string, Eigen::VectorXd> state();
    
    private:
        std::list<Observer *> children;
};

class LeafObserver : public Observer {
    public:
        LeafObserver(std::string, int, double, int, int);
        virtual void init();
        int getM();
        int getN();
        double getDt();
        std::map<std::string, Eigen::VectorXd> state();

    protected:
        // System dimensions
        int m, n; 
        int axis;
        double dt;
        bool initialized;
        // Estimated states 
        Eigen::VectorXd xAct;
};


class LuenbergerObserver : public LeafObserver{
    public:
        /*
        Class constructor

        @param A: Dynamics matrix
        @param B: Input matrix
        @param C: Output matrix
        @param G: Gain matrix
        @param name: Observer name
        @param axis: Observer axis
        */
        LuenbergerObserver(
            double dt,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& G, std::string name, int axis);

        /*
        Initialization of the Luenberger Observer
        */
        void init();

        /*
        Initialization of the Luenberger Observer with initial guessing

        @param x0: initial estimate of state system
        */
        void init(const Eigen::VectorXd& x0);

        /*
        Update step (contains both prediction and correction)

        @param U: matrix of inputs
        @param Y: matrix of measurements
        */
        void update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y);
        // std::map<std::string, Eigen::VectorXd> state();
        std::map<std::string, Eigen::MatrixXd> uncertainty();

     private:
         // Matrices
         Eigen::MatrixXd A,B,C,G;
};

class KalmanFilter : public LeafObserver {

    public:
        /*
        Class constructor

        @param dt: timestep
        @param R: process noise covariance
        @param P: estimate error covariance
        @param sigmaJerk: covariance of CoM jerk
        @param sigmaDdfext: covariance of second derivative of external force
        */
        KalmanFilter(double dt,
                    const Eigen::MatrixXd& A,
                    const Eigen::MatrixXd& B,
                    const Eigen::MatrixXd& C,
                    const Eigen::MatrixXd& R,
                    double sigmaJerk,
                    double sigmaDdfext, std::string name, int axis);


        /*
        Initialization of the Kalman Filter
        */

        void init();


        /*
        Initialization of the Kalman Filter with initial guessing

        @param x0: initial estimate of state system
        @param P0: initial covariance estimate
        */

        void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);

        /*
        Update step (contains both prediction and correction) - for Z observer

        @param y: vector of measurements
        */
        
        void update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y);

        /*
        Update step (contains both prediction and correction) - for X, Y observers

        @param y: vector of measurements
        @param C: dynamic matrix to overwrite
        */

        void update(const Eigen::MatrixXd& C, const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y);
        

        /*
        Returns the current estimate of the system state
        */
        //std::map<std::string, Eigen::VectorXd> state();
        std::map<std::string, Eigen::MatrixXd> uncertainty();
        
    private:
        Eigen::MatrixXd A, C, Q, R, P;//,  K, NI; //Kalman Filter matrices
};

class KalmanComposite : public CompositeObserver {
    public:
        KalmanComposite(double g, double Mc): CompositeObserver("kalman") {
            this->g = g;
            this->Mc = Mc;
        };

        void update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y);
        // std::map<std::string, Eigen::MatrixXd> uncertainty();
        // std::map<std::string, Eigen::VectorXd> state();

        void addKalmanX(KalmanFilter *obs);
        void addKalmanY(KalmanFilter *obs);
        void addKalmanZ(KalmanFilter *obs);


    private:
        double g;
        double Mc;
        KalmanFilter *kalmanX;
        KalmanFilter *kalmanY;
        KalmanFilter *kalmanZ;
};

// -------------------- Stephens -----------------------

class StephensFilter : public LeafObserver {

public:

    /*
    Class constructor

    @param A: system dynamics matrix
    @param B: input matrix
    @param C: output matrix
    @param Q: process noice covariance 
    @param R: output noise covariance
    @param P: estimate error covariance
    */
    StephensFilter(
            double dt,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R, std::string name, int axis);


    /*
    Initialization of the Kalman Filter
    */
    void init();


    /*
    Initialization of the Kalman Filter with initial guessing

    @param x0: initial estimate of state system
    @param P0: initial covariance estimate
    */
    void init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0);


    /*
    Update step (contains both prediction and correction)

    @param x_ vectpr of inputs
    @param y: vector of measurements
    */
    void update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y);

    
    // /*
    // Returns the current estimate of the system state
    // */
    // std::map<std::string, Eigen::VectorXd> state();

    /*
    Returns the current covariance
    */
    std::map<std::string, Eigen::MatrixXd> uncertainty();

    private:
        Eigen::MatrixXd A, B, C, Q, R, P; // , G, NI; //Kalman Filter matrices
};

