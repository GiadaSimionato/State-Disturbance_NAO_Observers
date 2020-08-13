#include <iostream>
#include <stdexcept>
#include "Observer.hpp"
#include <map>
#include <string>


/* ------------ Composite ---------------*/

void CompositeObserver::add(Observer *obs) {
     children.push_back(obs); }

CompositeObserver::~CompositeObserver () {
    for (Observer *c : children) {
        delete c;
    }
}

void CompositeObserver::rem(Observer *obs) {
    children.remove(obs);
    delete obs;
}

void CompositeObserver::update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y) { 
    for (Observer *c : children) {
       c->update(U, Y);
    }
}

std::map<std::string, Eigen::VectorXd> CompositeObserver::state() {
    std::map<std::string, Eigen::VectorXd> mp;
    for (Observer *c : children) {
        auto map = c->state();
        mp.insert(map.begin(), map.end());
    }
    return mp;
}

std::map<std::string, Eigen::MatrixXd> CompositeObserver::uncertainty() {
    std::map<std::string, Eigen::MatrixXd> m;
    for (Observer *c : children) {
         auto map = c->uncertainty();
         m.insert(map.begin(), map.end());
    }
    return m;
}

Observer* CompositeObserver::getChild(std::string name){
    for (Observer* obs : children) {
        if (obs->getName() == name){
            return obs;
        }
    }
    return 0;
}

/* --------- Leaf ----------- */

LeafObserver::LeafObserver(std::string name, int axis, double dt, int n, int m):
Observer(name), axis(axis), dt(dt), n(n), m(m) {
        xAct.resize(n);
}

void LeafObserver::init() { initialized = true; }

int LeafObserver::getM() { return m; }

int LeafObserver::getN() { return n; }

double LeafObserver::getDt() { return dt; }


std::map<std::string, Eigen::VectorXd> LeafObserver::state(){
    std::map<std::string, Eigen::VectorXd> map;
    map[name] = xAct;
    return map;
}


/* ---------- Luenberger --------------- */

LuenbergerObserver::LuenbergerObserver(
            double dt,
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& B,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& G, std::string name, int axis):
            LeafObserver(name, axis, dt, A.rows(), C.rows()),
             A(A), C(C), B(B), G(G){
               xAct.resize(n);
           }


void LuenbergerObserver::init(const Eigen::VectorXd& x0){
    LeafObserver::init();
    xAct = x0;
}

void LuenbergerObserver::init(){
    LeafObserver::init();
    xAct.setZero();
}

void LuenbergerObserver::update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y){
    if(!initialized)
        throw std::runtime_error("Observer is not initialized!");
    Eigen::VectorXd u = U.row(this->axis);
    Eigen::Vector2d y;
    if (this->axis == 2) {
        y << Y.row(this->axis)[0], 0.0;
    } else {
        y << Y.row(this->axis)[0], Y.row(this->axis)[1];
    }
    
    Eigen::VectorXd xD = (A * xAct) + (B * u) + (G*(y - C*xAct));
    xAct = xD * dt + xAct;
}

std::map<std::string, Eigen::MatrixXd> LuenbergerObserver::uncertainty() {
    std::map<std::string, Eigen::MatrixXd> map;
    //TODO: rendere piu efficiente
    Eigen::MatrixXd mat(n, n);
    mat.setZero();
    map[name] = mat;
    return map;
}


/* ------------------ Kalman Composite ---------------- */


void KalmanComposite::update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y) {
    kalmanZ->update(U, Y);

    double zcHat = kalmanZ->state()["kalman_z"](0);   // estimate of z CoM
    double ddzcHat = kalmanZ->state()["kalman_z"](2); // estimate of acceleration z CoM
    double fzHat = kalmanZ->state()["kalman_z"](3);   // estimate of external force z axis
    double grfHat = -Mc*g - Mc*ddzcHat + fzHat;
    //double grfHat = Y.row(2)[1];

    Eigen::MatrixXd Cx(kalmanZ->getM(), kalmanZ->getN());
    Cx << 1, 0, 0, 0, 0,
          0, 0, 1, 0, 0,
          1, 0, Mc*zcHat/grfHat, -zcHat/grfHat, 0;

    kalmanX->update(Cx, U, Y);
    kalmanY->update(Cx, U, Y);
}
         
void KalmanComposite::addKalmanX(KalmanFilter *obs) {
    kalmanX = obs;
    CompositeObserver::add(kalmanX);
}

void KalmanComposite::addKalmanY(KalmanFilter *obs) {
    kalmanY = obs;
    CompositeObserver::add(kalmanY);
}
void KalmanComposite::addKalmanZ(KalmanFilter *obs) {
    kalmanZ = obs;
    CompositeObserver::add(kalmanZ);
}

/* ------------------ Kalman ------------------ */

// Constructor implementation
KalmanFilter::KalmanFilter(double dt,
                            const Eigen::MatrixXd& A,
                            const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& C,
                            const Eigen::MatrixXd& R,
                            double sigma_jerk,
                            double sigma_ddfext, 
                            std::string name, int axis):LeafObserver(name, axis, dt, A.rows(),
                            C.rows()){

    Eigen::Matrix2d cov_input;
    cov_input << sigma_jerk, 0, 0, sigma_ddfext;
    Q = B*cov_input*B.transpose();                  // compute covariance process noise
    this->A = A;
    this->C = C;
    this->R = R;
    xAct.resize(n);
}


// KF initialization no guessing
void KalmanFilter::init(){
    int f = pow(10, 2);
    xAct.setZero();
    P.resize(n,n);
    P = f*P.setIdentity();
}

// KF initialization with guessing
void KalmanFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0){
    xAct = x0;
    P = P0;
}

// Update function implementation (for Z observer)
void KalmanFilter::update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y){
    Eigen::Vector3d y;
    y << Y.row(this->axis)[0], Y.row(this->axis)[2], Y.row(this->axis)[1];

    // Prediction step
    xAct = A*xAct;
    P = A*P*A.transpose() + Q;

    // Correction step
    Eigen::MatrixXd K = P*C.transpose()*(C*P*C.transpose()+R).inverse();
    Eigen::MatrixXd NI = y - C*xAct;
    xAct = xAct + K*NI;
    P = P - K*C*P;
}

// @overloaded update function implementation (for X, Y observers)
void KalmanFilter::update(const Eigen::MatrixXd& C, const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y){
    this->C = C;
    update(U, Y);
}

// Returns current covariance estimate
std::map<std::string, Eigen::MatrixXd> KalmanFilter::uncertainty(){
    std::map<std::string, Eigen::MatrixXd> map;
    map[name] = P;
    return map;
}

// -------------------- Stephens -----------------------

// Constructor implementation
StephensFilter::StephensFilter(double dt,
                               const Eigen::MatrixXd& A,
                               const Eigen::MatrixXd& B,
                               const Eigen::MatrixXd& C, 
                               const Eigen::MatrixXd& Q,
                               const Eigen::MatrixXd& R, std::string name, int axis) : LeafObserver(name, axis, dt, A.rows(),
                               C.rows()) {
    this->A = A;
    this->B = B;
    this->C = C;
    this->Q = Q;
    this->R = R;
}


// KF initialization no guessing
void StephensFilter::init(){
    int f = pow(10, 2);
    xAct.setZero();
    P.resize(n,n);
    P = f*P.setIdentity();
}

// KF initialization with guessing
void StephensFilter::init(const Eigen::VectorXd& x0, const Eigen::MatrixXd& P0){
    xAct = x0;
    P = P0;
}

// Update function implementation
void StephensFilter::update(const Eigen::MatrixXd& U, const Eigen::MatrixXd& Y){
    Eigen::VectorXd u(1);
    u = U.row(axis);
    Eigen::Vector2d y;
    if (axis == 2) {
        y << Y.row(axis)[0], 0.0;
    } else {
        y << Y.row(axis)[0], Y.row(axis)[1];
    }
    // Prediction step

    xAct = A*xAct + B*u;
    P = A*P*A.transpose() + Q;

    // Correction step
    Eigen::MatrixXd G = P*C.transpose()*(C*P*C.transpose()+R).inverse();
    Eigen::MatrixXd NI = y - C*xAct;
    xAct = xAct + G*NI;
    P = P - G*C*P;
}


std::map<std::string, Eigen::MatrixXd> StephensFilter::uncertainty(){
    std::map<std::string, Eigen::MatrixXd> map;
    map[name] = P;
    return map;
}

