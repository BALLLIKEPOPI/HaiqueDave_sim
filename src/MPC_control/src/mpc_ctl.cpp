#include "../include/mpc_ctl.h"
#include "ros/publisher.h"
#include <asm-generic/errno.h>
#include <casadi/core/calculus.hpp>
#include <casadi/core/function.hpp>
#include <casadi/core/generic_matrix.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <iostream>
#include <vector>

MPC_CTL::MPC_CTL(){
    Q(0, 0) = 100; Q(3, 3) = 100;
    Q(1, 1) = 1000; Q(4, 4) = 1;
    Q(2, 2) = 1; Q(5, 5) = 1;
    // Q(6, 6) = 100; Q(7, 7) = 100;
    // Q(8, 8) = 100; Q(9, 9) = 1;
    // Q(10, 10) = 1; Q(11, 11) = 1;

    R(0, 0) = 10; R(1, 1) = 10;
    R(2, 2) = 10; R(3, 3) = 10;
    R(4, 4) = 10; R(5, 5) = 10;
    R(6, 6) = 10; R(7, 7) = 10;
    R(8, 8) = 5; R(9, 9) = 5;

    st = X(all, 0);
    g(Slice(0, n_state)) = st - P(Slice(0, n_state));
    for(int i = 0; i < N; i++){
        st = X(all, i);
        con = U(all, i);
        obj += SX::mtimes({(st-P(Slice(n_state, n_state*2))).T(), Q, (st-P(Slice(n_state, n_state*2)))}) +
                SX::mtimes({con.T(), R, con});
        st_next = X(all, i+1);
        // RK4
        k1 = Dyna_Func(st, con);
        k2 = Dyna_Func(st + h/2*k1, con);
        k3 = Dyna_Func(st + h/2*k2, con);
        k4 = Dyna_Func(st + h*k3, con);
        st_next_RK4 = st + h/6*(k1+k2+k3+k4);
        // compute constraints 
        g(Slice((i+1)*n_state+i*3, (i+2)*n_state+i*3)) = st_next - st_next_RK4;
        g(Slice((i+2)*n_state+i*3, (i+2)*n_state+i*3+1)) = U(1,i)-U(5,i);
        g(Slice((i+2)*n_state+i*3+1, (i+2)*n_state+i*3+2)) = U(3,i)-U(7,i);
        g(Slice((i+2)*n_state+i*3+2, (i+2)*n_state+i*3+3)) = U(8,i)*U(9,i);
    }
    // make the decision variable one column vector
    OPT_variables = SX::vertcat({SX::reshape(X, n_state*(N+1), 1), SX::reshape(U, 10*N, 1)});
    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}, {"p", P}};
    Dict opts = { {"ipopt.max_iter", 49}, {"ipopt.print_level", 0}, {"print_time", 1}, 
                    {"ipopt.acceptable_tol", 1e-3}, {"ipopt.acceptable_obj_change_tol", 1e-3}};
    solver = nlpsol("solver", "ipopt", nlp, opts);

    // Initial guess and bounds for the optimization variables
    // x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -85.0, 0.0, 0.0, 0.0}; // initial state
    // xs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -85.0, 0.0, 0.0, 0.0}; // desire state
    x0 = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // initial state
    xs = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // desire state
    X0 = SX::repmat(x0, 1, N+1);
    
    // state_lower_bound = {-pi/2, -pi/2, -pi/2, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf, -inf};
    // state_upper_bound = {pi/2, pi/2, pi/2, inf, inf, inf, inf, inf, inf, inf, inf, inf};
    state_lower_bound = {-pi/2, -pi/2, -pi/2, -inf, -inf, -inf};
    state_upper_bound = {pi/2, pi/2, pi/2, inf, inf, inf};
    // con_lower_bound_n = {0, 0, 0, -2, 0, 0, 0, -2, 0, -pi/2};
    // con_upper_bound_n = {2, 2, 2, 0, 2, 2, 2, 0,pi/2, pi/2};
    con_lower_bound = {0, -2, 0, -2, 0, -2, 0, -2, 0, 0};
    con_upper_bound = {2, 2, 2, 2, 2, 2, 2, 2,0, 0};
    con_lower_bound_n = {0, 0, 0, -2, 0, 0, 0, -2, -pi/2, 0};
    con_upper_bound_n = {2, 2, 2, 0, 2, 2, 2, 0,0, pi/2};
    con_lower_bound_p = {0, -2, 0, 0, 0, -2, 0, 0, 0 , -pi/2};
    con_upper_bound_p = {2, 0, 2, 2, 2, 0, 2, 2,pi/2, 0};
    for(int i = 0; i < N+1; i++){
        lbx.insert(lbx.end(), state_lower_bound.begin(), state_lower_bound.end());
        ubx.insert(ubx.end(), state_upper_bound.begin(), state_upper_bound.end());
    }
    for(int i = 0; i < N; i++){
        lbx.insert(lbx.end(), con_lower_bound.begin(), con_lower_bound.end());
        ubx.insert(ubx.end(), con_upper_bound.begin(), con_upper_bound.end());
    }
    lbg.insert(lbg.begin(), n_state*(N+1)+3*N, 0);
    ubg.insert(ubg.begin(), n_state*(N+1)+3*N, 0);

    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
}

MPC_CTL::~MPC_CTL(){
}

void MPC_CTL::solve(){
    updatePara(); // set the values of the parameters vector
    arg["p"] = para;
    if(x0[0]-xs[0]<0.1){
        for(int i = 0; i < N; i++){
            std::copy(con_lower_bound_n.begin(), con_lower_bound_n.end(), 
                lbx.begin() + (N+1) * state_lower_bound.size() + i * con_lower_bound_n.size());
            std::copy(con_upper_bound_n.begin(), con_upper_bound_n.end(), 
                ubx.begin() + (N+1) * state_upper_bound.size() + i * con_upper_bound_n.size());
        }
    }
    else if(x0[0]-xs[0]>=0.1){
        for(int i = 0; i < N; i++){
            std::copy(con_lower_bound_p.begin(), con_lower_bound_p.end(), 
                lbx.begin() + (N+1) * state_lower_bound.size() + i * con_lower_bound_p.size());
            std::copy(con_upper_bound_p.begin(), con_upper_bound_p.end(), 
                ubx.begin() + (N+1) * state_upper_bound.size() + i * con_upper_bound_p.size());
        }
    }
    else {
        for(int i = 0; i < N; i++){
            std::copy(con_lower_bound.begin(), con_lower_bound.end(), 
                lbx.begin() + (N+1) * state_lower_bound.size() + i * con_lower_bound.size());
            std::copy(con_upper_bound.begin(), con_upper_bound.end(), 
                ubx.begin() + (N+1) * state_upper_bound.size() + i * con_upper_bound.size());
        }
    }
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    // initial value of the optimization variables
    arg["x0"] = SX::vertcat({SX::reshape(X0.T(), n_state*(N+1), 1),
                                SX::reshape(u0.T(), 10*N, 1)});
    res = solver(arg);
    getFirstCon();
}

void MPC_CTL::updatePara(){
    para.clear();
    para.insert(para.end(), x0.begin(), x0.end());
    para.insert(para.end(), xs.begin(), xs.end());
}

void MPC_CTL::getFirstCon(){
    u_f.clear();
    vector<double> u_f_ = res.at("x").get_elements();
    u_f.insert(u_f.begin(), u_f_.begin()+n_state*(N+1), u_f_.begin()+10+n_state*(N+1));
    cout << u_f << endl;
    
    mpc_control::controlPub uf_msg;
    uf_msg.thrust1 = u_f[0];
    uf_msg.thrust2 = u_f[1];
    uf_msg.thrust3 = u_f[2];
    uf_msg.thrust4 = u_f[3];
    uf_msg.thrust5 = u_f[4];
    uf_msg.thrust6 = u_f[5];
    uf_msg.thrust7 = u_f[6];
    uf_msg.thrust8 = u_f[7];
    uf_msg.alpha = u_f[8];
    uf_msg.beta = u_f[9];
    conPub.publish(uf_msg);
}

void MPC_CTL::updatex0(double phi_, double theta_, double psi_, double p_, double q_, double r_,
                    double x_, double y_, double z_, double u_, double v_, double w_){
    // x_last = x0;
    x0.clear();
    x0.push_back(phi_);
    x0.push_back(theta_);
    x0.push_back(psi_);
    x0.push_back(p_);
    x0.push_back(q_);
    x0.push_back(r_);
    // x0.push_back(x_);
    // x0.push_back(y_);
    // x0.push_back(z_);
    // x0.push_back(u_);
    // x0.push_back(v_);
    // x0.push_back(w_);

    // x_dot.clear();
    // x_dot.push_back((x0[0]-x_last[0])/h);
    // x_dot.push_back((x0[1]-x_last[1])/h);
    // x_dot.push_back((x0[2]-x_last[2])/h);
}

void MPC_CTL::updatexs(){

}