
/**
\file   sm_ros_npts_traj.cpp
\brief  to generate trajectory using the softmotion_library developed by Laas-france
 *
\author  Mahmoud Ali
\date    3/5/2019
*/



#include<vector>
#include<iostream>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"


#include "softMotion/Sm_Traj.h"
#include "normal_toppra_traj_instant.h"
//#include"moveit_spline_interp_trajectory_axis5_instant.h"
#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "softmotion_node");
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);


    // read trajectory:
    //============ read trajectory ==========
    trajectory_msgs::JointTrajectory  traj_msg;
    traj_msg = generate_traj();
    int n_jts = traj_msg.joint_names.size();
    int n_pts = traj_msg.points.size();
    ROS_INFO_STREAM( "number of joints, and points: "<< n_jts << ", "<<n_pts );
    ros::Rate init_delay( 1);
    init_delay.sleep();

    // variables
    std::vector< std::vector<double> > P_jt_wpt, V_jt_wpt, A_jt_wpt;
    P_jt_wpt.resize( n_jts);
    V_jt_wpt.resize( n_jts);
    A_jt_wpt.resize( n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            P_jt_wpt[jt].push_back( traj_msg.points[pt].positions[jt] );
            V_jt_wpt[jt].push_back( traj_msg.points[pt].velocities[jt] );
            A_jt_wpt[jt].push_back( traj_msg.points[pt].accelerations[jt] );
        }
    }


    for(int jt=0; jt<n_jts; jt++){
        // initial point, to be assigned to current pose
        P_jt_wpt[jt][0] =  0;
        V_jt_wpt[jt][0] =  0;
        A_jt_wpt[jt][0] =  0;
        // overwrite last point vel and acc to zeros
        V_jt_wpt[jt][n_pts-1] =  0;
        A_jt_wpt[jt][n_pts-1] =  0;
    }

    // remove noise condition for all changes less than 0.001 from previous location
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=1; pt<n_pts; pt++){
            if( fabs(P_jt_wpt[jt][pt] - P_jt_wpt[jt][pt-1]) < 1e-3 )
                P_jt_wpt[jt][pt] =   P_jt_wpt[jt][pt-1];
            if(fabs(V_jt_wpt[jt][pt] - V_jt_wpt[jt][pt-1] ) < 1e-3 )
                V_jt_wpt[jt][pt] =   V_jt_wpt[jt][pt-1];
            if(fabs(A_jt_wpt[jt][pt] - A_jt_wpt[jt][pt-1])  < 1e-3 )
                A_jt_wpt[jt][pt] =   A_jt_wpt[jt][pt-1];
        }
    }

    // print values for each waypoint (pos. vel. acc.): to check everything going well
    for(int pt=1; pt<n_pts; pt++){
        for(int jt=0; jt<n_jts; jt++)
            ROS_INFO_STREAM(" pt:"<< pt <<", jt:"<< jt<< ": pos, vel, acc="  <<  P_jt_wpt[jt][pt] <<  ",  "  <<  V_jt_wpt[jt][pt]  <<  ",  "  <<  A_jt_wpt[jt][pt] );
        ROS_INFO("\n " );
    }

    //extract times for each point from trajectory
    std::vector<double> T_wpt;
    T_wpt.resize(n_pts);
    for(int pt=0; pt<n_pts; pt++)
        T_wpt[pt] = traj_msg.points[pt].time_from_start.toSec()*1e-9 ;



    //define trajectory object
    SM_TRAJ traj;
    ROS_INFO(" SM_TRAJ object is created .... ");


    // inialize  variables to check different approaches
    SM_LIMITS lmt;
    std::vector<SM_LIMITS> lmts;
    double d=0, last_d=0, t=0;
    std::vector<SM_COND> cond;
    std::vector< std::vector<SM_COND> > sampled_traj;
    std::vector< std::vector<double> > pos,vel,acc;
    std::vector<double> T_vec;

    // set limits for eac joint
    for (int jt=0; jt< n_jts; jt++){
        lmt.maxAcc = 50;
        lmt.maxVel = 50;
        lmt.maxJerk = 500;
        lmts.push_back(lmt);
    }

    // iniialize variables
    std::vector<double> waypt;
    std::deque< std::vector<double> > waypts;
    waypt.resize(n_jts); //

    // for each point inside the traj for eac joint
    for (int pt=0; pt< n_pts; pt++) {
        for (int jt=0; jt< n_jts; jt++) {
            waypt[jt] = P_jt_wpt[jt][pt]  ;
        }
        waypts.push_back( waypt);
    }

    /*-------------- approach 5: trajectory via waypoint ---------------------
   * should plan for way point and have three "types":
   *  stop at, approach eaach, pass through waypoint
   * types are:     SM_STOP_AT_VIA_POINT, SM_SMOOTH_APPROACH_VIA_POINT, SM_SMOOTH_AT_VIA_POINT
  *-----------------------------------------------------------------------------------------------------*/
    SM_TRAJ::SM_TRAJ_TYPE type = SM_TRAJ::SM_SMOOTH_APPROACH_VIA_POINT;
    int returned_error = traj.computeTrajViaPoints( waypts, lmts, type);
    std::cout<<"joints are synchronized, errors=  "<< returned_error <<std::endl;

    // sampling with freq =125
    t=0; //last_d;
    d= traj.getSegDuration() ;
    std::cout<<"duration:  "<< d <<std::endl;
    while (t<(d-0.008)) {
        traj.getMotionCond(t, cond);
        sampled_traj.push_back(cond);
        T_vec.push_back(t+last_d);
        t=t+.008;
    }


    //store pos, vel, accc values to plot them using matplotlib
    pos.resize( 6);
    vel.resize( 6);
    acc.resize( 6);

    for (int jt=0; jt<6; jt++) {
        for (int i=0; i<sampled_traj.size(); i++) {
            pos[jt].push_back(sampled_traj[i][jt].x);
            vel[jt].push_back(sampled_traj[i][jt].v);
            acc[jt].push_back(sampled_traj[i][jt].a);
            //            ROS_INFO_STREAM("pos:vel:acc=   "<< pos[i]<< "  :  "<< vel[i] <<"  :  " << acc[i]);
        }
    }

    //  traj.plot();// plot traj using GNU (implicit plotting method of the SM lib)

    //plotting
    std::string jt_name;
    std::vector< std::string> clr={"b", "k", "r", "g", "m", "c" }; //so joints colors are same for different subplots
    bool sbplt= true;
    for (int jt=0; jt<6; jt++) {
        jt_name= "jt_" + std::to_string(jt);
        if(sbplt)
            plt::subplot(3,1,1);
        plt::named_plot( jt_name, pos[jt], clr[jt]);// , T_vec);
        plt::title("position"); // Add graph title
        plt::legend(); // Enable legend.
        //    plt::show();
    }
    plt::grid(true);
    if(!sbplt)
        plt::show();


    for (int jt=0; jt<6; jt++) {
        jt_name= "jt_" + std::to_string(jt);
        if(sbplt)
            plt::subplot(3,1,2);
        plt::named_plot( jt_name, vel[jt], clr[jt]);// , T_vec);
        plt::title("velocity"); // Add graph title
        plt::legend(); // Enable legend.
    }
    plt::grid(true);
    if(!sbplt)
        plt::show();



    for (int jt=0; jt<6; jt++) {
        jt_name= "jt_" + std::to_string(jt);
        if(sbplt)
            plt::subplot(3,1,3);
        plt::named_plot( jt_name, acc[jt], clr[jt]);// , T_vec);
        plt::title("acceleration"); // Add graph title
        plt::legend(); // Enable legend.
    }
    plt::grid(true);
    plt::show();



    return 0;
}
