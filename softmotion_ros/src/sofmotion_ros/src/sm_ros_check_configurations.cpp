
/**
\file   sm_ros_check_configurations.cpp
\brief  to test different configurations of softmotion library(developed by laas-france)
 * Differnet configuration of softmotion library based on
 *(pt2pt_traj or n_pts_traj) && (one_dim or n_dim) are:
 *
 *-------------- point2point_approach 1: one_DoF (start2goal, with no time specified)-------------------
 * does not need to specify any mode
 * calculate time automatically
 *-----------------------------------------------------------------------------------------------------*
 *
 *-------------- point2point_approach 2: N_DoF (start2goal, no time, generic to genric state) ----------
 *works with mode SM_TIME_SYNCHRONIZED ==> start and end in same time but phases are different
 *works with mode SM_PHASE_SYNCHRONIZED ==> start and end in same time, phases are synchronized
 *does not work wih phases: SM_3SEGMENT or SM_1SEGMENT;
 * modes are: // SM_INDEPENDANT, SM_PHASE_SYNCHRONIZED,  SM_TIME_SYNCHRONIZED, SM_3SEGMENT, SM_1SEGMENT
 *-----------------------------------------------------------------------------------------------------*
 *
 *-------------- point2point_approach 3: N_DoF (start2goal, with specified time, generic) --------------
 *  works from generic to genric state with specified time
 * workd with mode SM_3SEGMENT so the traj spanned on the given time
 * does not work with SM_1SEGMENT at all
 * work with  SM_PHASE_SYNCHRONIZED and  SM_TIME_SYNCHRONIZED, but calculate time automatically,
 * and do not use the given time with both of them
 * * modes are: // SM_INDEPENDANT, SM_PHASE_SYNCHRONIZED,  SM_TIME_SYNCHRONIZED, SM_3SEGMENT, SM_1SEGMENT
 *-----------------------------------------------------------------------------------------------------*
 *
 *-------------- point2point_approach 4: N_DoF (start2goal, no time, generic to genric state) ----------
 * works point_to_point start2goal with time, without any mode
 * does not have mode argument like approach, but the traj is consist of three phases
 *-----------------------------------------------------------------------------------------------------*
 *
 *-------------- waypoints_approach 5: trajectory via waypoint ----------------------------------------
 * should plan for way point and have three "types":
 *  stop at, approach each waypoint, or pass through waypoint
 * types are:   SM_STOP_AT_VIA_POINT, SM_SMOOTH_APPROACH_VIA_POINT, SM_SMOOTH_AT_VIA_POINT
 *-----------------------------------------------------------------------------------------------------*
 *
 * uncomment any approach to check its output
\author  Mahmoud Ali
\date    1/5/2019
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
  SM_TRAJ::SM_TRAJ_MODE mode;
  mode = SM_TRAJ::SM_TIME_SYNCHRONIZED ;  //
  //mode = SM_TRAJ::SM_PHASE_SYNCHRONIZED;  //
  //mode = SM_TRAJ::SM_3SEGMENT;            //
  //mode = SM_TRAJ::SM_1SEGMENT;            //

  double t=1.5;
  SM_COND IC;
  SM_COND FC;
  SM_LIMITS lmt;
  IC.a = .1;
  IC.v = .2;
  IC.x = .3;
  FC.a = 1;
  FC.v = 2;
  FC.x = 3;
  lmt.maxAcc = 50;
  lmt.maxVel = 50;
  lmt.maxJerk = 500;


  /*-------------- approach 1: one_DoF (no time specified): works ----------------
  * does not need to specify any mode
  * calculate time automatically
  *-----------------------------------------------------------------------------------------------------*/
  //int check= traj.computeOneDimTraj(IC, FC , lmt);
  //std::cout<<"check  :  "<< check <<std::endl;


  // iniialize variables for 2d approach
  std::vector<SM_COND> IC_vec,FC_vec;
  std::vector<SM_LIMITS> lmts;

  n_jts=6;  //n_pts=3;
  double d=0, last_d=0; t=0;
  std::vector<SM_COND> cond;
  std::vector< std::vector<SM_COND> > sampled_traj;
  std::vector< std::vector<double> > pos,vel,acc;
  std::vector<double> T_vec;

// for each seg of the trajectory
  for (int pt=0; pt< n_pts-1; pt++){
      ROS_INFO_STREAM("=================== Seg_"<<pt<< " =====================");


      IC_vec.clear();
      FC_vec.clear();
      lmts.clear();
      for (int jt=0; jt< n_jts; jt++){
          // set initial and final condition for eac joint
          IC.x = P_jt_wpt[jt][pt];
          IC.v = V_jt_wpt[jt][pt];
          IC.a = A_jt_wpt[jt][pt];

          FC.x = P_jt_wpt[jt][pt+1];
          FC.v = V_jt_wpt[jt][pt+1];
          FC.a = A_jt_wpt[jt][pt+1];
          // set limits for eac joint
          lmt.maxAcc = 50;
          lmt.maxVel = 50;
          lmt.maxJerk = 500;

          IC_vec.push_back(IC);
          FC_vec.push_back(FC);
          lmts.push_back(lmt);
      }


      //print IC and FC for each joint
      for (int jt=0; jt< n_jts; jt++){
          ROS_INFO_STREAM("jt_"<< jt<<  ": IC(x,v,a)=  " << IC_vec[jt].x <<", "<<IC_vec[jt].v <<", "<<IC_vec[jt].a <<" ==> FC(x,v,a)=  " << FC_vec[jt].x <<", "<<FC_vec[jt].v <<", "<<FC_vec[jt].a);
      }

      mode = SM_TRAJ::SM_TIME_SYNCHRONIZED ;
      int returned_error= traj.computeTraj(IC_vec, FC_vec, lmts, mode);
      std::cout<<"check2  :  "<< returned_error <<std::endl;
      if(returned_error !=0){
          std::cout<<" \n -------------------------------- \n can not synchronize the motion\n -------------------------------- \n";
          break;
      }

      t=0; //last_d;
      d= traj.getSegDuration() ;
      while (t<(d-0.008)) {
          traj.getMotionCond(t, cond);
          sampled_traj.push_back(cond);
          T_vec.push_back(t+last_d);
          t=t+.008;
//          ROS_INFO_STREAM("cond.x .v. a"<< cond[0].x<< "   "<< cond[0].v <<"  " << cond[0].a);
//          for (int i=0; i<6; i++)
//              ROS_INFO_STREAM("cond.x .v. a:  "<< cond[i].x<< "    "<< cond[i].v <<"   " << cond[i].a);
//              ROS_INFO_STREAM("----- ");
      }
//      traj.plot();



  } //end of for each point pt

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





  /*-------------- approach 2: N_DoF point2point (start2goal): works from generic to genric state---------
  *works with mode SM_TIME_SYNCHRONIZED ==> start and end in same time but phases are different
  *works with mode SM_PHASE_SYNCHRONIZED ==> start and end in same time, phases are synchronized
  *does not work wih phases: SM_3SEGMENT or SM_1SEGMENT;
  * modes are: // SM_INDEPENDANT, SM_PHASE_SYNCHRONIZED,  SM_TIME_SYNCHRONIZED, SM_3SEGMENT, SM_1SEGMENT
  *-----------------------------------------------------------------------------------------------------*/
//  mode = SM_TRAJ::SM_TIME_SYNCHRONIZED ;
//  int check2= traj.computeTraj(IC_vec, FC_vec, lmts, mode);
//  std::cout<<"check2  :  "<< check2 <<std::endl;



//  // iniialize variables for 3rd approach
//  std::vector<double> imposedDuration;
//  for (int jt=0; jt< n_jts; jt++)
//      imposedDuration.push_back(t);


  /*-------------- approach 3: N_DoF point2point (start2goal) with specified time ---------------------
   *  works from generic to genric state with specified time
   * workd with mode SM_3SEGMENT so the traj spanned on the given time
   * does not work with SM_1SEGMENT at all
   * work with  SM_PHASE_SYNCHRONIZED and  SM_TIME_SYNCHRONIZED, but calculate time automatically,
   * and do not use the given time with both of them
   * * modes are: // SM_INDEPENDANT, SM_PHASE_SYNCHRONIZED,  SM_TIME_SYNCHRONIZED, SM_3SEGMENT, SM_1SEGMENT
  *-----------------------------------------------------------------------------------------------------*/
  //mode= SM_TRAJ::SM_3SEGMENT;
  //int check3= traj.computeTraj(IC_vec, FC_vec, lmts, mode, imposedDuration);
  //std::cout<<"check3  :  "<< check3 <<std::endl;
  //for (int jt=0; jt< n_jts; jt++)
  //    std::cout<<"imposedDuration["<<jt <<"]  :  "<< imposedDuration[jt] <<std::endl;




//  // iniialize variables for 4th approach
//  std::vector<double> waypt;
//  std::deque< std::vector<double> > waypts;
//  waypt.resize(n_jts); //

//  double pt1[]={2, 3, 5};
//  for (int jt=0; jt< n_jts; jt++)
//      waypt[jt] = pt1[jt];
//  waypts.push_back( waypt);

//  double pt2[]={4, 2, 2};
//  for (int jt=0; jt< n_jts; jt++)
//      waypt[jt] = pt2[jt];
//  waypts.push_back( waypt);

//  double pt3[]={5, 0, 4};
//  for (int jt=0; jt< n_jts; jt++)
//      waypt[jt] = pt3[jt];
//  waypts.push_back( waypt);
  /*-------------- approach 4: trajectory via waypoint ---------------------
   * works point_to_point start2goal with time
   * does not have mode argument but the traj is consist of three phases
  *-----------------------------------------------------------------------------------------------------*/
  //int check1= traj.compute3SegmentTrajLimitJerkAccVel( IC_vec,  FC_vec, lmts, imposedDuration) ;
  //std::cout<<"check1  :  "<< check1 <<std::endl;


  /*-------------- approach 5: trajectory via waypoint ---------------------
   * should plan for way point and have three "types" (not modes):
   *  stop at, approach eaach, pass through waypoint
   * types are:     SM_STOP_AT_VIA_POINT, SM_SMOOTH_APPROACH_VIA_POINT, SM_SMOOTH_AT_VIA_POINT
  *-----------------------------------------------------------------------------------------------------*/
//  SM_TRAJ::SM_TRAJ_TYPE type = SM_TRAJ::SM_STOP_AT_VIA_POINT;
//  int check_via_pts = traj.computeTrajViaPoints( waypts, lmts, type);
//  std::cout<<"returned valuept:  "<< check_via_pts <<std::endl;
//  traj.save("saved");






//  traj.print();
//  traj.plot();


//  for (int jt=0; jt<n_jts; jt++)
//      traj.plot(jt);





//  ros::Rate loop_rate(10);
//  while (ros::ok())
//  {
//    std_msgs::String msg;
//    msg.data = "hello world";

//    chatter_pub.publish(msg);

//    ros::spinOnce();

//    loop_rate.sleep();
//  }





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
