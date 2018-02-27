
/*****************************************************************************/
/* File:        compar_octrees.cpp                                           */
/* Description: metrics IoU, logged ratio erros, correlation                 */
/*              can be run from ros node                                     */
/*              setpoints for each degree of freedom                         */
/* Email:       alexander.kang@tum.de                                        */
/* Author:      alexander.kang                                               */
/* Date:        12-2017                                                      */
/*****************************************************************************/

/*
 *Metric based on OctoMap
 */
//pcl
#include <pcl/io/pcd_io.h> 
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>

#include <octomap/octomap.h>
#include <fstream>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <list>
#include <cmath>

#ifdef _MSC_VER // fix missing isnan for VC++
#define isnan(x) _isnan(x)  
#endif

using namespace std;

using namespace octomap;

std::string reference_map = "/home/robond/catkin_ws/src/fusion_octomap/binary_maps/test_005.ot";
std::string motion_map = "/home/robond/catkin_ws/src/fusion_octomap/binary_maps/test_005_copy.ot";
std::string single_pose_map = "/home/robond/catkin_ws/src/fusion_octomap/binary_maps/single-pose.ot";
std::string four_poses_map = "/home/robond/catkin_ws/src/fusion_octomap/binary_maps/four-poses.ot";
std::string six_poses_map = "/home/robond/catkin_ws/src/fusion_octomap/binary_maps/six-poses.ot";
//count for tree1
int occ_num1 = 0;
int free_num1 = 0;
int unknown_num1 = 0;
//count for tree2
int occ_num2 = 0;
int free_num2 = 0;
int unknown_num2 = 0;

float occ_percentage = 0.0;
float free_percentage = 0.0;
float unknown_percentage = 0.0;

int intersection[3] = {0,0,0};    //index 0 for occupied space, 1 for free space, 2 for unknown
int unionsection[3] = {0,0,0};
float error = 1e-1;

//float resolution = 0.2;   // unit m

float v1 = 0.0;
float v2 = 0.0;

float occ_score = 0.0;
float free_score = 0.0;
float unknown_score = 0.0;

float resolution1 = 0.0;
float resolution2 = 0.0;

double bb_xmin, bb_xmax, bb_ymin, bb_ymax, bb_zmin, bb_zmax;   //bouding box span over 3d 

double metric_acc_logodds (double &prob1, double &prob2){
//    check for weird case
      if (prob1 < 0.0 || prob1 > 1.0)
        OCTOMAP_ERROR("p1 wrong: %f", prob1);
      if (prob2 < 0.0 || prob2 > 1.0)
        OCTOMAP_ERROR("p2 wrong: %f", prob2);

//    check for waining of inverse case
      if (prob1 > 0.001 && prob2 < 0.001)
        OCTOMAP_WARNING("p2 near 0, p1 > 0 => inf?");
      if (prob1 < 0.999 && prob2 > 0.999)
        OCTOMAP_WARNING("p2 near 1, p1 < 1 => inf?");

      double kld = 0;
      if (prob1 < 0.0001)
        kld =log((1-prob1)/(1-prob2))*(1-prob1);       //quotient can be reciprocal
      else if (prob1 > 0.9999)
        kld =log(prob1/prob2)*prob1;
      else
        kld +=log(prob1/prob2)*prob1 + log((1-prob1)/(1-prob2))*(1-prob1);
      return kld;
}

float metric_IoU (int ins[3], int uo[3]){

    float quotient[3];
    quotient[0] = float(ins[0])/uo[0];
    quotient[1] = float(ins[1])/uo[1];
    quotient[2] = float(ins[2])/uo[2];

    float weight1 = float(uo[0])/(uo[0] + uo[1] + uo[2]);
    float weight2 = float(uo[1])/(uo[0] + uo[1] + uo[2]);
    float weight3 = float(uo[2])/(uo[0] + uo[1] + uo[2]);

    if (weight1 < 0.1){
      weight1 = float(uo[0])/(uo[0] + uo[1]);
      weight2 = float(uo[1])/(uo[0] + uo[1]);
      weight3 = 0.0;
      }
    float score = weight1 * quotient[0] + weight2 * quotient[1] + weight3 * quotient[2];
    cout << "occupeid IoU:" << quotient[0] << endl;
    cout << "free IoU:" << quotient[1] << endl;
    cout <<  "unknown IoU:" << quotient[2] << endl;
    return score;
}
double metric_correlation (double &sum_subproduct, double square1, double square2)
{  
    //cout << sum_subproduct << endl;
    //cout << square1 << endl;
    //cout << square2 << endl;
    float correla_rou = double(sum_subproduct)/double(sqrt(square1 * square2));
    return correla_rou;
}
void display (int &occ_num, int &free_num, int &unknown_num){

  cout << "occupied num:" << occ_num<< endl;
  cout << "free num:" << free_num<< endl;
  cout <<  "unknown num in bounding box:" << unknown_num << endl;

  if (float(occ_num)/(occ_num + free_num + unknown_num) < 0.1)
      {occ_percentage = float(occ_num)/(occ_num + free_num)*100.0;
       free_percentage = float(free_num)/(occ_num + free_num)*100.0;
       unknown_percentage = 0.0;
       cout << "occupied percentage:" << occ_percentage << "%" << endl;
       cout << "free percentage:" << free_percentage << "%" << endl;
       }
  else    
      {occ_percentage = float(occ_num)/(occ_num + free_num + unknown_num)*100.0;
       free_percentage = float(free_num)/(occ_num + free_num + unknown_num)*100.0;
       unknown_percentage = float(unknown_num)/(occ_num + free_num + unknown_num)*100.0;
       cout << "occupied percentage:" << occ_percentage << "%" << endl;
       cout << "free percentage:" << free_percentage << "%" << endl;
       cout << "unknown percentage:" << unknown_percentage << "%" << endl;
      }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "metricNode");
  ros::start(); 
  ros::Time lasttime=ros::Time::now();
  cout << "\nReading octree files...\n";

  OcTree* tree1 = dynamic_cast<OcTree*>(OcTree::read(reference_map));
  OcTree* tree2 = dynamic_cast<OcTree*>(OcTree::read(motion_map));

  resolution1 = tree1->getResolution();
  resolution2 = tree2->getResolution();
  if (fabs(resolution1 - resolution2) > 1e-3){               //1e-6
    OCTOMAP_ERROR("Error: Tree resolutions don't match!");
    exit(-1);
  }


  //cout << "Expanding octrees... \n";
  // expand both to full resolution:
  tree1->expand();
  tree2->expand();

  cout << "Expanded num1 leafs: " << tree1->getNumLeafNodes() << endl;
  cout << "Expanded num2 leafs: " << tree2->getNumLeafNodes() << endl;

  // check the the bounding range along each axis:
  double x1, y1, z1, x1_min, y1_min, z1_min, x1_max, y1_max, z1_max,
         x2, y2, z2, x2_min, y2_min, z2_min, x2_max, y2_max, z2_max;
  tree1->getMetricSize(x1, y1, z1);
  tree1->getMetricMin(x1_min, y1_min, z1_min);
  tree1->getMetricMax(x1_max, y1_max, z1_max);

  tree2->getMetricSize(x2, y2, z2);
  tree2->getMetricMin(x2_min, y2_min, z2_min);
  tree2->getMetricMax(x2_max, y2_max, z2_max);

  resolution1 = tree1->getResolution();
  resolution2 = tree2->getResolution();

  cout << "tree1 x1min: " << x1_min << endl;
  cout << "tree1 y1min: " << y1_min << endl;
  cout << "tree1 z1min: " << z1_min << endl;
  cout << "tree1 x1max: " << x1_max << endl;
  cout << "tree1 y1max: " << y1_max << endl;
  cout << "tree1 z1max: " << z1_max << endl;
  cout << "tree1 x1: " << x1 << endl;
  cout << "tree1 y1: " << y1 << endl;
  cout << "tree1 z1: " << z1 << endl;
  cout << "tree1 resolution: " << tree1->getResolution()<< endl;

  cout << "tree2 x2min: " << x2_min << endl;
  cout << "tree2 y2min: " << y2_min << endl;
  cout << "tree2 z2min: " << z2_min << endl;
  cout << "tree2 x2max: " << x2_max << endl;
  cout << "tree2 y2max: " << y2_max << endl;
  cout << "tree2 z2max: " << z2_max << endl;
  cout << "tree2 x2: " << x2 << endl;
  cout << "tree2 y2: " << y2 << endl;
  cout << "tree2 z2: " << z2 << endl;
  cout << "tree2 resolution: " << tree2->getResolution()<< endl;

  cout << "Comparing trees... \n";
  
  if (x1_min <= x2_min)
     bb_xmin = x1_min;
  else 
     bb_xmin = x2_min;  
  if (y1_min <= y2_min)
     bb_ymin = y1_min;
  else 
     bb_ymin = y2_min;   
  if (z1_min <= z2_min)
     bb_zmin = z1_min;
  else 
     bb_zmin = z2_min;   
   

  if (x1_max >= x2_max)
     bb_xmax = x1_max;
  else 
     bb_xmax = x2_max;  
  if (y1_max >= y2_max)
     bb_ymax = y1_max;
  else 
     bb_ymax = y2_max;   
  if (z1_max >= z2_max)
     bb_zmax = z1_max;
  else 
     bb_zmax = z2_max;   

  for (double ix = bb_xmin; ix < bb_xmax; ix += resolution1)
    for (double iy = bb_ymin; iy < bb_ymax; iy += resolution1)
         for (double iz = bb_zmin; iz < bb_zmax; iz += resolution1){
              if ((!tree1->search(ix,iy,iz)) && (!tree2->search(ix,iy,iz))){
                  unknown_num1 += 1; //This cell is unknown
                  unknown_num2 += 1; //This cell is unknown
                  intersection[2] += 1;
                  unionsection[2] += 1;}
              else{
                  if (!tree1->search(ix,iy,iz)){
                     unknown_num1 += 1; //This cell is unknown                  
                     unionsection[2] += 1;
                     }
                  else if(!tree2->search(ix,iy,iz)){
                     unknown_num2 += 1; //This cell is unknown
                     unionsection[2] += 1;
                     }
                  }
         }

  double kld_sum = 0.0;
  double sum_prob = 0.0;
  double averaged_prob = 0.0;
  double square1 = 0.0;
  double square2 = 0.0;
  double sub1 = 0.0;
  double sub2 = 0.0;
  double sum_subproduct = 0.0;
  for (OcTree::leaf_iterator ref = tree1->begin_leafs(),
      end = tree1->end_leafs();  ref != end; ++ref)
  {
    OcTreeNode* target = tree2->search(ref.getKey());    //search(double x,double y, double z, uint depth = 0)

    if (!target){
      //OCTOMAP_ERROR("Could not find coordinate of 1st octree in 2nd octree\n");
      double b1 = ref->getOccupancy();
      if (b1 >= 0.9){
         occ_num1 += 1; //occupied cells
         unionsection[0] += 1;
         }
      else if (b1 <= 0.15) { //free cells
         free_num1 += 1;
         unionsection[1] += 1;
         }
      else {
         unknown_num1 += 1;
      }
      unknown_num2 += 1;
      unionsection[2] += 1;   
    } 
    else{
      // check occupancy prob:
      double p1 = ref->getOccupancy();
      double p2 = target->getOccupancy();

      if ((p1 >= 0.9) && (p2 >= 0.9)){
          occ_num1 += 1; //occupied cells
          occ_num2 += 1; //occupied cells
          intersection[0] += 1;
          unionsection[0] += 1;}
      else{
          if (p1 >= 0.9){
              occ_num1 += 1;                 
              unionsection[0] += 1;
              }
          else if(p2 >= 0.9){
              occ_num2 += 1;                 
              unionsection[0] += 1;
              }
           }

      if ((p1 <= 0.15) && (p2 <= 0.15)){
          free_num1 += 1; //occupied cells
          free_num2 += 1; //occupied cells
          intersection[1] += 1;
          unionsection[1] += 1;}
      else{
          if (p1 <= 0.15){
              free_num1 += 1;                 
              unionsection[1] += 1;
              }
          else if(p2 <= 0.15){
              free_num2 += 1;                 
              unionsection[1] += 1;
              }
           }

      if ((0.15 < p1 < 0.9) && (0.15 < p1 < 0.9)){
          unknown_num1 += 1; //occupied cells
          unknown_num2 += 1; //occupied cells
          intersection[2] += 1;
          unionsection[2] += 1;}
      else{
          if (0.15 < p1 < 0.9){
               unknown_num1 += 1; //occupied cells
               unionsection[2] += 1;
               }
          else if(0.15 < p2 < 0.9){
               unknown_num2 += 1; //occupied cells
               unionsection[2] += 1;
              }
           }

      //cout << "p1 from tree1:" << p1<< endl;
      //cout << "p2 from tree2:" << p2<< endl;

      //cout << "tree1 value:" << it->getValue()<< endl;
//    sum up the etropy for the undefined operation of floating point
      double mean_logodd = 0;
      //metric for accumulative logodds
      mean_logodd = metric_acc_logodds(p1, p2);
      if (std::isnan(mean_logodd)){
        OCTOMAP_ERROR("KLD is nan! KLD(%f,%f)=%f; sum = %f", p1, p2, mean_logodd, kld_sum);
        exit(-1);
      }

      kld_sum += abs(mean_logodd);
      sum_prob +=  (p1+p2);       //By default the unknown cells have no impact
    }
  }


  if (float(occ_num1)/(occ_num1 + free_num1 + unknown_num1) < 0.1)
       averaged_prob = double(sum_prob)/((intersection[0] + intersection[1]) * 2);
  else
       averaged_prob = double(sum_prob)/((intersection[0] + intersection[1] + intersection[2]) * 2);
  //cout << "sum" << sum_prob << endl;
  cout << "average:" << averaged_prob << endl;

  for (OcTree::leaf_iterator it1 = tree1->begin_leafs(),
      end1 = tree1->end_leafs(); it1 != end1; ++it1){
      OcTreeNode* it2node = tree2->search(it1.getKey());
      if (it2node)
      {double m1 = it1->getOccupancy();
      double m2 = it2node->getOccupancy();
      sub1 = abs(m1 - averaged_prob);
      sub2 = abs(m2 - averaged_prob);
      sum_subproduct += (abs(sub1) * abs(sub2));
      square1 += pow(sub1, 2);
      square2 += pow(sub2, 2);}
      }
    cout << "variance 1:"<< sqrt(square1/(2*(intersection[0] + intersection[1]))) << endl;
    cout << "variance 2:"<< sqrt(square2/(2*(intersection[0] + intersection[1]))) << endl; 
    
    double voume_size = 0.0;
    for(int i =0; i <= intersection[0]; i++)
         voume_size += pow(tree1->getResolution(), 3);
    cout << "volume size:"<< voume_size << endl;
  /*for (OcTree::leaf_iterator it2 = tree2->begin_leafs(),
      end2 = tree2->end_leafs(); it2 != end2; ++it2){
      double m2 = it2->getOccupancy();
      sub2 +=  abs(m2 - averaged_prob);
      square2 += pow(sub2, 2);
      }*/

  /*cout << "intersection 0:" << intersection[0]<< endl;
  cout << "intersection 1:" << intersection[1]<< endl;
  cout << "intersection 2:" << intersection[2]<< endl;
  cout << "unionsection 0:" << unionsection[0]<< endl;
  cout << "unionsection 1:" << unionsection[1]<< endl;
  cout << "unionsection 2:" << unionsection[2]<< endl;*/

  display(occ_num1, free_num1, unknown_num1);
  display(occ_num2, free_num2, unknown_num2);

  cout << "metric logodds: " << kld_sum << endl;

  float final_score  = metric_IoU(intersection, unionsection);
  cout << "metric Iou: " << final_score << endl;

  double correlation = metric_correlation (sum_subproduct, square1, square2);
  cout << "metric correlation: " << correlation << endl;
  ros::Time currtime=ros::Time::now();
  ros::Duration diff=currtime-lasttime;
  cout<<"diff:" <<diff<<endl;
  delete tree1;
  delete tree2;
  
  return 0;
}
