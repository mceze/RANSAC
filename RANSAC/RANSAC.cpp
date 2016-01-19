//
//  RANSAC.cpp
//  RANSAC
//
//  Created by Marco Ceze on 11/8/15.
//  Copyright © 2015 Marco Ceze. All rights reserved.
//

#include <iostream>
#include <vector>
#include <list>
#include <iterator>
#include <Eigen/Dense>
#include <string>


using namespace std;
using namespace Eigen;

//----------------------------------------------------------//
// Generic data fit model to be used in RANSAC
template <class D>
class ransac_model {
public:
  //minimun number of samples needed
  int nDataMin;
  //number of inliers for model (NOTE: nInliers >= nDataMin)
  signed long nInliers;
  //threshold for discerning inliers from outliers
  double threshold;
  
  //constructor setting values to uninitialized
  ransac_model(){
    nDataMin = nInliers = -1;
    threshold = -1.0;
  };
  //destructor
  ~ransac_model(){
    nInliers = -1;
  };
  
  //list of observations used for fit
  list<D> fit_set;
  
protected:
  //method for computing parameter. set to private so it is only
  //called when a new item is added to the fit_set
  //NEED to define this for each specific model
  virtual void compute_param(){};
  //calculates distance of fit to data
  //NEED to define this for each specific model
  virtual double fit_distance(D& data){return 0.0;};
  //checks if data is an inlier
  virtual bool is_inlier(D& data){return false;};
public:
  //add new item to fit_set
  void add_2_fit_set(D& data){
    fit_set.push_back(data);
    nInliers = fit_set.size();
    //update fit parameters
    compute_param();
  };
  //clear model
  void clear(){
    nInliers = -1;
    fit_set.erase(fit_set.begin(),fit_set.end());
  };
};

//----------------------------------------------------------//
//Define a 2D line model
class line2D:public ransac_model<Vector2d*> {
public:
  //child specific parameters
  double slope, intercept;
  
  //constructor set the default values
  line2D(){
    nDataMin = 2;
  };
  
  //define line2D copy
  line2D& operator= (line2D other){
    nDataMin  = other.nDataMin;
    nInliers  = other.nInliers;
    threshold = other.threshold;
    fit_set   = other.fit_set;
    slope     = other.slope;
    intercept = other.intercept;
    
    return *this;
  }

  
  //define parameter computation
  virtual void compute_param(){
    Matrix2d M;
    Vector2d b, *point, param;
    list<Vector2d*>::iterator obs;
    
    M << 0.0, 0.0, 0.0, 0.0;
    b << 0.0, 0.0;
    
    //check if we have enough data in fit_set
    int ndata = (int)fit_set.size();
    if (ndata >= nDataMin) {
      //do linear least-squares fit
      for (obs = fit_set.begin(); obs != fit_set.end(); obs++) {
        point = *obs;
        M(0,0) += 1;
        M(0,1) += (*point)(0);
        M(1,0) += (*point)(0);
        M(1,1) += (*point)(0)*(*point)(0);
        b(0)   += (*point)(1);
        b(1)   += (*point)(0)*(*point)(1);
      }
      //calculate parameters
      param = M.inverse()*b;
      intercept = param(0);
      slope     = param(1);
    }
  };
  //compute fit distance
  virtual double fit_distance(Vector2d& point) {
    double dist;
    //calculate distance
    dist  = fabs(slope*point(0)-point(1)+intercept);
    dist /= sqrt(slope*slope+1.0);
    return dist;
  };
  bool is_inlier(Vector2d& data){
    if (threshold < 0.0)
      throw std::invalid_argument("Uninitialized threshold");
    return (fit_distance(data) <= threshold);
  };
};

//----------------------------------------------------------//
//pick random number from list
template <class T>
T random_element(list<T> L)
{

  const unsigned long n = std::distance(L.begin(), L.end());
  const unsigned long k = rand()%n;
  
  typename list<T>::iterator it = L.begin();
  
  advance(it,k);
  
  return *(it);
};

//----------------------------------------------------------//
//main program
int main(int argc, const char * argv[]) {
  FILE *fid;
  fid = fopen("line2d.txt", "r");
  int npoint, i, it, nitermax = 100;
  double buf[2];
  
  
  fscanf(fid, "%d\n",&npoint);
  
  std::list<Vector2d*> PointList;
  
  for (i = 0; i < npoint; i++) {
    fscanf(fid, "%lf %lf\n",&buf[0],&buf[1]);
    Vector2d *Point = new Vector2d();
    (*Point)(0) = buf[0];
    (*Point)(1) = buf[1];
    PointList.push_back(Point);
  }
  fclose(fid);
  
  //construct a 2d model
  line2D model, model_opt;
  
  //define model's threshold
  model.threshold = 0.7;
  
  Vector2d *P;
  list<Vector2d*>::iterator Obs;
  
  for (it = 0; it < nitermax; it++){
    //run one iteration of the RANSAC algorithm...
    //pick random minimum fit set
    for (i = 0; i < model.nDataMin; i++) {
      P = random_element<Vector2d*>(PointList);
      model.add_2_fit_set(P);
      PointList.remove(P);
    }
    for (Obs = PointList.begin(); Obs != PointList.end(); Obs++) {
      if (model.is_inlier(**Obs)) {
        model.nInliers++;
      }
    }
    
    //store optimum model
    if (model.nInliers > model_opt.nInliers){
      model_opt = model;
      std::cout << "slope: " << std::endl << model_opt.slope;
      std::cout << " intercept: " << model_opt.intercept;
      std::cout << " ninliers: " << model_opt.nInliers << std::endl;
    }
    //restore list
    Obs = model.fit_set.begin();
    for (i = 0; i < model.nDataMin; i++) {
      PointList.push_back(*(Obs));
      Obs++;
    }
    //clear previous model
    model.clear();
    
  }

  //clean up
  for (Obs = PointList.begin(); Obs != PointList.end(); Obs++) {
    delete *Obs;
  }

  
  return 0;
}


