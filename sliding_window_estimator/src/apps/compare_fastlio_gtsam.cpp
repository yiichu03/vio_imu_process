#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {
using Blocks=std::map<std::string,Eigen::MatrixXd>;
bool nextLine(std::istream &in,std::string &line) {
  while(std::getline(in,line)) {
    line=line.substr(0,line.find('#'));
    auto b=line.find_first_not_of(" \t\r");
    if(b==std::string::npos)continue;
    line=line.substr(b,line.find_last_not_of(" \t\r")-b+1); return true;
  }
  return false;
}
double number(const std::string &s) {
  size_t n=0; double v=std::stod(s,&n);
  if(n!=s.size()||!std::isfinite(v))throw std::runtime_error("Invalid/non-finite number: "+s);
  return v;
}
Blocks read(const std::string &file) {
  std::ifstream in(file); if(!in)throw std::runtime_error("Cannot open "+file);
  Blocks out; std::string line;
  while(nextLine(in,line)) {
    std::istringstream header(line); std::string name,shape,extra;
    if(!(header>>name>>shape)||(header>>extra))throw std::runtime_error("Bad block header: "+line);
    int rows=0,cols=0; char l=0,x=0,r=0; std::istringstream dims(shape);
    if(!(dims>>l>>rows>>x>>cols>>r)||(dims>>extra)||l!='('||x!='x'||r!=')'||rows<1||cols<1||rows>100||cols>100)
      throw std::runtime_error("Bad matrix shape: "+shape);
    if(out.count(name))throw std::runtime_error("Duplicate block: "+name);
    Eigen::MatrixXd m(rows,cols);
    for(int i=0;i<rows;++i) {
      if(!nextLine(in,line))throw std::runtime_error("Truncated block: "+name);
      std::istringstream row(line); std::string token;
      for(int j=0;j<cols;++j){if(!(row>>token))throw std::runtime_error("Short row in "+name);m(i,j)=number(token);}
      if(row>>extra)throw std::runtime_error("Extra values in "+name);
    }
    out.emplace(name,m);
  }
  return out;
}
const Eigen::MatrixXd &block(const Blocks &all,const std::string &key,int rows,int cols) {
  auto it=all.find(key);if(it==all.end())throw std::runtime_error("Missing block: "+key);
  if(it->second.rows()!=rows||it->second.cols()!=cols)throw std::runtime_error("Unexpected dimensions: "+key);
  return it->second;
}
bool compare(const Eigen::MatrixXd &a,const Eigen::MatrixXd &b,double abs,double rel,const std::string &label) {
  int failed=0,wr=0,wc=0;double worst=-1,maxDiff=0;
  for(int r=0;r<a.rows();++r)for(int c=0;c<a.cols();++c){
    const double d=std::abs(a(r,c)-b(r,c)),tol=abs+rel*std::max(std::abs(a(r,c)),std::abs(b(r,c)));
    maxDiff=std::max(maxDiff,d);if(d>tol)++failed;
    if(d-tol>worst){worst=d-tol;wr=r;wc=c;}
  }
  std::cout<<(failed?"[FAIL] ":"[ OK ] ")<<label<<" failures="<<failed<<" max_abs="<<maxDiff;
  if(failed)std::cout<<" worst=("<<wr<<","<<wc<<") actual="<<a(wr,wc)<<" reference="<<b(wr,wc)
      <<" diff="<<std::abs(a(wr,wc)-b(wr,wc))<<" tol="<<abs+rel*std::max(std::abs(a(wr,wc)),std::abs(b(wr,wc)));
  std::cout<<"\n";return failed==0;
}
bool covariance(const Eigen::MatrixXd &P,const std::string &label) {
  double sym=(P-P.transpose()).cwiseAbs().maxCoeff();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> e(0.5*(P+P.transpose()));
  bool ok=e.info()==Eigen::Success && sym<1e-8 && e.eigenvalues().minCoeff()>=-1e-8;
  std::cout<<(ok?"[ OK ] ":"[FAIL] ")<<label<<" symmetry="<<sym;
  if(e.info()==Eigen::Success)std::cout<<" min_eigen="<<e.eigenvalues().minCoeff();
  std::cout<<"\n";return ok;
}
}
int main(int argc,char **argv) {
  try {
    std::string fast,ref;double abs=1e-4,rel=1.5e-2;
    for(int i=1;i<argc;++i){
      const std::string a=argv[i];
      auto next=[&](){if(++i>=argc)throw std::runtime_error("Missing value for "+a);return std::string(argv[i]);};
      if(a=="--fastlio_all")fast=next();else if(a=="--gtsam_all")ref=next();
      else if(a=="--abs_tol")abs=number(next());else if(a=="--rel_tol")rel=number(next());
      else if(a=="--help"){std::cout<<"--fastlio_all FILE --gtsam_all FILE [--abs_tol A --rel_tol R]\n";return 0;}
      else throw std::runtime_error("Unknown argument: "+a);
    }
    if(fast.empty()||ref.empty()||abs<0||rel<0)throw std::runtime_error("Require input files and nonnegative tolerances");
    const Blocks f=read(fast),g=read(ref);
    const auto &S=block(f,"Sigma_z_fastlio_gtsam",15,15),&Sr=block(g,"Sigma_z_gtsam",15,15);
    const auto &J=block(f,"JincBias_ba_bg_fastlio",9,6),&Jr=block(g,"JincBias_ba_bg_gtsam",9,6);
    std::cout<<std::setprecision(17)<<"Matrix tolerances: abs="<<abs<<" rel="<<rel<<"\n";
    bool ok=compare(S,Sr,abs,rel,"Sigma_z15");ok &= compare(J,Jr,abs,rel,"JincBias_ba_bg9x6");
    ok &= covariance(S,"FAST-LIO covariance");ok &= covariance(Sr,"GTSAM covariance");
    const Eigen::Matrix3d R=block(f,"dR_fastlio",3,3),Rr=block(g,"dR_gtsam",3,3);
    if((R.transpose()*R-Eigen::Matrix3d::Identity()).norm()>1e-8||std::abs(R.determinant()-1)>1e-8||
       (Rr.transpose()*Rr-Eigen::Matrix3d::Identity()).norm()>1e-8||std::abs(Rr.determinant()-1)>1e-8)
      throw std::runtime_error("Invalid SO3 mean");
    std::cout<<"Mean differences (informational, not matrix PASS): angle="<<Eigen::AngleAxisd(R.transpose()*Rr).angle()
       <<" dp_norm="<<(block(f,"dP_fastlio",3,1)-block(g,"dP_gtsam",3,1)).norm()
       <<" dv_norm="<<(block(f,"dV_fastlio",3,1)-block(g,"dV_gtsam",3,1)).norm()<<"\n";
    const double dt=block(f,"DT_fastlio",1,1)(0,0),dtr=block(g,"DT_gtsam",1,1)(0,0);
    if(dt<=0||dtr<=0)throw std::runtime_error("Nonpositive interval");
    ok &= compare(block(f,"DT_fastlio",1,1),block(g,"DT_gtsam",1,1),1e-12,0,"same DT");
    std::cout<<"Overall: "<<(ok?"PASS":"FAIL")<<"\n";return ok?0:1;
  }catch(const std::exception &e){std::cerr<<"[FAIL] compare_fastlio_gtsam: "<<e.what()<<"\n";return 1;}
}
