#!/usr/bin/env python3
"""Compile the production solve() body against a controlled solver-read boundary.

Inject NaN/Inf at every state/input slot with a successful mock acados status.
This checks acceptance and warm-start ownership, not acados numerical accuracy.
"""
from pathlib import Path
import os
import subprocess
import tempfile

PREAMBLE = r"""
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#define ROS_ERROR(...) ((void)0)
#define ROS_WARN_THROTTLE(...) ((void)0)
constexpr int UAV_NMPC_N=10;
template<size_t N> struct Values {
    std::array<double,N> data{};
    bool allFinite() const {for(double v:data)if(!std::isfinite(v))return false;return true;}
    const Values& array()const{return *this;}
    const Values& isFinite()const{return *this;}
    bool all()const{return allFinite();}
};
namespace Eigen {using Vector3d=Values<3>;}
namespace control {template<class T> bool isFinite(const T& v){return v.allFinite();}}
using Se3StateVector=Values<13>;using UavNmpcStateVector=Values<14>;
struct Se3Reference{};
struct ReadBoundary {int status=0;};
int uav_nmpc_acados_solve(ReadBoundary* b){return b->status;}
class UavNmpcSolver {
public:
    bool initialized_=true;ReadBoundary boundary;ReadBoundary* capsule_=&boundary;
    int solver_status_=0;double solve_time_ms_=0;
    std::array<Values<14>,11> x_solution_{};
    std::array<Values<4>,10> u_solution_{};
    int warm_commits=0,resets=0;
    bool initialize(){return true;}
    UavNmpcStateVector packInternalState(const Se3StateVector&,double){return {};}
    bool setInitialState(const UavNmpcStateVector&){return true;}
    bool setReference(int,const Se3Reference&,double,const Eigen::Vector3d&){return true;}
    void setGuesses(const UavNmpcStateVector&,const std::vector<Se3Reference>&){}
    void readSolution(){} // arrays below stand for the acados out_get return buffers
    void shiftWarmStart(const std::vector<Se3Reference>&){++warm_commits;}
    void resetWarmStart(){++resets;}
    bool solve(const Se3StateVector&,double,double,const Eigen::Vector3d&,const std::vector<Se3Reference>&);
};
"""

MAIN = r"""
void require(bool ok,const char* message){if(!ok)throw std::runtime_error(message);}
int main(){
    try {
        const std::vector<Se3Reference> refs(UAV_NMPC_N+2);
        auto solve=[&](UavNmpcSolver& s){return s.solve({},9.8,9.8,{},refs);};
        UavNmpcSolver normal;require(solve(normal),"finite nominal solution rejected");
        require(normal.warm_commits==1,"valid warm start not committed");
        int cases=0;
        for(double bad:{std::numeric_limits<double>::quiet_NaN(),
                        std::numeric_limits<double>::infinity(),
                        -std::numeric_limits<double>::infinity()}){
            for(size_t stage=0;stage<11;++stage)for(size_t slot=0;slot<14;++slot){
                UavNmpcSolver s;s.x_solution_[stage].data[slot]=bad;
                require(!solve(s),"non-finite state accepted with status=0");
                require(s.warm_commits==0&&s.resets==1,"bad state seeded warm start");
                require(s.solver_status_!=0,"candidate rejection reported as success");
                s.x_solution_[stage].data[slot]=0;
                require(solve(s),"finite recovery rejected");++cases;
            }
            for(size_t stage=0;stage<10;++stage)for(size_t slot=0;slot<4;++slot){
                UavNmpcSolver s;s.u_solution_[stage].data[slot]=bad;
                require(!solve(s),"non-finite input accepted with status=0");
                require(s.warm_commits==0&&s.resets==1,"bad input seeded warm start");++cases;
            }
        }
        UavNmpcSolver failed;failed.boundary.status=2;
        require(!solve(failed)&&failed.warm_commits==0,"failed solver accepted");
        std::cout<<"PASS: "<<cases<<" injected state/input faults rejected; finite recovery and status failure checked\n";
    } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
"""


def extract_solve(source):
    start = source.index("bool UavNmpcSolver::solve(")
    opening = source.index("{", start)
    depth = 1
    index = opening + 1
    while depth and index < len(source):
        depth += (source[index] == "{") - (source[index] == "}")
        index += 1
    if depth:
        raise ValueError("Unbalanced solve() definition")
    return source[start:index]


def main():
    package = Path(__file__).resolve().parents[1]
    source = (package / "src/nmpc/uav_nmpc_solver.cpp").read_text()
    with tempfile.TemporaryDirectory(prefix="uav-candidate-") as tmp:
        root = Path(tmp)
        (root / "test.cpp").write_text(PREAMBLE + extract_solve(source) + MAIN)
        subprocess.run([
            os.environ.get("CXX", "g++"), "-std=c++17", "-Wall", "-Wextra", "-Werror",
            "-pedantic", str(root / "test.cpp"), "-o", str(root / "test"),
        ], check=True, timeout=60)
        subprocess.run([str(root / "test")], check=True, timeout=10)


if __name__ == "__main__":
    main()
