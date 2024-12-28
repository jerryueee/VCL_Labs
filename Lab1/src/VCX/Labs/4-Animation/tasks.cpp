#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            ik.JointGlobalRotation[i] = ik.JointGlobalRotation[i - 1] * ik.JointLocalRotation[i];
            ik.JointGlobalPosition[i] = ik.JointGlobalPosition[i - 1] + ik.JointGlobalRotation[i - 1] * ik.JointLocalOffset[i];
        }
        return ;
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        //int ret = 0;
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            for (int i = ik.NumJoints() - 2; i >= 0; i--){
                glm::vec3 a = glm::normalize(EndPosition - ik.JointGlobalPosition[i]);
                glm::vec3 b = glm::normalize(ik.EndEffectorPosition() - ik.JointGlobalPosition[i]);
                glm::quat rotation = glm::rotation(b, a);
                ik.JointLocalRotation[i] = rotation * ik.JointLocalRotation[i];
                ForwardKinematics(ik, i);
            }
            //if (glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps) ret = CCDIKIteration;
        }
        //std::cout << "CCDIK" << ret << std::endl;
        return ;
    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        //int ret = 0;
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                float d = ik.JointOffsetLength[i + 1];
                glm::vec3 dir = glm::normalize(ik.JointGlobalPosition[i] - next_position);
                backward_positions[i] = next_position + d * dir;
                next_position = backward_positions[i];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                float d = ik.JointOffsetLength[i + 1];
                glm::vec3 dir = glm::normalize(backward_positions[i + 1] - now_position);
                forward_positions[i + 1] = now_position + d * dir;
                now_position = forward_positions[i + 1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
            //if (glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps) ret = IKIteration;
        }
        //std::cout<<ret<<std::endl;
        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
        return ;
    }

    //IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
    //    // get function from https://www.wolframalpha.com/input/?i=Albert+Einstein+curve
    //    int nums = 5000;
    //    using Vec3Arr = std::vector<glm::vec3>;
    //    std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
    //    int index = 0;
    //    for (int i = 0; i < nums; i++) {
    //        float x_val = 1.5e-3f * custom_x(92 * glm::pi<float>() * i / nums);
    //        float y_val = 1.5e-3f * custom_y(92 * glm::pi<float>() * i / nums);
    //        if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
    //        (*custom)[index++] = glm::vec3(1.6f - x_val, 0.0f, y_val - 0.2f);
    //    }
    //    custom->resize(index);
    //    return custom;
    //}

    float get_x(float t){
        return 0.8 * glm::cos(t) / (1 + glm::pow(glm::sin(t), 2));
    }

    float get_y(float t){
        return 0.8 * (glm::cos(t) * glm::sin(t)) / (1 + glm::pow(glm::sin(t), 2));
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition(){
        int nums = 5000;
        using Vec3Arr = std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums + 10));
        int index = 0;
        for(int i = 0; i < nums; i++){
            float x_val = get_x(2 * glm::pi<float>() * (i - nums / 2) / nums);
            float y_val = get_y(2 * glm::pi<float>() * (i - nums / 2) / nums);
            if (std::abs(x_val) < 1e-3 || std::abs(y_val) < 1e-3) continue;
            (*custom)[index++] = glm::vec3(y_val, 0.0f, x_val);
        }
        custom->resize(index);
        return custom;
    }



    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    //void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
    //    // your code here: rewrite following code
    //    int const steps = 1000;
    //    float const ddt = dt / steps; 
    //    for (std::size_t s = 0; s < steps; s++) {
    //        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0));
    //        for (auto const spring : system.Springs) {
    //            auto const p0 = spring.AdjIdx.first;
    //            auto const p1 = spring.AdjIdx.second;
    //            glm::vec3 const x01 = system.Positions[p1] - system.Positions[p0];
    //            glm::vec3 const v01 = system.Velocities[p1] - system.Velocities[p0];
    //            glm::vec3 const e01 = glm::normalize(x01);
    //            glm::vec3 f = (system.Stiffness * (glm::length(x01) - spring.RestLength) + system.Damping * glm::dot(v01, e01)) * e01;
    //            forces[p0] += f;
    //            forces[p1] -= f;
    //        }
    //        for (std::size_t i = 0; i < system.Positions.size(); i++) {
    //            if (system.Fixed[i]) continue;
    //            system.Velocities[i] += (glm::vec3(0, -system.Gravity, 0) + forces[i] / system.Mass) * ddt;
    //            system.Positions[i] += system.Velocities[i] * ddt;
    //        }
    //    }
    //}

    void AdvanceMassSpringSystem(MassSpringSystem& system,float const dt){
        int n = system.Positions.size();
        //compute_M();
        std::vector<Eigen::Triplet<float>> triplets;
        float m = system.Mass;
        for (int i = 0; i < n; i++){
            triplets.emplace_back(3 * i, 3 * i, m);
            triplets.emplace_back(3 * i + 1, 3 * i + 1, m);
            triplets.emplace_back(3 * i + 2, 3 * i + 2, m);
        }
        auto M = CreateEigenSparseMatrix(3 * n, triplets);
        
        //compute_f()
        std::vector<glm::vec3> forces(system.Positions.size(), glm::vec3(0.0f, -system.Gravity, 0.0f));
        for(int i = 0; i < forces.size(); i++) {
            if (!system.Fixed[i]) continue;
            forces[i] = glm::vec3(0);
        }
        for (auto const& spring : system.Springs){
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;
            glm::vec3 const xi = system.Positions[p0], xj = system.Positions[p1];
            glm::vec3 const xij = xi - xj;
            glm::vec3 f = -system.Stiffness * (glm::length(xij) - spring.RestLength) * xij / glm::length(xij);
            forces[p0] += f;
            forces[p1] -= f;
        }

        //compute_Jacobian();
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jacobian = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(3 * n, 3 * n);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> Jacobian_d = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Zero(3 * n, 3 * n);
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        for (auto const& spring : system.Springs){
            if (spring.RestLength == 0) continue;
            auto const p0 = spring.AdjIdx.first;
            auto const p1 = spring.AdjIdx.second;

            if (system.Fixed[p0] && system.Fixed[p1]) continue;

            glm::vec3 const xi = system.Positions[p0], xj = system.Positions[p1];
            glm::vec3 const x_ij = xi - xj;
            Eigen::Vector3f xij(x_ij[0], x_ij[1], x_ij[2]);
            xij = xij / glm::length(x_ij);
            Eigen::Matrix3f jx = -system.Stiffness * ((1.0f - spring.RestLength / glm::length(x_ij)) * (I - xij * xij.transpose()) + xij * xij.transpose());
            if (!system.Fixed[p0]) Jacobian.block<3,3>(3 * p0,3 * p0) += jx;
            if (!system.Fixed[p1]) Jacobian.block<3,3>(3 * p1,3 * p1) += jx;
            if (!system.Fixed[p0] && !system.Fixed[p1])Jacobian.block<3,3>(3 * p0,3 * p1) -= jx, Jacobian.block<3, 3>(3 * p1, 3 * p0) -= jx;

            Eigen::Matrix3f jd = -system.Damping * xij * xij.transpose();
            if (!system.Fixed[p0]) Jacobian_d.block<3, 3>(3 * p0, 3 * p0) += jd;
            if (!system.Fixed[p1]) Jacobian_d.block<3, 3>(3 * p1, 3 * p1) += jd;
            if (!system.Fixed[p0] && !system.Fixed[p1]) Jacobian_d.block<3, 3>(3 * p0, 3 * p1) -= jd, Jacobian_d.block<3, 3>(3 * p1, 3 * p0) -= jd;
        }


        //compute_A();
        Eigen::SparseMatrix<float> A = M - dt * dt * Jacobian - dt * Jacobian_d;
        
        //compute_b();
        Eigen::VectorXf b = dt * (glm2eigen(forces) + dt * Jacobian * glm2eigen(system.Velocities));
        

        //solve_deltav();
        auto delta_v = ComputeSimplicialLLT(A, b);
        
        //updateXandV();
        auto v = eigen2glm(delta_v);
        for (int i = 0; i < n; i++){

            if(system.Fixed[i]) continue;
			system.Velocities[i] += v[i];
			system.Positions[i] += system.Velocities[i] * dt;
		}
        return ;
    }
}




















