#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices. 按照公式更新原有每个顶点
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                int n = neighbors.size();
                float u = (n == 3 ? 3.0f/16.0f : 3.0f/(8.0f * n));
                glm::vec3 sum(0.0f);
                for(auto neighbor : neighbors){
                    sum += prev_mesh.Positions[neighbor];
                }
                //curr_mesh.Positions[i] = (1 - n * u) * prev_mesh.Positions[i] + u * sum;
                curr_mesh.Positions.push_back((1 - n * u) * prev_mesh.Positions[i] + u * sum);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);//传入nullptr是为了在不存在时返回nullptr
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 v1 = prev_mesh.Positions[e->From()], v2 = prev_mesh.Positions[e->To()];
                    glm::vec3 ep = (v1 + v2) / 2.0f;
                    curr_mesh.Positions.push_back(ep);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 v0 = prev_mesh.Positions[e->From()], v2 = prev_mesh.Positions[e->To()];
                    glm::vec3 v1 = prev_mesh.Positions[e->OppositeVertex()], v3 = prev_mesh.Positions[e->TwinOppositeVertex()];
                    glm::vec3 ep  = (3.0f/8.0f) * (v0 + v2) + (1.0f/8.0f) * (v1 + v3);
                    curr_mesh.Positions.push_back(ep);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    v0,m2,m1,
                    v1,m0,m2,
                    v2,m1,m0,
                    m0,m1,m2,
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        std::vector<int>Boundary_index;
        int BoundaryPoints = 0;
        //先获取边界上的点
        for(int i = 0; i < output.Positions.size(); i++) {
            DCEL::VertexProxy const * v = G.Vertex(i);
            if (v->OnBoundary()) {
                Boundary_index.push_back(i);
                //std::cout<<output.Positions[i].x<<' '<<output.Positions[i].y<<" "<<output.Positions[i].z<< std::endl;
                BoundaryPoints++;
            }
        }
        //初始化边界点，选择的是圆形边界
        for(int i = 0; i < BoundaryPoints; i++){
            glm::vec2 tmp{output.Positions[Boundary_index[i]].x, output.Positions[Boundary_index[i]].y};
            tmp /= glm::length(tmp);
            //映射到[0,1]区间
            tmp += 1.0f;
            tmp /= 2.0f;
            output.TexCoords[Boundary_index[i]] = tmp;
        }
        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for(int interior = 0; interior < output.Positions.size(); interior++){
                DCEL::VertexProxy const * tmp = G.Vertex(interior);
                if (tmp->OnBoundary()) continue;//只访问内部点
                glm::vec2 sum{0.0f, 0.0f};
                int n = 0;
                for(auto neighbor : tmp->Neighbors()){
                    sum += output.TexCoords[neighbor];
                    n++;
                }
                if(n) {
                    output.TexCoords[interior] = sum / (float)n;
                }
            }
        }
        return ;
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp(0.0f);
                // your code here:
                glm::vec3 p0 = output.Positions[f->VertexIndex(0)];
                glm::vec3 p1 = output.Positions[f->VertexIndex(1)];
                glm::vec3 p2 = output.Positions[f->VertexIndex(2)];
                glm::vec3 n = glm::cross((p0 - p1), (p2 - p1));
                if (glm::length(n) == 0) return Kp;
                n = glm::normalize(n);
                float a = n.x, b = n.y, c = n.z, d = -glm::dot(p0, n);
                //std::cout<<p0.x<<' '<<p0.y<<" "<<p0.z<<"" << p1.x << " " << p1.y << " " << p1.z << "d2d" << p2.x << " " << p2.y << " " << p2.z << "a" << a << 'b' << b << "c" << c << 'd' << d << std::endl;
                Kp[0][0] = a * a,Kp[0][1] = a * b,Kp[0][2] = a * c,Kp[0][3] = a * d;
                Kp[1][0] = a * b,Kp[1][1] = b * b,Kp[1][2] = b * c,Kp[1][3] = b * d;
                Kp[2][0] = a * c,Kp[2][1] = b * c,Kp[2][2] = c * c,Kp[2][3] = c * d;
                Kp[3][0] = a * d,Kp[3][1] = b * d,Kp[3][2] = c * d,Kp[3][3] = d * d;
                return Kp;
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;            // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ContractionPair {
                // your code here:
                ContractionPair ret = ContractionPair();
                ret.edge = edge;
                glm::vec4 v;
                //根据Q构造出所需的矩阵，注意行列的顺序
                glm::mat4 q = {
                    Q[0][0],Q[1][0],Q[2][0],0.0f,
                    Q[0][1],Q[1][1],Q[2][1],0.0f,
                    Q[0][2],Q[1][2],Q[2][2],0.0f,
                    Q[0][3],Q[1][3],Q[2][3],1.0f,
                };
                if(glm::abs(glm::determinant(q)) > 0.001f){
                    //std::cout<<glm::determinant(Q)<<std::endl;
                    
                    //Q可逆
                    v = glm::inverse(q) * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                    //std::cout<<v.x<<" "<<v.y<<" "<<v.z<< std::endl;
                    ret.targetPosition = v;
                    ret.cost = glm::dot(v,Q * v);
                }
                else {
                    //不可逆就设置成中点,p1和p2中Q值最小的点
                    glm::vec4 middle((p1,p2) * 0.5f,1.0f);
                    glm::vec4 pp1 (p1,1.0f);
                    glm::vec4 pp2 (p2,1.0f);
                    float q1 = glm::dot(pp1, Q * pp1), q2 = glm::dot(pp2, Q * pp2), q3 = glm::dot(middle, Q * middle);
                    if(q1 <= q2 && q1 <= q3) {
                        ret.targetPosition = pp1;
                        ret.cost = q1;
                    }
                    else if (q2 <= q3 && q2 <= q1){
                        ret.targetPosition = pp2;
                        ret.cost = q2;
                    }
                    else if(q3 <= q1 && q3 <= q2){
                        ret.targetPosition = middle;
                        ret.cost = q3;
                    }
                }
                /*v = glm::vec4((p1 + p2) * 0.5f,1.0f);*/
                return ret;
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ContractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)]       = Q;
        }
        //避免重新分配内存带来的问题，更改容量
        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);//类似于push_back()，但是省去了创建临时对象，更快
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;// 相当于把min_idx设成最大的那个值
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;//已经坍缩过的边
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;// 此时不一定还能继续坍缩，需要判断
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Contract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.更新与v1相关的pair的信息
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0.0f);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).
                //     4. Update $Kf$.
                auto Kp_new = UpdateQ(e->Face());
                auto Kp_old = Kf[G.IndexOf(e->Face())];
                auto delta = Kp_new - Kp_old;
                Qv[e->To()] += delta;
                Qv[e->From()] += delta;
                Qv[v1] += Kp_new;
                Kf[G.IndexOf(e->Face())] = Kp_new;
            }
            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            for(auto e : ring){
                if (e == nullptr) continue;
                int index_v1 = e->From(), index_v2 = e->To(), index_e = G.IndexOf(e);
                if (G.IsVertexRemoved(index_v1)||G.IsVertexRemoved(index_v2)) continue;
                /*if (!G.IsContractable(e)) continue;*/
                auto Q = Qv[index_v1] + Qv[index_v2];
                pairs[pair_map[index_e]] = MakePair(e, output.Positions[index_v1], output.Positions[index_v2], Q);
            }
        }
        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
        return ;
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {// static constexpr auto:声明一个静态的常量表达式
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here
                glm::vec3 a = v1 - vAngle;
                glm::vec3 b = v2 - vAngle;
                float cos = glm::dot(a, b);
                float sin = glm::length(glm::cross(a, b));
                //if (sin < 1e-6f) return 0.0f;
                float cot = cos / (sin + 1e-6);
                cot = glm::clamp(cot, 0.0f, 1.0f);
                return cot;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                DCEL::VertexProxy const * cur_point = G.Vertex(i);
                auto neighbors = cur_point->Neighbors();
                int len = neighbors.size();
                float sum = 0.0f;
                glm::vec3 point(0.0f);
                for(int j = 0; j < len; j++){
                    int tmp = neighbors[j];
                    float w = 1.0f;
                    glm::vec3 vj = prev_mesh.Positions[tmp];
                    if (!useUniformWeight){
                        glm::vec3 vi = prev_mesh.Positions[i];
                        int k1 = (j + 1 + len) % len, k2 = (j - 1 + len) % len;
                        glm::vec3 vk1 = prev_mesh.Positions[neighbors[k1]];
                        glm::vec3 vk2 = prev_mesh.Positions[neighbors[k2]];
                        w = GetCotangent(vk1, vi, vj) + GetCotangent(vk2, vi, vj);
                    }
                    sum += w;
                    point += w * vj;
                }
                if(sum > 1e-3f) {
                    point /= sum;
                    curr_mesh.Positions[i] = (1 - lambda) * prev_mesh.Positions[i] + lambda * point;
                }
                else curr_mesh.Positions[i] = prev_mesh.Positions[i];
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
        return;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
        //std::cout<<output.Indices.size()<<std::endl;
        glm::vec3 unit[3] = {
            {1.0f,0.0f,0.0f},
            {0.0f,1.0f,0.0f},
            {0.0f,0.0f,1.0f},
        };
        int num = 0;
        for(int x = 0; x <= n; x++){
            for(int y = 0; y <= n; y++) {
                for(int z = 0; z <= n; z++) {
                    glm::vec3 v0 = grid_min + glm::vec3(x * dx, y * dx, z * dx);
                    //计算属于哪一种类型的cube
                    int vertex_status = 0;
                    float sdf_val[8] = {};
                    for(int i = 0; i < 8; i++) {
                        glm::vec3 pos = v0 + glm::vec3((i & 1) * dx, ((i >> 1) & 1) * dx, (i >> 2) * dx);
                        sdf_val[i] = sdf(pos);
                        if (sdf_val[i] >= 0) vertex_status += (1 << i);
                    }
                    int edge_status = c_EdgeStateTable[vertex_status];
                    //std::cout<<edge_status<<" ";
                    if (edge_status == 0) continue;// 如果每一条边都没有mesh的顶点，不用再继续操作
                    //查找边上的Mesh顶点
                    glm::vec3 mesh_vertex[12] = {};//j条边上的顶点坐标
                    for(int j = 0; j < 12; j++){
                        if ((edge_status & (1 << j)) == 0) continue;
                        glm::vec3 v1 = v0 + (dx * (j & 1) * unit[((j >> 2) + 1) % 3] + dx * ((j >> 1) & 1) * unit[((j >> 2) + 2) % 3]);//根据公式算出第j条边的起始点
                        glm::vec3 v2 = v1 + dx * unit[(j >> 2)]; // 一定要乘dx
                        //std::cout<<"first "<<v1.x << " " << v1.y << " " << v1.z << " second " << v2.x << ' ' << v2.y << ' ' << v2.z << std::endl;
                        mesh_vertex[j] = v1 + (0 - sdf(v1)) * (v2 - v1) / (sdf(v2) - sdf(v1));//上面已经判断过，不存在除0的情况
                        //mesh_vertex[j] = ((0 - sdf(v1)) * v2 + (sdf(v2) - 0) * v1) / (sdf(v2) - sdf(v1) + 0.000001f);
                        //std::cout<< mesh_vertex[j].x - v0.x<<" "<<j<<" ";
                    }
                    //std::cout<<vertex_status<<" ";
                    //std::cout<<std::endl;
                    for(int k = 0; c_EdgeOrdsTable[vertex_status][3 * k] != -1; k++){
                        uint32_t e0 = c_EdgeOrdsTable[vertex_status][3 * k];
                        uint32_t e1 = c_EdgeOrdsTable[vertex_status][3 * k + 1];
                        uint32_t e2 = c_EdgeOrdsTable[vertex_status][3 * k + 2];
                        output.Positions.push_back(mesh_vertex[e2]);
                        output.Indices.push_back(num++);
                        output.Positions.push_back(mesh_vertex[e1]);
                        output.Indices.push_back(num++);
                        output.Positions.push_back(mesh_vertex[e0]);
                        output.Indices.push_back(num++);
                    }
                }
            }
        }
        //std::cout<<output.Indices.size()<<std::endl;
        return ;
    }
} // namespace VCX::Labs::GeometryProcessing
