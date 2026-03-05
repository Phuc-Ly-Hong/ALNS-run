#include "LargeNeighborhoodSearch.h"
#include <iostream>
#include <limits>
#include <cmath>

using Random = effolkronium::random_static;

LargeNeighborhoodSearch::LargeNeighborhoodSearch(Config &conf, Input &inp) {
    this->config = conf;
    this->input = inp;
    // Sử dụng số iter từ config (vốn dành cho Tabu)
    this->maxIterations = conf.tabuMaxIter; 
    if (this->maxIterations <= 0) this->maxIterations = 50;

    // Cấu hình số lượng khách hàng xóa (10% - 30%)
    int numCustomers = inp.numCus;
    this->minDestroy = std::max(1, (int)(numCustomers * 0.1));
    this->maxDestroy = std::max(2, (int)(numCustomers * 0.3));
}

std::tuple<double, Solution, std::vector<std::vector<int>>> LargeNeighborhoodSearch::run(json &log, std::string &path_e, Input &input, Solution solution) {
    // Cập nhật input mới nhất (vì MultiLevel thay đổi input qua từng tầng)
    this->input = input;
    solution.setInput(input);

    Solution bestSol = solution;
    double bestScore = solution.getScore();
    Solution currentSol = solution;

    // Ma trận tần suất cạnh (Edge Frequency Matrix) dùng cho việc "Matching" trong MultiLevel
    std::vector<std::vector<int>> edgeMatrix(input.numCus + 2, std::vector<int>(input.numCus + 2, 0));

    // Vòng lặp LNS
    for (int iter = 0; iter < maxIterations; ++iter) {
        // 1. Determine destroy size
        int destroyCount = Random::get(minDestroy, maxDestroy);
        if (destroyCount > input.numCus) destroyCount = std::max(1, input.numCus / 2);

        // 2. Destroy
        auto destroyRes = destroyRandom(currentSol, destroyCount);
        Solution destroyedSol = destroyRes.first;
        std::vector<int> removedCustomers = destroyRes.second;

        // 3. Repair
        Solution repairedSol = repairGreedy(destroyedSol, removedCustomers);

        // 4. Acceptance (Hill Climbing cải tiến: Chấp nhận nếu tốt hơn Best hoặc tốt hơn Current)
        double newScore = repairedSol.getScore();
        double currentScore = currentSol.getScore();

        if (repairedSol.check_feasible()) {
            // Nếu tìm được kỷ lục mới
            if (newScore < bestScore - 1e-5) {
                bestScore = newScore;
                bestSol = repairedSol;
                currentSol = repairedSol;
            }
            // Nếu tốt hơn hiện tại
            else if (newScore < currentScore - 1e-5) {
                currentSol = repairedSol;
            }
            // Cơ chế thoát: Đôi khi chấp nhận giải pháp tồi hơn nhẹ (Simulated Annealing đơn giản)
            // Ở đây giữ Hill Climbing để đảm bảo tốc độ cho MultiLevel
        }

        // 5. Cập nhật Edge Matrix (quan trọng cho bước Coarsening)
        updateEdgeFrequencyMatrix(currentSol, edgeMatrix);
    }

    bestSol.refactorSolution(); // Loại bỏ các trip rỗng
    return std::make_tuple(bestScore, bestSol, edgeMatrix);
}

std::pair<Solution, std::vector<int>> LargeNeighborhoodSearch::destroyRandom(Solution sol, int numToRemove) {
    std::vector<int> removed;
    int count = 0;
    int attempts = 0;

    while (count < numToRemove && attempts < 500) {
        attempts++;
        int type = Random::get(0, 1); // 0: Drone, 1: Tech

        if (type == 0 && !sol.droneTripList.empty()) {
            int dIdx = Random::get(0, (int)sol.droneTripList.size() - 1);
            if (sol.droneTripList[dIdx].empty()) continue;
            
            int tIdx = Random::get(0, (int)sol.droneTripList[dIdx].size() - 1);
            if (sol.droneTripList[dIdx][tIdx].empty()) continue;

            int cIdx = Random::get(0, (int)sol.droneTripList[dIdx][tIdx].size() - 1);
            int customer = sol.droneTripList[dIdx][tIdx][cIdx];

            sol.droneTripList[dIdx][tIdx].erase(sol.droneTripList[dIdx][tIdx].begin() + cIdx);
            removed.push_back(customer);
            count++;
        } 
        else if (type == 1 && !sol.techTripList.empty()) {
            int tIdx = Random::get(0, (int)sol.techTripList.size() - 1);
            if (sol.techTripList[tIdx].empty()) continue;

            int cIdx = Random::get(0, (int)sol.techTripList[tIdx].size() - 1);
            int customer = sol.techTripList[tIdx][cIdx];

            sol.techTripList[tIdx].erase(sol.techTripList[tIdx].begin() + cIdx);
            removed.push_back(customer);
            count++;
        }
    }
    return {sol, removed};
}

Solution LargeNeighborhoodSearch::repairGreedy(Solution sol, std::vector<int>& removedCustomers) {
    // Xáo trộn danh sách để tránh thiên vị
    std::shuffle(removedCustomers.begin(), removedCustomers.end(), Random::get_engine());

    for (int customer : removedCustomers) {
        double bestCost = std::numeric_limits<double>::max();
        int bestType = -1; // 0: Drone, 1: Tech
        int bestVehicleIdx = -1;
        int bestTripIdx = -1;
        int bestPos = -1;

        bool onlyTech = false;
        if (customer < input.cusOnlyServedByTech.size()) {
            onlyTech = input.cusOnlyServedByTech[customer];
        }

        // --- Thử chèn vào Drone ---
        if (!onlyTech) {
            for (int d = 0; d < sol.droneTripList.size(); ++d) {
                for (int t = 0; t < sol.droneTripList[d].size(); ++t) {
                    for (int p = 0; p <= sol.droneTripList[d][t].size(); ++p) {
                        sol.droneTripList[d][t].insert(sol.droneTripList[d][t].begin() + p, customer);
                        
                        if (sol.check_feasible()) {
                            double score = sol.getScore();
                            if (score < bestCost) {
                                bestCost = score;
                                bestType = 0;
                                bestVehicleIdx = d;
                                bestTripIdx = t;
                                bestPos = p;
                            }
                        }
                        // Backtrack
                        sol.droneTripList[d][t].erase(sol.droneTripList[d][t].begin() + p);
                    }
                }
            }
        }

        // --- Thử chèn vào Tech ---
        for (int k = 0; k < sol.techTripList.size(); ++k) {
            for (int p = 0; p <= sol.techTripList[k].size(); ++p) {
                sol.techTripList[k].insert(sol.techTripList[k].begin() + p, customer);

                if (sol.check_feasible()) {
                    double score = sol.getScore();
                    if (score < bestCost) {
                        bestCost = score;
                        bestType = 1;
                        bestVehicleIdx = k;
                        bestPos = p;
                    }
                }
                // Backtrack
                sol.techTripList[k].erase(sol.techTripList[k].begin() + p);
            }
        }

        // --- Chèn thật ---
        if (bestType == 0) {
            sol.droneTripList[bestVehicleIdx][bestTripIdx].insert(
                sol.droneTripList[bestVehicleIdx][bestTripIdx].begin() + bestPos, customer);
        } else if (bestType == 1) {
            sol.techTripList[bestVehicleIdx].insert(
                sol.techTripList[bestVehicleIdx].begin() + bestPos, customer);
        } else {
            // Nếu không chèn được vào đâu (infeasible), nhét tạm vào tech đầu tiên để không mất khách
            // (Trong thực tế nên xử lý tốt hơn, ví dụ tạo trip mới)
            if (!sol.techTripList.empty()) 
                sol.techTripList[0].push_back(customer);
        }
    }
    return sol;
}

void LargeNeighborhoodSearch::updateEdgeFrequencyMatrix(const Solution& sol, std::vector<std::vector<int>>& matrix) {
    int size = matrix.size();

    // Drone
    for (const auto& drone : sol.droneTripList) {
        for (const auto& trip : drone) {
            if (trip.empty()) continue;
            // 0 -> node đầu
            if (trip[0] < size) matrix[0][trip[0]]++;
            
            for (size_t i = 0; i < trip.size() - 1; ++i) {
                int u = trip[i];
                int v = trip[i+1];
                if (u < size && v < size) matrix[u][v]++;
            }
        }
    }

    // Tech
    for (const auto& trip : sol.techTripList) {
        if (trip.empty()) continue;
        if (trip[0] < size) matrix[0][trip[0]]++;

        for (size_t i = 0; i < trip.size() - 1; ++i) {
            int u = trip[i];
            int v = trip[i+1];
            if (u < size && v < size) matrix[u][v]++;
        }
    }
}