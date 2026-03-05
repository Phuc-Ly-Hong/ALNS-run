#include "LargeNeighborhoodSearch.h"
#include <iostream>
#include <limits>
#include <cmath>
#include <algorithm>
#include <random>
#include <chrono> // Thêm thư viện thời gian
using Random = effolkronium::random_static;
using namespace std::chrono;

LargeNeighborhoodSearch::LargeNeighborhoodSearch(Config &conf, Input &inp) {
    this->config = conf;
    this->input = inp;
    // Tăng số vòng lặp nếu cấu hình quá thấp
    this->maxIterations = std::max(conf.tabuMaxIter, 200); 

    int numCustomers = inp.numCus;
    // Điều chỉnh tỷ lệ phá hủy: từ 15% đến 40% số lượng khách
    this->minDestroy = std::max(2, (int)(numCustomers * 0.15));
    this->maxDestroy = std::max(3, (int)(numCustomers * 0.40));
}

std::tuple<double, Solution, std::vector<std::vector<int>>, std::vector<std::vector<int>>> LargeNeighborhoodSearch::run(json &log, std::string &path_e, Input &input, Solution solution) {
    this->input = input;
    solution.setInput(input);

    Solution bestSol = solution;
    double bestScore = solution.getScore();
    Solution currentSol = solution;

    // Khởi tạo 2 ma trận tần suất
    // MatrixCurrent: Dùng cho Method C
    // MatrixBest: Dùng cho Method A
    std::vector<std::vector<int>> edgeMatrixCurrent(input.numCus + 2, std::vector<int>(input.numCus + 2, 0));
    std::vector<std::vector<int>> edgeMatrixBest(input.numCus + 2, std::vector<int>(input.numCus + 2, 0));

    // Cập nhật trạng thái ban đầu
    updateEdgeFrequencyMatrix(currentSol, edgeMatrixCurrent);
    updateEdgeFrequencyMatrix(bestSol, edgeMatrixBest);

    for (int iter = 0; iter < maxIterations; ++iter) {
        int destroyCount = Random::get(minDestroy, maxDestroy);
        if (destroyCount >= input.numCus) destroyCount = std::max(1, input.numCus / 2);

        auto destroyRes = destroyRandom(currentSol, destroyCount);
        Solution destroyedSol = destroyRes.first;
        std::vector<int> removedCustomers = destroyRes.second;

        Solution repairedSol = repairGreedy(destroyedSol, removedCustomers);

        if (repairedSol.check_feasible()) {
            double newScore = repairedSol.getScore();
            double currentScore = currentSol.getScore();
            double delta = newScore - currentScore;

            // Chấp nhận nghiệm tốt hơn
            if (delta < 0) {
                currentSol = repairedSol;
                if (newScore < bestScore - 1e-5) {
                    bestScore = newScore;
                    bestSol = repairedSol;
                    // Cập nhật tần suất cho Best Solution (Method A)
                    updateEdgeFrequencyMatrix(bestSol, edgeMatrixBest);
                }
            } 
        }

        // Cập nhật tần suất cho Current Solution (Method C) sau mỗi vòng lặp
        updateEdgeFrequencyMatrix(currentSol, edgeMatrixCurrent);
    }

    bestSol.refactorSolution();
    return std::make_tuple(bestScore, bestSol, edgeMatrixCurrent, edgeMatrixBest);
}

std::tuple<double, Solution> LargeNeighborhoodSearch::runWithTimeLimit(Input &input, Solution solution, double timeLimitSeconds) {
    this->input = input;
    solution.setInput(input);

    Solution bestSol = solution;
    double bestScore = solution.getScore();
    Solution currentSol = solution;
    
    auto startTime = high_resolution_clock::now();

    // Vòng lặp vô hạn, chỉ dừng khi hết giờ
    long long iter = 0;
    while (true) {
        // Kiểm tra thời gian
        auto now = high_resolution_clock::now();
        double elapsed = duration_cast<milliseconds>(now - startTime).count() / 1000.0;
        if (elapsed >= timeLimitSeconds) break;

        iter++;

        // 1. Destroy
        int destroyCount = Random::get(minDestroy, maxDestroy);
        if (destroyCount >= input.numCus) destroyCount = input.numCus - 1;
        if (destroyCount < 1) destroyCount = 1;

        auto destroyRes = destroyRandom(currentSol, destroyCount);
        Solution destroyedSol = destroyRes.first;
        std::vector<int> removedCustomers = destroyRes.second;

        // 2. Repair
        Solution repairedSol = repairGreedy(destroyedSol, removedCustomers);

        // 3. Acceptance 
        if (repairedSol.check_feasible()) {
            double newScore = repairedSol.getScore();
            double currentScore = currentSol.getScore();
            double delta = newScore - currentScore;

            if (delta < 0) {
                currentSol = repairedSol;
                if (newScore < bestScore - 1e-5) {
                    bestScore = newScore;
                    bestSol = repairedSol;
                }
            }
        }

    }

    bestSol.refactorSolution();
    return std::make_tuple(bestScore, bestSol);
}

std::pair<Solution, std::vector<int>> LargeNeighborhoodSearch::destroyRandom(Solution sol, int numToRemove) {
    std::vector<int> removed;
    int count = 0;
    int attempts = 0;

    // Clone danh sách khách hàng để chọn ngẫu nhiên không trùng lặp
    std::vector<int> currentNodes;
    for(auto& d : sol.droneTripList) for(auto& t : d) for(int n : t) currentNodes.push_back(n);
    for(auto& t : sol.techTripList) for(int n : t) currentNodes.push_back(n);

    if (currentNodes.empty()) return {sol, removed};

    // Xáo trộn để xóa ngẫu nhiên
    std::shuffle(currentNodes.begin(), currentNodes.end(), Random::get_engine());

    for(int nodeToRemove : currentNodes) {
        if (count >= numToRemove) break;

        // Tìm và xóa node trong Drone
        bool found = false;
        for (auto& drone : sol.droneTripList) {
            for (auto& trip : drone) {
                auto it = std::find(trip.begin(), trip.end(), nodeToRemove);
                if (it != trip.end()) {
                    trip.erase(it);
                    removed.push_back(nodeToRemove);
                    count++;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
        if (found) continue;

        // Tìm và xóa node trong Tech
        for (auto& trip : sol.techTripList) {
            auto it = std::find(trip.begin(), trip.end(), nodeToRemove);
            if (it != trip.end()) {
                trip.erase(it);
                removed.push_back(nodeToRemove);
                count++;
                break;
            }
        }
    }
    
    return {sol, removed};
}

Solution LargeNeighborhoodSearch::repairGreedy(Solution sol, std::vector<int>& removedCustomers) {
    // Greedy Insertion: Với mỗi khách hàng bị xóa, thử chèn vào TẤT CẢ vị trí có thể
    // và chọn vị trí làm tăng Makespan ít nhất (Best Fit).
    
    // Xáo trộn thứ tự chèn để tạo sự đa dạng
    std::shuffle(removedCustomers.begin(), removedCustomers.end(), Random::get_engine());

    for (int customer : removedCustomers) {
        double bestIncrease = std::numeric_limits<double>::max();
        int bestType = -1; // 0: Drone, 1: Tech
        int bestVehicleIdx = -1;
        int bestTripIdx = -1;
        int bestPos = -1;
        
        double currentMakespan = sol.getScore();

        bool onlyTech = false;
        if (customer < input.cusOnlyServedByTech.size()) {
            onlyTech = input.cusOnlyServedByTech[customer];
        }

        // --- 1. Thử chèn vào DRONE ---
        if (!onlyTech) {
            for (int d = 0; d < sol.droneTripList.size(); ++d) {
                // Thử chèn vào các trip hiện có
                for (int t = 0; t < sol.droneTripList[d].size(); ++t) {
                    for (int p = 0; p <= sol.droneTripList[d][t].size(); ++p) {
                        // Insert thử
                        sol.droneTripList[d][t].insert(sol.droneTripList[d][t].begin() + p, customer);
                        
                        if (sol.check_feasible()) {
                            double newMakespan = sol.getScore();
                            double increase = newMakespan - currentMakespan;
                            
                            // Ưu tiên: Tăng makespan ít nhất -> Nếu bằng nhau thì chọn cái có tổng quãng đường ngắn hơn (phụ)
                            if (increase < bestIncrease) {
                                bestIncrease = increase;
                                bestType = 0;
                                bestVehicleIdx = d;
                                bestTripIdx = t;
                                bestPos = p;
                            }
                        }
                        // Revert
                        sol.droneTripList[d][t].erase(sol.droneTripList[d][t].begin() + p);
                    }
                }
                
                // Thử tạo Trip MỚI cho Drone (nếu được phép thêm trip rỗng)
                // (Logic này tùy thuộc vào việc droneTripList có fix cứng số trip hay không. 
                // Code hiện tại của bạn có vẻ fix cứng số trip nên ta bỏ qua việc tự push_back trip mới)
            }
        }

        // --- 2. Thử chèn vào TECH ---
        for (int k = 0; k < sol.techTripList.size(); ++k) {
            for (int p = 0; p <= sol.techTripList[k].size(); ++p) {
                sol.techTripList[k].insert(sol.techTripList[k].begin() + p, customer);

                if (sol.check_feasible()) {
                    double newMakespan = sol.getScore();
                    double increase = newMakespan - currentMakespan;

                    if (increase < bestIncrease) {
                        bestIncrease = increase;
                        bestType = 1;
                        bestVehicleIdx = k;
                        bestPos = p;
                    }
                }
                // Revert
                sol.techTripList[k].erase(sol.techTripList[k].begin() + p);
            }
        }

        // --- 3. CHÈN THẬT ---
        if (bestType == 0) {
            sol.droneTripList[bestVehicleIdx][bestTripIdx].insert(
                sol.droneTripList[bestVehicleIdx][bestTripIdx].begin() + bestPos, customer);
        } else if (bestType == 1) {
            sol.techTripList[bestVehicleIdx].insert(
                sol.techTripList[bestVehicleIdx].begin() + bestPos, customer);
        } else {
            // Cứu cánh: Nếu không tìm được chỗ chèn feasible, chèn đại vào Tech đầu tiên (chấp nhận phạt)
            // Để thuật toán không bị crash/mất khách
            if (!sol.techTripList.empty()) sol.techTripList[0].push_back(customer);
        }
    }
    return sol;
}

void LargeNeighborhoodSearch::updateEdgeFrequencyMatrix(const Solution& sol, std::vector<std::vector<int>>& matrix) {
    int size = matrix.size();
    for (const auto& drone : sol.droneTripList) {
        for (const auto& trip : drone) {
            if (trip.empty()) continue;
            if (trip[0] < size) matrix[0][trip[0]]++; 
            for (size_t i = 0; i < trip.size() - 1; ++i) {
                int u = trip[i]; int v = trip[i+1];
                if (u < size && v < size) matrix[u][v]++;
            }
        }
    }
    for (const auto& trip : sol.techTripList) {
        if (trip.empty()) continue;
        if (trip[0] < size) matrix[0][trip[0]]++;
        for (size_t i = 0; i < trip.size() - 1; ++i) {
            int u = trip[i]; int v = trip[i+1];
            if (u < size && v < size) matrix[u][v]++;
        }
    }
}