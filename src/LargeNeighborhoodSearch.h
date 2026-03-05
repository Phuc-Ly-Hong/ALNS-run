#ifndef DASTS2_VERSION9_C_LARGENEIGHBORHOODSEARCH_H
#define DASTS2_VERSION9_C_LARGENEIGHBORHOODSEARCH_H

#include "Config.h"
#include "Input.h"
#include "Solution.h"
#include "Random.h" // Thư viện random (effolkronium)
#include <vector>
#include <tuple>
#include <algorithm>
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class LargeNeighborhoodSearch {
public:
    Config config;
    Input input;

    // Constructor
    LargeNeighborhoodSearch(Config &conf, Input &inp);
    
    // Cập nhật kiểu trả về: <BestScore, BestSol, FreqMatrixCurrent, FreqMatrixBest>
    std::tuple<double, Solution, std::vector<std::vector<int>>, std::vector<std::vector<int>>> run(json &log, std::string &path_e, Input &input, Solution solution);
    // --- HÀM MỚI: Chạy theo giới hạn thời gian ---
    std::tuple<double, Solution> runWithTimeLimit(Input &input, Solution solution, double timeLimitSeconds);

private:
    int maxIterations;
    int minDestroy;
    int maxDestroy;

    // Destroy: Xóa ngẫu nhiên khách hàng
    std::pair<Solution, std::vector<int>> destroyRandom(Solution sol, int numToRemove);

    // Repair: Chèn lại tham lam (Greedy Insertion)
    Solution repairGreedy(Solution sol, std::vector<int>& removedCustomers);

    // Helper: Cập nhật ma trận tần suất cạnh cho MultiLevel
    void updateEdgeFrequencyMatrix(const Solution& sol, std::vector<std::vector<int>>& matrix);
};

#endif