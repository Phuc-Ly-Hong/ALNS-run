#ifndef DASTS2_VERSION9_C_MULTILEVEL_H
#define DASTS2_VERSION9_C_MULTILEVEL_H

#include "nlohmann/json.hpp"
#include "Config.h"
#include "Solution.h"
#include "Input.h"
#include "LargeNeighborhoodSearch.h" // Sử dụng LNS thay cho TabuSearch
#include <map>
#include <tuple>
#include <vector>

class MultiLevel
{
public:
    Config config;
    Input input;
    
    // Module tìm kiếm
    LargeNeighborhoodSearch lns; 

    Solution initSolution;
    Solution currentSolution;
    Solution bestSolution;

    // Constructor
    MultiLevel(Config &config, Input &input);

    int numCustomer{};
    int tabuDuration{};

    // Các biến lưu trữ trạng thái qua các level
    std::vector<std::map<int, std::vector<int>>> mapLevel;
    std::vector<std::vector<std::vector<double>>> DistanceMatrixLevel;
    std::vector<std::vector<bool>> C1Level;

    // Helper: Chuyển đổi ma trận khoảng cách khi gộp đỉnh
    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> convertMatrix (
        std::vector<std::vector<double>> currentMatrix, 
        std::vector<std::vector<double>> matrixReBefore, 
        std::map<int, std::vector<int>> Map
    );

    // Hàm mergeSol: Thực hiện gộp đỉnh (Matching & Coarsening)
    // Cập nhật signature để nhận ma trận tần suất và khoảng cách
    std::tuple<int, Solution, std::map<int, std::vector<int>>> mergeSol(
        Solution solution, 
        int NumCus, 
        std::vector<std::vector<int>> freqMatrix,       // Dùng cho Method A/C
        std::vector<std::vector<double>> MatrixRe,      // Dùng cho Method B
        std::vector<std::vector<double>> distanceMatrix,// Ma trận gốc (giữ lại để khớp call)
        std::vector<bool> C1
    );
    
    // Quy trình Coarsening (Gom nhóm)
    std::tuple<Solution, std::map<int, std::vector<int>>, std::vector<double>> mergeProcess(
        Config &config, 
        Input &input, 
        std::vector<std::map<int, std::vector<int>>> &mapLevel, 
        std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, 
        std::vector<std::vector<bool>> &C1Level
    ); 

    // Helper: Tách đỉnh (Uncoarsening)
    Solution splitSol(Solution solution, std::map<int, std::vector<int>> Map);

    // Quy trình Uncoarsening (Tách nhóm & Tinh chỉnh)
    std::tuple<Solution, std::vector<double>> splitProcess(
        Solution solution, 
        Config &config, 
        Input &input, 
        std::vector<std::map<int, std::vector<int>>> &mapLevel, 
        std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, 
        std::vector<std::vector<bool>> &C1Level
    );

    // Hàm chạy chính
    std::tuple<double, Solution, std::vector<double>> run(Config &config, Input &input);
};

#endif