#ifndef DASTS2_VERSION9_C_MULTILEVEL_H
#define DASTS2_VERSION9_C_MULTILEVEL_H

#include "nlohmann/json.hpp"
#include "Config.h"
#include "Solution.h"
#include "Input.h"
#include "TabuSearch.h" // Giữ lại nếu cần tham khảo cũ
#include "LargeNeighborhoodSearch.h" // <--- Import LNS
#include <map>
#include <tuple>

class MultiLevel
{
public:
    Config config;
    Input input;
    
    // Thay thế vai trò của TabuSearch bằng LNS trong logic
    // TabuSearch tabuSearch; 
    LargeNeighborhoodSearch lns; 

    Solution initSolution;
    Solution currentSolution;
    Solution bestSolution;

    MultiLevel();

    MultiLevel(Config &config, Input &input);

    int numCustomer{};
    int tabuDuration{};
    int type8{};

    std::vector<std::vector<double>> droneTimeMatrix{};
    std::vector<std::vector<double>> techTimeMatrix{};

    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> convertMatrix (std::vector<std::vector<double>> currentMatrix, std::vector<std::vector<double>> matrixReBefore, std::map<int, std::vector<int>> Map);

    // Hàm này có thể không còn được dùng nếu thay thế hoàn toàn trong cpp
    std::tuple<double, Solution, std::vector<std::vector<int>>> tabuInMultiLevel (Solution solution, int numRun);

    std::vector<std::map<int, std::vector<int>>> mapLevel;

    std::vector<std::vector<std::vector<double>>> DistanceMatrixLevel;

    std::vector<std::vector<bool>> C1Level;

    std::tuple<Solution, std::vector<std::tuple<int, int>>> beUpdate(Solution solution, int NumCus, std::vector<std::vector<double>> distanceMatrix, int Level);

    std::tuple<int, Solution, std::map<int, std::vector<int>>> mergeSol(Solution solution, int NumCus, std::vector<std::vector<int>> mainMatrix, std::vector<std::vector<double>> MatrixRe, std::vector<std::vector<double>> distanceMatrix, std::vector<bool> C1);
    
    std::tuple<Solution, std::map<int, std::vector<int>>, std::vector<double>> mergeProcess(Config &config, Input &input, std::vector<std::map<int, std::vector<int>>> &mapLevel, std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, std::vector<std::vector<bool>> &C1Level); 

    std::tuple<int, Solution, std::map<int, std::vector<int>>> mergeSol_Version2(Solution solution, int NumCus, std::vector<std::vector<int>> mainMatrix);

    Solution splitSol(Solution solution, std::map<int, std::vector<int>> Map);

   std::tuple<Solution, std::vector<double>> splitProcess(Solution solution, Config &config, Input &input, std::vector<std::map<int, std::vector<int>>> &mapLevel, std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, std::vector<std::vector<bool>> &C1Level);

    std::tuple<double, Solution, std::vector<double>> run(Config &config, Input &input);
};

#endif