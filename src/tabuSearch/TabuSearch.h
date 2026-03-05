
#ifndef DASTS2_VERSION9_C_TABUSEARCH_H
#define DASTS2_VERSION9_C_TABUSEARCH_H

#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Config.h"
#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Input.h"
#include "iostream"
#include "D:/Users/ADMIN/Documents/0.Study/Multi_Level/DASTS2_VERSION9_C/src/Solution.h"
#include "nlohmann/json.hpp"

using json = nlohmann::json;

class TabuSearch
{
public:
    Config config;
    Input input;

    Solution initSolution;
    Solution currentSolution;
    Solution bestSolution;

    int tabuDuration{};
    double alpha1{};
    double alpha2{};

    //TabuSearch();

    TabuSearch(Config &conf, Input &inp);

    void setConfigandInput(Config &conf, Input &inp);

    std::tuple<double, Solution, std::vector<std::vector<int>>> run(json &log, std::string &path_e, Input &input, Solution solution);

    Solution runPostOptimization(json &log, Solution solution);

    static Solution runEjection(Solution &solution);

    static Solution runInterRoute(Solution &solution);

    static Solution runIntraRoute(Solution &solution);

    void updatePenalty(double dz, double cz);
};

#endif 
