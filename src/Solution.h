
#ifndef DASTS2_VERSION9_C_SOLUTION_H
#define DASTS2_VERSION9_C_SOLUTION_H

#include "vector"
#include "Config.h"
#include "Input.h"
#include "map"
#include "limits"

enum InitType
{
    ANGLE,
    DISTANCE,
    MIX
};

enum TripType
{
    DRONE,
    TECHNICIAN
};

enum RouteType
{
    INTER,
    INTRA,
    ALL
};

enum NeighborhoodType
{
    MOVE_10 = 1,
    MOVE_11 = 2,
    MOVE_20 = 3,
    MOVE_21 = 4,
    TWO_OPT = 5,
    EJECTION = 6,
    EJECTION2 = 7
};

enum InterRouteType
{
    INTER_RELOCATE = 1,
    INTER_EXCHANGE = 2,
    INTER_TWO_OPT = 3,
    INTER_OR_OPT = 4,
    INTER_CROSS_EXCHANGE = 5
};

enum IntraRouteType
{
    INTRA_RELOCATE = 1,
    INTRA_EXCHANGE = 2,
    INTRA_TWO_OPT = 3,
    INTRA_OR_OPT = 4
};

class Solution
{
public:
    Config config;
    Input input;
    std::vector<std::vector<std::vector<int>>> droneTripList;
    std::vector<std::vector<int>> techTripList;
    
    std::map<std::string, std::string> ext;
    double c{}, cz{}, dz{}, alpha1{}, alpha2{};

    Solution(Config &config, Input &input, double alpha1, double alpha2);

    Solution();

    static Solution *
    initSolution(Config &config, Input &input, InitType type = DISTANCE, double alpha1 = 0, double alpha2 = 0);

    double getScore();

    bool check_feasible();

    std::vector<double> getScoreATrip(int tripIndex, TripType type);

    Solution *relocate(const std::vector<std::string> &tabuList,  Solution &bestFeasibleSolution,
                       RouteType type = ALL);

    Solution *exchange(const std::vector<std::string> &tabuList,  Solution &bestFeasibleSolution,
                       RouteType type = ALL);

    Solution *orOpt(const std::vector<std::string> &tabuList,  Solution &bestFeasibleSolution,
                    RouteType type = ALL, int dis = 1);

    Solution *
    crossExchange(const std::vector<std::string> &tabuList,  Solution &bestFeasibleSolution,
                  RouteType type = ALL, int dis1 = 1, int dis2 = 0);

    Solution *twoOpt(const std::vector<std::string> &tabuList,  Solution &bestFeasibleSolution,
                     RouteType type = ALL);


    Solution *ejectionNeighborhoodAdd(Solution &bestFeasibleSolution);


    Solution ejection();

    
    void ejection(Solution &solution, std::vector<int> xIndex, TripType type, std::vector<int> &customerX, double gain, double &bestGain, int &level,
                  std::vector<std::pair<std::vector<int>, std::vector<int>>> &shiftSequence,
                  std::vector<std::pair<std::vector<int>, std::vector<int>>> &bestShiftSequence);


    void perturbation();

    std::string toString();
    void refactorSolution();

    void setInput (Input &input);

private:
    void initByDistance(bool reverse);

    void initByAngle(bool reverse, int direction);

    static bool checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val);

    static bool
    checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val1, const std::string &val2);

    static bool
    checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val1, const std::string &val2,
                       const std::string &val3);
};

#endif 
