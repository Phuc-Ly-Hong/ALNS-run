
#define _USE_MATH_DEFINES
#include <cmath>
#include "Solution.h"
#include "Utils.h"
#include "map"
#include "nlohmann/json.hpp"
#include "Random.h"

using json = nlohmann::json;
using Random = effolkronium::random_static;

Solution *Solution::initSolution(Config &config, Input &input, InitType type, double alpha1, double alpha2)
{
    Solution *best = nullptr, *current;
    double bestScore = std::numeric_limits<double>::max(), currentScore;
    //if (type != DISTANCE)
    if (false)
    {
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(true, 1);
        currentScore = current->getScore();
        
        if (currentScore < bestScore)
        {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(false, 1);
        currentScore = current->getScore();
        if (currentScore < bestScore)
        {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(false, -1);
        currentScore = current->getScore();
        if (currentScore < bestScore)
        {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByAngle(true, -1);
        currentScore = current->getScore();
        if (currentScore < bestScore)
        {
            best = current;
            bestScore = currentScore;
        }
    }
    if (type != ANGLE)
    {
        current = new Solution(config, input, alpha1, alpha2);
        current->initByDistance(false);
        currentScore = current->getScore();
        if (currentScore < bestScore)
        {
            best = current;
            bestScore = currentScore;
        }
        current = new Solution(config, input, alpha1, alpha2);
        current->initByDistance(true);
        currentScore = current->getScore();
        if (currentScore < bestScore)
        {
            best = current;
            bestScore = currentScore;
        }
    }
    return best;
}

void Solution::initByDistance(bool reverse)
{
    std::vector<std::vector<int>> orderDistance(input.numCus + 1);

    for (int i = 0; i < input.numCus + 1; i++)
    {
        for (size_t e : Utils::sortIndices(input.distances[i], reverse))
        {
            if (e != i && e != input.numCus + 1)
            {
                orderDistance[i].emplace_back(e);
            }
        }
    }

    std::vector<double> travelTime(config.numTech + config.numDrone, 0);
    std::vector<bool> visitedCus(input.numCus + 1, false);
    visitedCus[0] = true;
    int numVisitedCus = 0;
    int nIter = 0, i = 0;
    //&&  nIter < (config.numDrone + config.numTech) * input.numCus
    while (numVisitedCus < input.numCus)
    {
        //std::cout << std::endl << "nIter = " << nIter << ", i = " << i;
        nIter++;
        std::vector<std::vector<double>> time;
        int lastCus;
        int index = i;
        if (i < config.numDrone)
        {
            time = input.droneTimes;
            if (droneTripList[index].empty())
            {
                droneTripList[index].emplace_back();
            }
            if (droneTripList[index].back().empty())
            {
                lastCus = 0;
            }
            else
            {
                lastCus = droneTripList[index].back().back();
            }
            int nextCus = -1;
            for (int tmp : orderDistance[lastCus])
            {
                if (!visitedCus[tmp] && !input.cusOnlyServedByTech[tmp])
                {
                    nextCus = tmp;
                    break;
                }
            }

            if (nextCus <= 0)
            {
                i++;
                i %= config.numDrone + config.numTech;
                continue;
            }

            double timeGoToFirstCus = time[0][nextCus];
            if (!droneTripList[index].back().empty())
            {
                timeGoToFirstCus = time[0][droneTripList[index].back()[0]];
            }

            double flightTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0];
            double waitTime = flightTime - timeGoToFirstCus;

            if (flightTime > config.droneLimitationFlightTime || waitTime > config.sampleLimitationWaitingTime)
            {
                droneTripList[index].emplace_back();
                travelTime[i] = time[0][nextCus];
            }
            else
            {
                travelTime[i] += time[lastCus][nextCus];
            }
            droneTripList[index].back().push_back(nextCus);

            visitedCus[nextCus] = true;
            numVisitedCus++;
        }
        else
        {
            time = input.techTimes;
            index -= config.numDrone;

            if (techTripList[index].empty())
            {
                lastCus = 0;
            }
            else
            {
                lastCus = techTripList[index].back();
            }

            int nextCus = -1;
            for (int tmp : orderDistance[lastCus])
            {
                if (!visitedCus[tmp])
                {
                    nextCus = tmp;
                    break;
                }
            }

            if (nextCus <= 0)
            {
                i++;
                i %= config.numDrone + config.numTech;
                continue;
            }

            double timeGoToFirstCus = time[0][nextCus];
            if (!techTripList[index].empty())
            {
                timeGoToFirstCus = time[0][techTripList[index][0]];
            }

            double waitTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0] - timeGoToFirstCus;
            if ((waitTime <= config.sampleLimitationWaitingTime) or (nIter > (config.numDrone + config.numTech) * input.numCus))
            {
                techTripList[index].push_back(nextCus);
                travelTime[i] += time[lastCus][nextCus];
                visitedCus[nextCus] = true;
                numVisitedCus++;
            }
        }

        i++;
        i %= config.numDrone + config.numTech;
    }
}


void Solution::initByAngle(bool reverse, int direction)
{
    std::vector<std::vector<int>> orderDistance(input.numCus + 1);

    for (int i = 0; i < input.numCus + 1; i++)
    {
        for (size_t e : Utils::sortIndices(input.distances[i], reverse))
        {
            if (e != i && e != input.numCus + 1)
            {
                orderDistance[i].emplace_back(e);
            }
        }
    }

    int pivotCus = orderDistance[0][0];
    std::vector<std::vector<double>> coordinates = Utils::slice(input.coordinates, 1,
                                                                (int)input.coordinates.size() - 2);
    std::vector<double> angle;

    std::vector<double> perpendicular = {coordinates[pivotCus - 1][1], coordinates[pivotCus - 1][0] * -1};

    for (int i = 0; i < (int)coordinates.size(); i++)
    {
        if (i == pivotCus - 1)
        {
            angle.push_back(0.);
            continue;
        }
        double inner = std::inner_product(coordinates[pivotCus - 1].begin(),
                                          coordinates[pivotCus - 1].end(),
                                          coordinates[i].begin(),
                                          0.);
        double norm = sqrt(std::inner_product(coordinates[pivotCus - 1].begin(),
                                              coordinates[pivotCus - 1].end(),
                                              coordinates[pivotCus - 1].begin(),
                                              0.)) *
                      sqrt(std::inner_product(coordinates[i].begin(),
                                              coordinates[i].end(),
                                              coordinates[i].begin(),
                                              0.));
        double acos = inner / norm;
        double tmpAngle = std::acos(acos);
        double cc = std::inner_product(perpendicular.begin(), perpendicular.end(), coordinates[i].begin(), 0.);
        if ((direction == 1 && cc < 0) || (direction == -1 && cc > 0))
        {
            tmpAngle = 2 * M_PI - tmpAngle;
        }
        angle.push_back(tmpAngle);
    }
    std::vector<int> orderAngle;
    for (size_t e : Utils::sortIndices(angle))
    {
        orderAngle.push_back((int)e + 1);
    }
    std::vector<double> travelTime(config.numTech + config.numDrone, 0);
    std::vector<bool> visitedCus(input.numCus + 1, false);
    visitedCus[0] = true;
    int numVisitedCus = 0;
    int nIter = 0, i = 0;

    while (numVisitedCus < input.numCus &&
           nIter < (config.numDrone + config.numTech) * input.numCus)
    {
        nIter++;
        std::vector<std::vector<double>> time;
        int lastCus;
        int index = i;

        if (i < config.numDrone)
        {
            time = input.droneTimes;
            travelTime[i] = 0.0;
            droneTripList[index].emplace_back();
            int j = 0;
            lastCus = 0;
            double timeGoToFirstCus = time[0][orderAngle[0]];
            int remainCus = (int)orderAngle.size();
            while (j < remainCus)
            {
                int nextCus = orderAngle[j];
                if (!input.cusOnlyServedByTech[nextCus])
                {
                    double flightTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0];
                    double waitTime = flightTime - timeGoToFirstCus;

                    if (flightTime <= config.droneLimitationFlightTime &&
                        waitTime <= config.sampleLimitationWaitingTime)
                    {
                        travelTime[i] += time[lastCus][nextCus];
                        droneTripList[index].back().push_back(nextCus);
                        visitedCus[nextCus] = true;
                        numVisitedCus++;
                        orderAngle.erase(orderAngle.begin() + j);
                        lastCus = nextCus;
                        remainCus--;
                    }
                    else
                    {
                        j++;
                    }
                }
                else
                {
                    j++;
                }
            }
        }
        else
        {
            index -= config.numDrone;
            if (techTripList[index].empty())
            {
                time = input.techTimes;
                int j = 0;
                lastCus = 0;
                double timeGoToFirstCus = time[0][orderAngle[0]];
                int remainCus = (int)orderAngle.size();
                while (j < remainCus)
                {
                    int nextCus = orderAngle[j];
                    double waitTime = travelTime[i] + time[lastCus][nextCus] + time[nextCus][0] - timeGoToFirstCus;

                    if (waitTime <= config.sampleLimitationWaitingTime)
                    {
                        techTripList[index].push_back(nextCus);
                        travelTime[i] += time[lastCus][nextCus];
                        visitedCus[nextCus] = true;
                        numVisitedCus++;
                        orderAngle.erase(orderAngle.begin() + j);
                        lastCus = nextCus;
                        remainCus--;
                    }
                    else
                    {
                        j++;
                    }
                }
            }
        }

        i++;
        i %= config.numDrone + config.numTech;
    }
}

double Solution::getScore()
{

    std::vector<double> techCompleteTime(config.numTech, 0);
    std::vector<double> droneCompleteTime(config.numDrone, 0);
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    std::vector<std::vector<double>> droneTripCompleteTime;
    int maxTrip = 0;
    for (auto &i : droneTripList)
    {
        if (maxTrip < i.size())
        {
            maxTrip = (int)i.size();
        }
    }

    for (int i = 0; i < config.numDrone; i++)
    {
        std::vector<double> tripTime(maxTrip, 0);
        droneTripCompleteTime.push_back(tripTime);
    }
    double tmp, tmp1;
    dz = 0, cz = 0;

    double allTechTime = 0, allDroneTime = 0;

    for (int i = 0; i < techTripList.size(); i++)
    {
        if (techTripList[i].empty())
        {
            continue;
        }

        tmp = input.techTimes[0][techTripList[i][0]];
        cusCompleteTime[techTripList[i][0]] = tmp;

        for (int j = 0; j < (int)techTripList[i].size() - 1; j++)
        {
            tmp += input.techTimes[techTripList[i][j]][techTripList[i][j + 1]];
            cusCompleteTime[techTripList[i][j + 1]] = tmp;
        }
        techCompleteTime[i] = tmp + input.techTimes[techTripList[i].back()][0];
        if (techCompleteTime[i] > allTechTime)
        {
            allTechTime = techCompleteTime[i];
        }
    }
    for (int i = 0; i < droneTripList.size(); i++)
    {
        tmp = 0;
        for (int j = 0; j < droneTripList[i].size(); j++)
        {
            if (droneTripList[i][j].empty())
            {
                continue;
            }
            tmp1 = input.droneTimes[0][droneTripList[i][j][0]];
            cusCompleteTime[droneTripList[i][j][0]] = tmp1;

            for (int k = 0; k < (int)droneTripList[i][j].size() - 1; k++)
            {
                tmp1 += input.droneTimes[droneTripList[i][j][k]][droneTripList[i][j][k + 1]];
                cusCompleteTime[droneTripList[i][j][k + 1]] = tmp1;
            }
            droneTripCompleteTime[i][j] = tmp1 + input.droneTimes[droneTripList[i][j].back()][0];
            tmp += droneTripCompleteTime[i][j];
        }
        droneCompleteTime[i] = tmp;
        if (tmp > allDroneTime)
        {
            allDroneTime = tmp;
        }
    }
    c = std::max(allDroneTime, allTechTime);

    for (int i = 0; i < techTripList.size(); i++)
    {
        if (techTripList[i].empty())
        {
            continue;
        }

        for (int j : techTripList[i])
        {
            cz += std::max(0., techCompleteTime[i] - cusCompleteTime[j] - config.sampleLimitationWaitingTime);
        }
    }

    for (int i = 0; i < droneTripList.size(); i++)
    {
        for (int j = 0; j < droneTripList[i].size(); j++)
        {
            if (droneTripList[i][j].empty())
            {
                continue;
            }
            for (int k : droneTripList[i][j])
            {
                cz += std::max(0.,
                               droneTripCompleteTime[i][j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
            }
            dz += std::max(0., droneTripCompleteTime[i][j] - config.droneLimitationFlightTime);
        }
    }

    return c + alpha1 * cz + alpha2 * dz;
}

void Solution::setInput(Input &input)
{
    this->input = input;
}

Solution::Solution(Config &config, Input &input, double alpha1, double alpha2)
{
    this->input = input;
    this->config = config;
    for (int i = 0; i < config.numDrone; i++)
    {
        droneTripList.emplace_back();
    }

    for (int i = 0; i < config.numTech; i++)
    {
        techTripList.emplace_back();
    }

    this->alpha1 = alpha1;
    this->alpha2 = alpha2;
}

Solution *Solution::relocate(const std::vector<std::string> &tabuList, Solution &bestFeasibleSolution, RouteType type)
{
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double bestScore = bestFeasibleSolution.getScore();
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

    if (type != INTER)
    {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++)
                {

                    if (xIndex != 0)
                    {
                        Solution s = *this;

                        s.droneTripList[droneIndex][tripIndex].insert(
                            s.droneTripList[droneIndex][tripIndex].begin(),
                            s.droneTripList[droneIndex][tripIndex][xIndex]);

                        s.droneTripList[droneIndex][tripIndex].erase(
                            s.droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);

                        double newScore = s.getScore();

                        //Method 1:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){ // check aspiration condition
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                          droneTripList[droneIndex][tripIndex][xIndex])))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;
                        }

                        /*if (newScore < curScore)
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                          s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }*/
                    }

                    for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++)
                    {
                        if (yIndex == xIndex || yIndex == xIndex - 1)
                        {
                            continue;
                        }

                        Solution s = *this;

                        s.droneTripList[droneIndex][tripIndex].insert(
                            s.droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                            s.droneTripList[droneIndex][tripIndex][xIndex]);

                        if (xIndex < yIndex)
                        {
                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                        }
                        else
                        {
                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                        }

                        double newScore = s.getScore();

                        //Method 1:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                          droneTripList[droneIndex][tripIndex][xIndex])))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;
                        }

                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                          s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++)
            {

                if (xIndex != 0)
                {
                    Solution s = *this;

                    s.techTripList[techIndex].insert(s.techTripList[techIndex].begin(),
                                                     s.techTripList[techIndex][xIndex]);

                    s.techTripList[techIndex].erase(s.techTripList[techIndex].begin() + xIndex + 1);

                    double newScore = s.getScore();

                    //Method 1:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                        techTripList[techIndex][xIndex])))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                        curScore = newScore;
                    }
                    
                    
                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                      s.techTripList[techIndex][xIndex])) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                    }*/
                }

                for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++)
                {
                    if (yIndex == xIndex || yIndex == xIndex - 1)
                    {
                        continue;
                    }

                    Solution s = *this;

                    s.techTripList[techIndex].insert(s.techTripList[techIndex].begin() + yIndex + 1,
                                                     s.techTripList[techIndex][xIndex]);

                    if (xIndex < yIndex)
                    {
                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex);
                    }
                    else
                    {
                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex + 1);
                    }

                    double newScore = s.getScore();

                    //Method 1:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                        techTripList[techIndex][xIndex])))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                        curScore = newScore;
                    }

                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                      s.techTripList[techIndex][xIndex])) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                    }*/
                }
            }
        }
    }
    if (type != INTRA)
    {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++)
                {

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                    {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                        {
                            if (droneIndex2 == droneIndex && tripIndex == tripIndex2)
                            {
                                continue;
                            }

                            Solution s = *this;
                            s.droneTripList[droneIndex2][tripIndex2].insert(
                                s.droneTripList[droneIndex2][tripIndex2].begin(),
                                s.droneTripList[droneIndex][tripIndex][xIndex]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                            double newScore = s.getScore();
                            //Method 1:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                            droneTripList[droneIndex][tripIndex][xIndex])))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }
                            
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                              s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);

                                    curScore = newScore;
                                }
                            }*/

                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++)
                            {
                                s = *this;
                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                    s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                    s.droneTripList[droneIndex][tripIndex][xIndex]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                                newScore = s.getScore();
                                //Method 1:
                                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                    isImproved = true;
                                    bestFeasibleSolution.droneTripList = s.droneTripList;
                                    bestFeasibleSolution.techTripList = s.techTripList;
                                    bestScore = newScore;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;
                                    
                                } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                                droneTripList[droneIndex][tripIndex][xIndex])))
                                {
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;
                                }
                                /*if (newScore < curScore)
                                {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                                  s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon))
                                    {
                                        isImproved = true;
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = std::to_string(
                                            droneTripList[droneIndex][tripIndex][xIndex]);

                                        curScore = newScore;
                                    }
                                }*/
                            }
                        }
                    }

                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
                    {
                        Solution s = *this;

                        s.techTripList[techIndex].insert(s.techTripList[techIndex].begin(),
                                                         s.droneTripList[droneIndex][tripIndex][xIndex]);

                        s.droneTripList[droneIndex][tripIndex].erase(
                            s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                        double newScore = s.getScore();
                        //Method 1:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                          droneTripList[droneIndex][tripIndex][xIndex])))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                          s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);

                                curScore = newScore;
                            }
                        }*/

                        for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++)
                        {
                            s = *this;

                            s.techTripList[techIndex].insert(s.techTripList[techIndex].begin() + yIndex + 1,
                                                             s.droneTripList[droneIndex][tripIndex][xIndex]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex);

                            newScore = s.getScore();
                            //Method 1:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                    droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore; 
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                            droneTripList[droneIndex][tripIndex][xIndex])))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                              s.droneTripList[droneIndex][tripIndex][xIndex])) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(
                                        droneTripList[droneIndex][tripIndex][xIndex]);
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++)
            {

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
                {
                    if (techIndex == techIndex2)
                    {
                        continue;
                    }

                    Solution s = *this;

                    s.techTripList[techIndex2].insert(s.techTripList[techIndex2].begin(),
                                                      s.techTripList[techIndex][xIndex]);

                    s.techTripList[techIndex].erase(
                        s.techTripList[techIndex].begin() + xIndex);

                    double newScore = s.getScore();
                    //Method 1:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                        techTripList[techIndex][xIndex])))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                        curScore = newScore;
                    }
                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                      s.techTripList[techIndex][xIndex])) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                    }*/

                    for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++)
                    {
                        Solution s = *this;

                        s.techTripList[techIndex2].insert(s.techTripList[techIndex2].begin() + yIndex + 1,
                                                          s.techTripList[techIndex][xIndex]);

                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex);

                        double newScore = s.getScore();
                        //Method 1:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                            techTripList[techIndex][xIndex])))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                          s.techTripList[techIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                                curScore = newScore;
                            }
                        }*/
                    }
                }

                // drone
                if (input.cusOnlyServedByTech[techTripList[techIndex][xIndex]])
                {
                    continue;
                }

                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
                {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
                    {
                        Solution s = *this;

                        s.droneTripList[droneIndex][tripIndex].insert(
                            s.droneTripList[droneIndex][tripIndex].begin(),
                            s.techTripList[techIndex][xIndex]);

                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex);

                        double newScore = s.getScore();
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                            techTripList[techIndex][xIndex])))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                          s.techTripList[techIndex][xIndex])) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                                curScore = newScore;
                            }
                        }*/

                        for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++)
                        {
                            Solution s = *this;

                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                                s.techTripList[techIndex][xIndex]);

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex);

                            double newScore = s.getScore();
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                curScore = newScore;
                                bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);  
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && checkTabuCondition(tabuList, std::to_string(
                                                                                                techTripList[techIndex][xIndex])))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, std::to_string(
                                                                                              s.techTripList[techIndex][xIndex])) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = std::to_string(techTripList[techIndex][xIndex]);
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }
    }

    if (bestSolution->getScore() < std::abs(config.tabuEpsilon)){
        return this;
    }
    return bestSolution;
}

Solution *Solution::exchange(const std::vector<std::string> &tabuList, Solution &bestFeasibleSolution, RouteType type)
{
    double bestScore = bestFeasibleSolution.getScore();
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

    if (type != INTRA)
    {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++)
                {

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                    {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                        {
                            if (droneIndex2 == droneIndex && tripIndex == tripIndex2)
                            {
                                continue;
                            }

                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++)
                            {
                                Solution s = *this;
                                std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                          s.droneTripList[droneIndex2][tripIndex2][yIndex]);
                                double newScore = s.getScore();

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                std::string val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);

                                //Method 2:
                                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                    isImproved = true;
                                    bestFeasibleSolution.droneTripList = s.droneTripList;
                                    bestFeasibleSolution.techTripList = s.techTripList;
                                    bestScore = newScore;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                    
                                } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                                {
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }

                                /*if (newScore < curScore)
                                {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon))
                                    {
                                        isImproved = true;
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                }*/
                            }
                        }
                    }

                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
                    {
                        for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++)
                        {
                            Solution s = *this;

                            std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                      s.techTripList[techIndex][yIndex]);

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            std::string val2 = std::to_string(techTripList[techIndex][yIndex]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++)
            {

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
                {
                    if (techIndex == techIndex2)
                    {
                        continue;
                    }
                    for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++)
                    {
                        Solution s = *this;

                        std::swap(s.techTripList[techIndex][xIndex], s.techTripList[techIndex2][yIndex]);

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex2][yIndex]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }

                // drone
                if (input.cusOnlyServedByTech[techTripList[techIndex][xIndex]])
                {
                    continue;
                }

                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
                {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
                    {
                        for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++)
                        {
                            Solution s = *this;

                            std::swap(s.techTripList[techIndex][xIndex],
                                      s.droneTripList[droneIndex][tripIndex][yIndex]);

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }
    }

    if (type != INTER)
    {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = 0; xIndex < (int)droneTripList[droneIndex][tripIndex].size() - 1; xIndex++)
                {
                    for (int yIndex = xIndex + 1; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++)
                    {
                        Solution s = *this;

                        std::swap(s.droneTripList[droneIndex][tripIndex][xIndex],
                                  s.droneTripList[droneIndex][tripIndex][yIndex]);

                        double newScore = s.getScore();
                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = 0; xIndex < (int)techTripList[techIndex].size() - 1; xIndex++)
            {
                for (int yIndex = xIndex + 1; yIndex < techTripList[techIndex].size(); yIndex++)
                {
                    Solution s = *this;

                    std::swap(s.techTripList[techIndex][xIndex],
                              s.techTripList[techIndex][yIndex]);

                    double newScore = s.getScore();
                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][yIndex]);
                    //Method 2:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                    }
                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                    }*/
                }
            }
        }
    }

    if (bestSolution->getScore() < std::abs(config.tabuEpsilon)){
        return this;
    }
    return bestSolution;
}

bool Solution::checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val)
{
    if (tabuList.empty())
    {
        return true;
    }
    else
    {
        for (const std::string &s : tabuList)
        {
            if (s == val)
            {
                return false;
            }
        }
    }
    return true;
}

bool Solution::checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val1,
                                  const std::string &val2)
{
    if (tabuList.empty())
    {
        return true;
    }
    else
    {
        std::string tmp1 = val1 + "-" + val2;
        std::string tmp2 = val2 + "-" + val1;
        for (const std::string &s : tabuList)
        {
            if (s == tmp1 || s == tmp2)
            {
                return false;
            }
        }
    }
    return true;
}

bool Solution::checkTabuCondition(const std::vector<std::string> &tabuList, const std::string &val1, const std::string &val2,
                                  const std::string &val3)
{
    if (tabuList.empty())
    {
        return true;
    }
    else
    {
        std::string tmp = val1 + "-" + val2 + "-" + val3;
        for (const std::string &s : tabuList)
        {
            if (s == tmp)
            {
                return false;
            }
        }
    }
    return true;
}

Solution *Solution::orOpt(const std::vector<std::string> &tabuList, Solution &bestFeasibleSolution, RouteType type, int dis)
{
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double bestScore = bestFeasibleSolution.getScore();
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

    if (type != INTRA)
    {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                if (droneTripList[droneIndex][tripIndex].size() <= dis)
                {
                    continue;
                }
                for (int xIndex = 0; xIndex < (int)droneTripList[droneIndex][tripIndex].size() - dis; xIndex++)
                {
                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                    {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                        {
                            if (droneIndex2 == droneIndex && tripIndex == tripIndex2)
                            {
                                continue;
                            }
                            Solution s = *this;
                            for (int i = dis; i >= 0; i--)
                            {
                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                    s.droneTripList[droneIndex2][tripIndex2].begin(),
                                    droneTripList[droneIndex][tripIndex][xIndex + i]);
                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + i);
                            }

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);

                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }

                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                            for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++)
                            {
                                s = *this;
                                for (int i = 0; i <= dis; i++)
                                {
                                    s.droneTripList[droneIndex2][tripIndex2].insert(
                                        s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + i + 1,
                                        droneTripList[droneIndex][tripIndex][xIndex + i]);

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                                }

                                newScore = s.getScore();

                                val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);

                                //Method 2:
                                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                    isImproved = true;
                                    bestFeasibleSolution.droneTripList = s.droneTripList;
                                    bestFeasibleSolution.techTripList = s.techTripList;
                                    bestScore = newScore;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                    
                                } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                                {
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }

                                /*if (newScore < curScore)
                                {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon))
                                    {
                                        isImproved = true;
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                }*/
                            }
                        }
                    }

                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
                    {
                        Solution s = *this;

                        for (int i = dis; i >= 0; i--)
                        {
                            s.techTripList[techIndex].insert(
                                s.techTripList[techIndex].begin(),
                                droneTripList[droneIndex][tripIndex][xIndex + i]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex + i);
                        }

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/

                        for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++)
                        {
                            s = *this;
                            for (int i = 0; i <= dis; i++)
                            {
                                s.techTripList[techIndex].insert(
                                    s.techTripList[techIndex].begin() + yIndex + i + 1,
                                    droneTripList[droneIndex][tripIndex][xIndex + i]);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                            }

                            newScore = s.getScore();

                            val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            if (techTripList[techIndex].size() <= dis)
            {
                continue;
            }
            for (int xIndex = 0; xIndex < (int)techTripList[techIndex].size() - dis; xIndex++)
            {
                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
                {
                    if (techIndex == techIndex2)
                    {
                        continue;
                    }

                    Solution s = *this;
                    for (int i = dis; i >= 0; i--)
                    {
                        s.techTripList[techIndex2].insert(
                            s.techTripList[techIndex2].begin(),
                            techTripList[techIndex][xIndex + i]);

                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex + i);
                    }

                    double newScore = s.getScore();

                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                    //Method 2:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                    }
                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                    }*/

                    for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++)
                    {
                        s = *this;
                        for (int i = 0; i <= dis; i++)
                        {
                            s.techTripList[techIndex2].insert(
                                s.techTripList[techIndex2].begin() + yIndex + i + 1,
                                techTripList[techIndex][xIndex + i]);

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex);
                        }

                        newScore = s.getScore();

                        val1 = std::to_string(techTripList[techIndex][xIndex]);
                        val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }

                // drone
                bool canMoveToDroneTrip = true;
                for (int i = xIndex; i < xIndex + dis; i++)
                {
                    if (input.cusOnlyServedByTech[techTripList[techIndex][i]])
                    {
                        canMoveToDroneTrip = false;
                        break;
                    }
                }
                if (!canMoveToDroneTrip)
                {
                    continue;
                }

                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
                {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
                    {
                        Solution s = *this;

                        for (int i = dis; i >= 0; i--)
                        {
                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin(),
                                techTripList[techIndex][xIndex + i]);

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex + i);
                        }

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                        std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                        for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++)
                        {
                            s = *this;
                            for (int i = 0; i <= dis; i++)
                            {
                                s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex + i + 1,
                                    techTripList[techIndex][xIndex + i]);

                                s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + xIndex);
                            }

                            newScore = s.getScore();

                            val1 = std::to_string(techTripList[techIndex][xIndex]);
                            val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }
    }

    if (type != INTER)
    {
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                if (droneTripList[droneIndex][tripIndex].size() <= dis)
                {
                    continue;
                }
                for (int xIndex = 0; xIndex < (int)droneTripList[droneIndex][tripIndex].size() - dis; xIndex++)
                {
                    if (xIndex != 0)
                    {
                        Solution s = *this;
                        for (int i = dis; i >= 0; i--)
                        {
                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin(),
                                droneTripList[droneIndex][tripIndex][xIndex + i]);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis + 1);
                        }

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                    for (int yIndex = 0; yIndex < droneTripList[droneIndex][tripIndex].size(); yIndex++)
                    {
                        if (yIndex == xIndex || (yIndex < xIndex && yIndex + dis >= xIndex))
                        {
                            continue;
                        }

                        Solution s = *this;

                        for (int i = 0; i <= dis; i++)
                        {
                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin() + yIndex + i + 1,
                                droneTripList[droneIndex][tripIndex][xIndex + i]);
                        }

                        for (int i = 0; i <= dis; i++)
                        {
                            if (xIndex < yIndex)
                            {
                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex);
                            }
                            else
                            {
                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis + 1);
                            }
                        }

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                        std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            if (techTripList[techIndex].size() <= dis)
            {
                continue;
            }
            for (int xIndex = 0; xIndex < (int)techTripList[techIndex].size() - dis; xIndex++)
            {
                if (xIndex != 0)
                {
                    Solution s = *this;
                    for (int i = dis; i >= 0; i--)
                    {
                        s.techTripList[techIndex].insert(
                            s.techTripList[techIndex].begin(),
                            techTripList[techIndex][xIndex + i]);

                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex + dis + 1);
                    }

                    double newScore = s.getScore();

                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                    //Method 2:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                    }
                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                    }*/
                }

                for (int yIndex = 0; yIndex < techTripList[techIndex].size(); yIndex++)
                {
                    if (yIndex == xIndex || (yIndex < xIndex && yIndex + dis >= xIndex))
                    {
                        continue;
                    }

                    Solution s = *this;
                    for (int i = 0; i <= dis; i++)
                    {
                        s.techTripList[techIndex].insert(
                            s.techTripList[techIndex].begin() + yIndex + i + 1,
                            techTripList[techIndex][xIndex + i]);
                    }

                    for (int i = 0; i <= dis; i++)
                    {
                        if (xIndex < yIndex)
                        {
                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex);
                        }
                        else
                        {
                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex + dis + 1);
                        }
                    }
                    double newScore = s.getScore();

                    std::string val1 = std::to_string(techTripList[techIndex][xIndex]);
                    std::string val2 = std::to_string(techTripList[techIndex][xIndex + dis]);
                    //Method 2:
                    if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                        isImproved = true;
                        bestFeasibleSolution.droneTripList = s.droneTripList;
                        bestFeasibleSolution.techTripList = s.techTripList;
                        bestScore = newScore;
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                        
                    } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                    {
                        bestSolution->droneTripList = s.droneTripList;
                        bestSolution->techTripList = s.techTripList;
                        bestSolution->ext["state"] = "";
                        bestSolution->ext["state"] += val1;
                        bestSolution->ext["state"] += "-";
                        bestSolution->ext["state"] += val2;
                        curScore = newScore;
                    }
                    /*if (newScore < curScore)
                    {
                        if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                     abs(newScore - baseScore) > config.tabuEpsilon))
                        {
                            isImproved = true;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                    }*/
                }
            }
        }
    }

    if (bestSolution->getScore() < std::abs(config.tabuEpsilon)){
        return this;
    }
    return bestSolution;
}

Solution *Solution::twoOpt(const std::vector<std::string> &tabuList, Solution &bestFeasibleSolution, RouteType type)
{
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double bestScore = bestFeasibleSolution.getScore();
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;

    if (type != INTRA)
    {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = -1; xIndex < (int)droneTripList[droneIndex][tripIndex].size(); xIndex++)
                {
                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
                    {
                        for (int yIndex = -1; yIndex < (int)techTripList[techIndex].size(); yIndex++)
                        {
                            std::vector<int> tmp1, tmp2;
                            if (xIndex >= 0)
                            {
                                tmp1.insert(tmp1.end(),
                                            droneTripList[droneIndex][tripIndex].begin(),
                                            droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                            }

                            if (yIndex >= 0)
                            {
                                tmp2.insert(tmp2.end(),
                                            techTripList[techIndex].begin(),
                                            techTripList[techIndex].begin() + yIndex + 1);
                            }

                            if (yIndex < (int)techTripList[techIndex].size())
                            {
                                tmp1.insert(tmp1.end(),
                                            techTripList[techIndex].begin() + yIndex + 1,
                                            techTripList[techIndex].end());
                            }

                            if (xIndex < (int)droneTripList[droneIndex][tripIndex].size())
                            {
                                tmp2.insert(tmp2.end(),
                                            droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                            droneTripList[droneIndex][tripIndex].end());
                            }
                            Solution s = *this;

                            s.droneTripList[droneIndex][tripIndex] = tmp1;
                            s.techTripList[techIndex] = tmp2;

                            double newScore = s.getScore();

                            std::string val1 = "0";
                            if (xIndex >= 0)
                            {
                                val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                            }
                            std::string val2 = "0";
                            if (yIndex >= 0)
                            {
                                val2 = std::to_string(techTripList[techIndex][yIndex]);
                            }

                            //Method 2:
                            //std::cout << std::endl << (!isImproved) << " and " << (newScore) << " -- " << curScore << " and " << checkTabuCondition(tabuList, val1, val2);

                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }

                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                    {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                        {
                            if (droneIndex == droneIndex2 && tripIndex == tripIndex2)
                            {
                                continue;
                            }
                            for (int yIndex = -1;
                                 yIndex < (int)droneTripList[droneIndex2][tripIndex2].size(); yIndex++)
                            {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0)
                                {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex][tripIndex].begin(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0)
                                {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1);
                                }

                                if (yIndex < (int)droneTripList[droneIndex2][tripIndex2].size())
                                {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                                droneTripList[droneIndex2][tripIndex2].end());
                                }

                                if (xIndex < (int)droneTripList[droneIndex][tripIndex].size())
                                {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                                droneTripList[droneIndex][tripIndex].end());
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex] = tmp1;
                                s.droneTripList[droneIndex2][tripIndex2] = tmp2;

                                double newScore = s.getScore();
                                std::string val1 = "0";
                                if (xIndex >= 0)
                                {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0)
                                {
                                    val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                }
                                //Method 2:
                                //std::cout << std::endl << (!isImproved) << " and " << (newScore) << " -- " << curScore << " and " << checkTabuCondition(tabuList, val1, val2);

                                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                    isImproved = true;
                                    bestFeasibleSolution.droneTripList = s.droneTripList;
                                    bestFeasibleSolution.techTripList = s.techTripList;
                                    bestScore = newScore;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                    
                                } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                                {
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                                /*if (newScore < curScore)
                                {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon))
                                    {
                                        isImproved = true;
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                }*/
                            }
                        }
                    }
                }
            }
        }

        // tech
        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = -1; xIndex < (int)techTripList[techIndex].size(); xIndex++)
            {
                // drone
                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
                {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
                    {
                        for (int yIndex = -1; yIndex < (int)droneTripList[droneIndex][tripIndex].size(); yIndex++)
                        {

                            bool canMoveToDroneTrip = true;
                            for (int i = xIndex + 1; i < techTripList[techIndex].size(); i++)
                            {
                                if (input.cusOnlyServedByTech[techTripList[techIndex][i]])
                                {
                                    canMoveToDroneTrip = false;
                                    break;
                                }
                            }
                            if (!canMoveToDroneTrip)
                            {
                                continue;
                            }

                            std::vector<int> tmp1, tmp2;
                            if (xIndex >= 0)
                            {
                                tmp1.insert(tmp1.end(),
                                            techTripList[techIndex].begin(),
                                            techTripList[techIndex].begin() + xIndex + 1);
                            }

                            if (yIndex >= 0)
                            {
                                tmp2.insert(tmp2.end(),
                                            droneTripList[droneIndex][tripIndex].begin(),
                                            droneTripList[droneIndex][tripIndex].begin() + yIndex + 1);
                            }

                            if (yIndex < (int)droneTripList[droneIndex][tripIndex].size())
                            {
                                tmp1.insert(tmp1.end(),
                                            droneTripList[droneIndex][tripIndex].begin() + yIndex + 1,
                                            droneTripList[droneIndex][tripIndex].end());
                            }

                            if (xIndex < (int)techTripList[techIndex].size())
                            {
                                tmp2.insert(tmp2.end(),
                                            techTripList[techIndex].begin() + xIndex + 1,
                                            techTripList[techIndex].end());
                            }
                            Solution s = *this;

                            s.techTripList[techIndex] = tmp1;
                            s.droneTripList[droneIndex][tripIndex] = tmp2;

                            double newScore = s.getScore();
                            std::string val1 = "0";
                            if (xIndex >= 0)
                            {
                                val1 = std::to_string(techTripList[techIndex][xIndex]);
                            }
                            std::string val2 = "0";
                            if (yIndex >= 0)
                            {
                                val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]);
                            }
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
                {
                    if (techIndex == techIndex2)
                    {
                        continue;
                    }
                    for (int yIndex = -1; yIndex < (int)techTripList[techIndex2].size(); yIndex++)
                    {
                        std::vector<int> tmp1, tmp2;
                        if (xIndex >= 0)
                        {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex].begin(),
                                        techTripList[techIndex].begin() + xIndex + 1);
                        }

                        if (yIndex >= 0)
                        {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex2].begin(),
                                        techTripList[techIndex2].begin() + yIndex + 1);
                        }

                        if (yIndex < (int)techTripList[techIndex2].size())
                        {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex2].begin() + yIndex + 1,
                                        techTripList[techIndex2].end());
                        }

                        if (xIndex < (int)techTripList[techIndex].size())
                        {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex].begin() + xIndex + 1,
                                        techTripList[techIndex].end());
                        }
                        Solution s = *this;

                        s.techTripList[techIndex] = tmp1;
                        s.techTripList[techIndex2] = tmp2;

                        double newScore = s.getScore();
                        std::string val1 = "0";
                        if (xIndex >= 0)
                        {
                            val1 = std::to_string(techTripList[techIndex][xIndex]);
                        }
                        std::string val2 = "0";
                        if (yIndex >= 0)
                        {
                            val2 = std::to_string(techTripList[techIndex2][yIndex]);
                        }
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }
    }

    if (type != INTER)
    {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = -1; xIndex < (int)droneTripList[droneIndex][tripIndex].size(); xIndex++)
                {
                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                    {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                        {
                            if (droneIndex == droneIndex2 && tripIndex == tripIndex2)
                            {
                                continue;
                            }
                            for (int yIndex = -1;
                                 yIndex < (int)droneTripList[droneIndex2][tripIndex2].size(); yIndex++)
                            {
                                std::vector<int> tmp1, tmp2;
                                if (xIndex >= 0)
                                {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex][tripIndex].begin(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1);
                                }

                                if (yIndex >= 0)
                                {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1);
                                }

                                if (yIndex < (int)droneTripList[droneIndex2][tripIndex2].size())
                                {
                                    tmp1.insert(tmp1.end(),
                                                droneTripList[droneIndex2][tripIndex2].begin() + yIndex + 1,
                                                droneTripList[droneIndex2][tripIndex2].end());
                                }

                                if (xIndex < (int)droneTripList[droneIndex][tripIndex].size())
                                {
                                    tmp2.insert(tmp2.end(),
                                                droneTripList[droneIndex][tripIndex].begin() + xIndex + 1,
                                                droneTripList[droneIndex][tripIndex].end());
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex][tripIndex] = tmp1;
                                s.droneTripList[droneIndex2][tripIndex2] = tmp2;

                                double newScore = s.getScore();
                                std::string val1 = "0";
                                if (xIndex >= 0)
                                {
                                    val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]);
                                }
                                std::string val2 = "0";
                                if (yIndex >= 0)
                                {
                                    val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]);
                                }
                                //Method 2:
                                //std::cout << std::endl << (!isImproved) << " and " << (newScore) << " -- " << curScore << " and " << checkTabuCondition(tabuList, val1, val2);
                                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                    isImproved = true;
                                    bestFeasibleSolution.droneTripList = s.droneTripList;
                                    bestFeasibleSolution.techTripList = s.techTripList;
                                    bestScore = newScore;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                 
                                } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                                {
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                                /*if (newScore < curScore)
                                {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon))
                                    {
                                        isImproved = true;
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                }*/
                            }
                        }
                    }
                }
            }
        }

        // tech
        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = -1; xIndex < (int)techTripList[techIndex].size(); xIndex++)
            {
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
                {
                    if (techIndex == techIndex2)
                    {
                        continue;
                    }
                    for (int yIndex = -1; yIndex < (int)techTripList[techIndex2].size(); yIndex++)
                    {
                        std::vector<int> tmp1, tmp2;
                        if (xIndex >= 0)
                        {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex].begin(),
                                        techTripList[techIndex].begin() + xIndex + 1);
                        }

                        if (yIndex >= 0)
                        {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex2].begin(),
                                        techTripList[techIndex2].begin() + yIndex + 1);
                        }

                        if (yIndex < (int)techTripList[techIndex2].size())
                        {
                            tmp1.insert(tmp1.end(),
                                        techTripList[techIndex2].begin() + yIndex + 1,
                                        techTripList[techIndex2].end());
                        }

                        if (xIndex < (int)techTripList[techIndex].size())
                        {
                            tmp2.insert(tmp2.end(),
                                        techTripList[techIndex].begin() + xIndex + 1,
                                        techTripList[techIndex].end());
                        }
                        Solution s = *this;

                        s.techTripList[techIndex] = tmp1;
                        s.techTripList[techIndex2] = tmp2;

                        double newScore = s.getScore();
                        std::string val1 = "0";
                        if (xIndex >= 0)
                        {
                            val1 = std::to_string(techTripList[techIndex][xIndex]);
                        }
                        std::string val2 = "0";
                        if (yIndex >= 0)
                        {
                            val2 = std::to_string(techTripList[techIndex2][yIndex]);
                        }
                        //Method 2:
                        //std::cout << std::endl << (!isImproved) << " and " << (newScore) << " -- " << curScore << " and " << checkTabuCondition(tabuList, val1, val2);

                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }
    }

    if (bestSolution->getScore() < std::abs(config.tabuEpsilon)){
        return this;
    }
    return bestSolution;
}

Solution *Solution::crossExchange(const std::vector<std::string> &tabuList, Solution &bestFeasibleSolution, RouteType type, int dis1,
                                  int dis2)
{
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double bestScore = bestFeasibleSolution.getScore();
    double curScore = std::numeric_limits<double>::max();
    double baseScore = this->getScore();
    bool isImproved = false;
    if (type != INTRA)
    {
        // drone

        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = 0; xIndex < (int)droneTripList[droneIndex][tripIndex].size() - dis1; xIndex++)
                {
                    // tech
                    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
                    {
                        for (int yIndex = 0; yIndex < (int)techTripList[techIndex].size() - dis2; yIndex++)
                        {
                            bool canMoveToDroneTrip = true;
                            for (int i = yIndex; i <= yIndex + dis2; i++)
                            {
                                if (input.cusOnlyServedByTech[techTripList[techIndex][i]])
                                {
                                    canMoveToDroneTrip = false;
                                    break;
                                }
                            }
                            if (!canMoveToDroneTrip)
                            {
                                continue;
                            }

                            Solution s = *this;

                            s.techTripList[techIndex].insert(
                                s.techTripList[techIndex].begin() + yIndex + dis2 + 1,
                                droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1);
                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1,
                                techTripList[techIndex].begin() + yIndex,
                                techTripList[techIndex].begin() + yIndex + dis2 + 1);

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + yIndex,
                                s.techTripList[techIndex].begin() + yIndex + dis2 + 1);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1);

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]) + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(techTripList[techIndex][yIndex]) + "-" + std::to_string(techTripList[techIndex][yIndex + dis2]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }

                    // drone
                    for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                    {
                        for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                        {
                            if (droneIndex == droneIndex2 && tripIndex == tripIndex2)
                            {
                                continue;
                            }
                            for (int yIndex = 0;
                                 yIndex < (int)droneTripList[droneIndex2][tripIndex2].size() - dis2; yIndex++)
                            {
                                if (droneIndex == droneIndex2 && tripIndex == tripIndex2)
                                {
                                    continue;
                                }
                                Solution s = *this;

                                s.droneTripList[droneIndex2][tripIndex2].insert(
                                    s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + dis2 + 1,
                                    droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                    droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1);
                                s.droneTripList[droneIndex][tripIndex].insert(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1,
                                    droneTripList[droneIndex2][tripIndex2].begin() + yIndex,
                                    droneTripList[droneIndex2][tripIndex2].begin() + yIndex + dis2 + 1);

                                s.droneTripList[droneIndex2][tripIndex2].erase(
                                    s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex,
                                    s.droneTripList[droneIndex2][tripIndex2].begin() + yIndex + dis2 + 1);

                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex,
                                    s.droneTripList[droneIndex][tripIndex].begin() + xIndex + dis1 + 1);

                                double newScore = s.getScore();

                                std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]) + "-" +
                                                   std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                                std::string val2 = std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex]) + "-" + std::to_string(droneTripList[droneIndex2][tripIndex2][yIndex + dis2]);
                                //Method 2:
                                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                    isImproved = true;
                                    bestFeasibleSolution.droneTripList = s.droneTripList;
                                    bestFeasibleSolution.techTripList = s.techTripList;
                                    bestScore = newScore;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                    
                                } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                                {
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                                /*if (newScore < curScore)
                                {
                                    if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                                 abs(newScore - baseScore) > config.tabuEpsilon))
                                    {
                                        isImproved = true;
                                        bestSolution->droneTripList = s.droneTripList;
                                        bestSolution->techTripList = s.techTripList;
                                        bestSolution->ext["state"] = "";
                                        bestSolution->ext["state"] += val1;
                                        bestSolution->ext["state"] += "-";
                                        bestSolution->ext["state"] += val2;
                                        curScore = newScore;
                                    }
                                }*/
                            }
                        }
                    }
                }
            }
        }

        // tech
        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = 0; xIndex < (int)techTripList[techIndex].size() - dis1; xIndex++)
            {

                // drone
                for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
                {
                    for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
                    {
                        for (int yIndex = 0;
                             yIndex < (int)droneTripList[droneIndex][tripIndex].size() - dis2; yIndex++)
                        {
                            bool canMoveToDroneTrip = true;
                            for (int i = xIndex; i <= xIndex + dis1; i++)
                            {
                                if (input.cusOnlyServedByTech[techTripList[techIndex][i]])
                                {
                                    canMoveToDroneTrip = false;
                                    break;
                                }
                            }
                            if (!canMoveToDroneTrip)
                            {
                                continue;
                            }
                            Solution s = *this;

                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1,
                                techTripList[techIndex].begin() + xIndex,
                                techTripList[techIndex].begin() + xIndex + dis1 + 1);
                            s.techTripList[techIndex].insert(
                                s.techTripList[techIndex].begin() + xIndex + dis1 + 1,
                                droneTripList[droneIndex][tripIndex].begin() + yIndex,
                                droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1);

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin() + yIndex,
                                s.droneTripList[droneIndex][tripIndex].begin() + yIndex + dis2 + 1);

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + xIndex,
                                s.techTripList[techIndex].begin() + xIndex + dis1 + 1);

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(techTripList[techIndex][xIndex]) + "-" + std::to_string(techTripList[techIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]) + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][yIndex + dis2]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }

                // tech
                for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
                {
                    for (int yIndex = 0; yIndex < (int)techTripList[techIndex2].size() - dis2; yIndex++)
                    {
                        if (techIndex == techIndex2)
                        {
                            continue;
                        }
                        Solution s = *this;

                        s.techTripList[techIndex2].insert(
                            s.techTripList[techIndex2].begin() + yIndex + dis2 + 1,
                            techTripList[techIndex].begin() + xIndex,
                            techTripList[techIndex].begin() + xIndex + dis1 + 1);
                        s.techTripList[techIndex].insert(
                            s.techTripList[techIndex].begin() + xIndex + dis1 + 1,
                            techTripList[techIndex2].begin() + yIndex,
                            techTripList[techIndex2].begin() + yIndex + dis2 + 1);

                        s.techTripList[techIndex2].erase(
                            s.techTripList[techIndex2].begin() + yIndex,
                            s.techTripList[techIndex2].begin() + yIndex + dis2 + 1);

                        s.techTripList[techIndex].erase(
                            s.techTripList[techIndex].begin() + xIndex,
                            s.techTripList[techIndex].begin() + xIndex + dis1 + 1);

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex]) + "-" + std::to_string(techTripList[techIndex][xIndex + dis1]);
                        std::string val2 = std::to_string(techTripList[techIndex2][yIndex]) + "-" +
                                           std::to_string(techTripList[techIndex2][yIndex + dis2]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        }
                        if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }
    }

    if (type != INTER)
    {
        // drone
        for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int xIndex = 0; xIndex < (int)droneTripList[droneIndex][tripIndex].size() - dis1; xIndex++)
                {
                    for (int yIndex = 0; yIndex < (int)droneTripList[droneIndex][tripIndex].size() - dis2; yIndex++)
                    {
                        if (xIndex == yIndex)
                        {
                            continue;
                        }
                        if (xIndex + dis1 < yIndex || xIndex > yIndex + dis2)
                        {
                            Solution s = *this;
                            std::vector<int> tmp;

                            int beg1, end1, beg2, end2;
                            if (xIndex < yIndex)
                            {
                                beg1 = xIndex;
                                end1 = xIndex + dis1 + 1;
                                beg2 = yIndex;
                                end2 = yIndex + dis2 + 1;
                            }
                            else
                            {
                                beg2 = xIndex;
                                end2 = xIndex + dis1 + 1;
                                beg1 = yIndex;
                                end1 = yIndex + dis2 + 1;
                            }

                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin(),
                                       droneTripList[droneIndex][tripIndex].begin() + beg1);
                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + beg2,
                                       droneTripList[droneIndex][tripIndex].begin() + end2);
                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + end1,
                                       droneTripList[droneIndex][tripIndex].begin() + beg2);
                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + beg1,
                                       droneTripList[droneIndex][tripIndex].begin() + end1);

                            tmp.insert(tmp.end(),
                                       droneTripList[droneIndex][tripIndex].begin() + end2,
                                       droneTripList[droneIndex][tripIndex].end());

                            s.droneTripList[droneIndex][tripIndex].erase(
                                s.droneTripList[droneIndex][tripIndex].begin(),
                                s.droneTripList[droneIndex][tripIndex].end());
                            s.droneTripList[droneIndex][tripIndex].insert(
                                s.droneTripList[droneIndex][tripIndex].begin(),
                                tmp.begin(), tmp.end());

                            double newScore = s.getScore();

                            std::string val1 = std::to_string(droneTripList[droneIndex][tripIndex][xIndex]) + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][xIndex + dis1]);
                            std::string val2 = std::to_string(droneTripList[droneIndex][tripIndex][yIndex]) + "-" +
                                               std::to_string(droneTripList[droneIndex][tripIndex][yIndex + dis2]);
                            //Method 2:
                            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                                isImproved = true;
                                bestFeasibleSolution.droneTripList = s.droneTripList;
                                bestFeasibleSolution.techTripList = s.techTripList;
                                bestScore = newScore;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                                
                            } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                            {
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                            /*if (newScore < curScore)
                            {
                                if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                             abs(newScore - baseScore) > config.tabuEpsilon))
                                {
                                    isImproved = true;
                                    bestSolution->droneTripList = s.droneTripList;
                                    bestSolution->techTripList = s.techTripList;
                                    bestSolution->ext["state"] = "";
                                    bestSolution->ext["state"] += val1;
                                    bestSolution->ext["state"] += "-";
                                    bestSolution->ext["state"] += val2;
                                    curScore = newScore;
                                }
                            }*/
                        }
                    }
                }
            }
        }

        // tech

        for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
        {
            for (int xIndex = 0; xIndex < (int)techTripList[techIndex].size() - dis1; xIndex++)
            {
                for (int yIndex = 0; yIndex < (int)techTripList[techIndex].size() - dis2; yIndex++)
                {
                    if (xIndex == yIndex)
                    {
                        continue;
                    }
                    if (xIndex + dis1 < yIndex || xIndex > yIndex + dis2)
                    {
                        Solution s = *this;

                        std::vector<int> tmp;

                        int beg1, end1, beg2, end2;
                        if (xIndex < yIndex)
                        {
                            beg1 = xIndex;
                            end1 = xIndex + dis1 + 1;
                            beg2 = yIndex;
                            end2 = yIndex + dis2 + 1;
                        }
                        else
                        {
                            beg2 = xIndex;
                            end2 = xIndex + dis1 + 1;
                            beg1 = yIndex;
                            end1 = yIndex + dis2 + 1;
                        }

                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin(),
                                   techTripList[techIndex].begin() + beg1);
                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + beg2,
                                   techTripList[techIndex].begin() + end2);
                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + end1,
                                   techTripList[techIndex].begin() + beg2);
                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + beg1,
                                   techTripList[techIndex].begin() + end1);

                        tmp.insert(tmp.end(),
                                   techTripList[techIndex].begin() + end2,
                                   techTripList[techIndex].end());

                        s.techTripList[techIndex].erase(s.techTripList[techIndex].begin(),
                                                        s.techTripList[techIndex].end());
                        s.techTripList[techIndex].insert(s.techTripList[techIndex].begin(),
                                                         tmp.begin(), tmp.end());

                        double newScore = s.getScore();

                        std::string val1 = std::to_string(techTripList[techIndex][xIndex]) + "-" + std::to_string(techTripList[techIndex][xIndex + dis1]);
                        std::string val2 = std::to_string(techTripList[techIndex][yIndex]) + "-" +
                                           std::to_string(techTripList[techIndex][yIndex + dis2]);
                        //Method 2:
                        if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                            isImproved = true;
                            bestFeasibleSolution.droneTripList = s.droneTripList;
                            bestFeasibleSolution.techTripList = s.techTripList;
                            bestScore = newScore;
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                            
                        } else if (!isImproved && (newScore - curScore < config.tabuEpsilon) && (checkTabuCondition(tabuList, val1, val2)))
                        {
                            bestSolution->droneTripList = s.droneTripList;
                            bestSolution->techTripList = s.techTripList;
                            bestSolution->ext["state"] = "";
                            bestSolution->ext["state"] += val1;
                            bestSolution->ext["state"] += "-";
                            bestSolution->ext["state"] += val2;
                            curScore = newScore;
                        }
                        /*if (newScore < curScore)
                        {
                            if (newScore < bestScore || (checkTabuCondition(tabuList, val1, val2) &&
                                                         abs(newScore - baseScore) > config.tabuEpsilon))
                            {
                                isImproved = true;
                                bestSolution->droneTripList = s.droneTripList;
                                bestSolution->techTripList = s.techTripList;
                                bestSolution->ext["state"] = "";
                                bestSolution->ext["state"] += val1;
                                bestSolution->ext["state"] += "-";
                                bestSolution->ext["state"] += val2;
                                curScore = newScore;
                            }
                        }*/
                    }
                }
            }
        }
    }

    if (bestSolution->getScore() < std::abs(config.tabuEpsilon)){
        return this;
    }
    return bestSolution;  
}


Solution *Solution::ejectionNeighborhoodAdd(Solution &bestFeasibleSolution){
    auto *bestSolution = new Solution(config, input, alpha1, alpha2);
    double bestScore = bestFeasibleSolution.getScore();
    double curScore = std::numeric_limits<double>::max();
    std::vector<std::pair<std::vector<int>, std::vector<int>>> bestShiftSequence;
    std::vector<std::pair<std::vector<int>, std::vector<int>>> shiftSequence;
    bool isImproved = false;
    bool isImproved2 = false;
    Solution s = *this;
    double bestGain = 0;
    double currentGain = 0;
    int currentLevel = 0;
    std::vector<int> customerX;
    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
    {
        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
        {
            for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++)
            {
                ejection(s,
                         {droneIndex, tripIndex, xIndex},
                         DRONE,
                         customerX,
                         currentGain,
                         bestGain,
                         currentLevel,
                         shiftSequence,
                         bestShiftSequence);
                double newScore = s.getScore();
                if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                    bestFeasibleSolution.droneTripList = s.droneTripList;
                    bestFeasibleSolution.techTripList = s.techTripList;
                    bestScore = newScore;
                }
            }
        }
    }

    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
    {
        for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++)
        {
            ejection(s,
                     {techIndex, xIndex},
                     TECHNICIAN,
                     customerX,
                     currentGain,
                     bestGain,
                     currentLevel,
                     shiftSequence,
                     bestShiftSequence);
            double newScore = s.getScore();
            if (s.check_feasible() && (newScore - bestScore < config.tabuEpsilon)){
                bestFeasibleSolution.droneTripList = s.droneTripList;
                bestFeasibleSolution.techTripList = s.techTripList;
                bestScore = newScore;
            }
        }
    }


    for (std::pair<std::vector<int>, std::vector<int>> move : bestShiftSequence)
    {
        int x;
        if (move.first.size() == 2)
        {
            // delete x in Tech
            x = s.techTripList[move.first[0]][move.first[1]];
            s.techTripList[move.first[0]].erase(s.techTripList[move.first[0]].begin() + move.first[1]);
        }
        else
        {
            // delete x in Drone
            x = s.droneTripList[move.first[0]][move.first[1]][move.first[2]];
            s.droneTripList[move.first[0]][move.first[1]].erase(
                s.droneTripList[move.first[0]][move.first[1]].begin() + move.first[2]);
        }

        if (move.second.size() == 2)
        {
            // insert x in Tech
            s.techTripList[move.second[0]].insert(s.techTripList[move.second[0]].begin() + move.second[1], x);
        }
        else
        {
            // insert x in Drone
            s.droneTripList[move.second[0]][move.second[1]].insert(
                s.droneTripList[move.second[0]][move.second[1]].begin() + move.second[2], x);
        }
    }

    if ((bestSolution->getScore() - bestScore < config.tabuEpsilon) and (bestSolution->check_feasible())){
        bestFeasibleSolution.droneTripList = s.droneTripList;
        bestFeasibleSolution.techTripList = s.techTripList;
    }
    bestSolution->droneTripList = bestFeasibleSolution.droneTripList;
    bestSolution->techTripList = bestFeasibleSolution.techTripList;
    //    std::cout<< "-gain:" << bestGain<<std::endl;
    if (bestSolution->getScore() < std::abs(config.tabuEpsilon)){
        return this;
    }
    return bestSolution;  
}


std::string Solution::toString()
{
    json jDrone(droneTripList);
    json jTech(techTripList);
    return jDrone.dump() + "::" + jTech.dump();
}

std::vector<double> Solution::getScoreATrip(int tripIndex, TripType type)
{
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    double tmp, tmp1;
    double ct, dzt = 0, czt = 0;
    double allTechTime = 0, allDroneTime = 0;

    if (type == DRONE)
    {
        double droneCompleteTime = 0;
        int maxTrip = 0;
        for (auto &i : droneTripList)
        {
            if (maxTrip < i.size())
            {
                maxTrip = (int)i.size();
            }
        }
        std::vector<double> droneTripCompleteTime(maxTrip, 0);
        tmp = 0;
        for (int j = 0; j < droneTripList[tripIndex].size(); j++)
        {
            if (droneTripList[tripIndex][j].empty())
            {
                continue;
            }

            tmp1 = input.droneTimes[0][droneTripList[tripIndex][j][0]];
            cusCompleteTime[droneTripList[tripIndex][j][0]] = tmp1;

            for (int k = 0; k < (int)droneTripList[tripIndex][j].size() - 1; k++)
            {
                tmp1 += input.droneTimes[droneTripList[tripIndex][j][k]][droneTripList[tripIndex][j][k + 1]];
                cusCompleteTime[droneTripList[tripIndex][j][k + 1]] = tmp1;
            }
            droneTripCompleteTime[j] = tmp1 + input.droneTimes[droneTripList[tripIndex][j].back()][0];
            tmp += droneTripCompleteTime[j];
        }
        droneCompleteTime = tmp;
        if (tmp > allDroneTime)
        {
            allDroneTime = tmp;
        }

        for (int j = 0; j < droneTripList[tripIndex].size(); j++)
        {
            if (droneTripList[tripIndex][j].empty())
            {
                continue;
            }
            for (int k : droneTripList[tripIndex][j])
            {
                czt += std::max(0.,
                                droneTripCompleteTime[j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
            }
            dzt += std::max(0., droneTripCompleteTime[j] - config.droneLimitationFlightTime);
        }
    }
    else
    {
        std::vector<double> techCompleteTime(config.numTech, 0);
        if (!techTripList[tripIndex].empty())
        {
            tmp = input.techTimes[0][techTripList[tripIndex][0]];

            cusCompleteTime[techTripList[tripIndex][0]] = tmp;

            for (int j = 0; j < (int)techTripList[tripIndex].size() - 1; j++)
            {
                tmp += input.techTimes[techTripList[tripIndex][j]][techTripList[tripIndex][j + 1]];
                cusCompleteTime[techTripList[tripIndex][j + 1]] = tmp;
            }

            techCompleteTime[tripIndex] = tmp + input.techTimes[techTripList[tripIndex].back()][0];
            if (techCompleteTime[tripIndex] > allTechTime)
            {
                allTechTime = techCompleteTime[tripIndex];
            }

            for (int j : techTripList[tripIndex])
            {
                cz += std::max(0.,
                               techCompleteTime[tripIndex] - cusCompleteTime[j] - config.sampleLimitationWaitingTime);
            }
        }
    }
    ct = std::max(allDroneTime, allTechTime);
    return {ct, dzt, czt};
}

void Solution::ejection(Solution &solution, std::vector<int> xIndex, TripType type, std::vector<int> &customerX, double gain, double &bestGain,
                        int &level,
                        std::vector<std::pair<std::vector<int>, std::vector<int>>> &shiftSequence,
                        std::vector<std::pair<std::vector<int>, std::vector<int>>> &bestShiftSequence)
{
    if (level >= config.maxEjectionLevel) {return;}
    Solution bestSol = solution;
    std::vector<double> droneScores;
    double maxDroneScore = 0;
    droneScores.reserve(solution.droneTripList.size());
    int maxDroneIndex;
    for (int droneIndex = 0; droneIndex < solution.droneTripList.size(); droneIndex++)
    {
        double sc = solution.getScoreATrip(droneIndex, DRONE)[0];
        droneScores.push_back(sc);
        if (sc > maxDroneScore)
        {
            maxDroneScore = sc;
            maxDroneIndex = droneIndex;
        }
    }

    if (type == DRONE && xIndex[0] != maxDroneIndex)
    {
        return;
    }

    std::vector<double> techScores;
    double maxTechScore = 0;
    techScores.reserve(solution.techTripList.size());
    int maxTechIndex;
    for (int techIndex = 0; techIndex < solution.techTripList.size(); techIndex++)
    {
        double sc = solution.getScoreATrip(techIndex, TECHNICIAN)[0];
        techScores.push_back(sc);
        if (sc > maxTechScore)
        {
            maxTechScore = sc;
            maxTechIndex = techIndex;
        }
    }

    if (type == TECHNICIAN && xIndex[0] != maxTechIndex)
    {
        return;
    }
    if (maxDroneScore > maxTechScore && type == TECHNICIAN)
    {
        return;
    }
    else if (maxTechScore > maxDroneScore && type == DRONE)
    {
        return;
    }
    double fScore = std::max(maxDroneScore, maxTechScore);

    int x, predecessor, successor;
    std::vector<std::vector<double>> times;
    if (type == DRONE)
    {
        times = input.droneTimes;
        x = solution.droneTripList[xIndex[0]][xIndex[1]][xIndex[2]];
        customerX.push_back(x);
        predecessor = (xIndex[2] == 0) ? 0 : solution.droneTripList[xIndex[0]][xIndex[1]][xIndex[2] - 1];
        successor = (xIndex[2] == solution.droneTripList[xIndex[0]][xIndex[1]].size() - 1)
                        ? input.numCus + 1
                        : solution.droneTripList[xIndex[0]][xIndex[1]][xIndex[2] + 1];
        droneScores[xIndex[0]] -= times[predecessor][x] + times[x][successor] - times[predecessor][successor];

        double cScore = std::max(maxTechScore, *std::max_element(droneScores.begin(), droneScores.end()));
        double g = fScore - cScore;
        solution.droneTripList[xIndex[0]][xIndex[1]].erase(
            solution.droneTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2]);
        gain += g;

        for (int droneIndex = 0; droneIndex < solution.droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < solution.droneTripList[droneIndex].size(); tripIndex++)
            {
                if (droneIndex == xIndex[0] && tripIndex == xIndex[1])
                {
                    continue;
                }

                for (int cusIndex = 0; cusIndex < solution.droneTripList[droneIndex][tripIndex].size(); cusIndex++)
                {
                    int cus = solution.droneTripList[droneIndex][tripIndex][cusIndex];
                    int cusPredecessor = (cusIndex == 0) ? 0 : solution.droneTripList[droneIndex][tripIndex][cusIndex - 1];
                    double delta = input.droneTimes[cusPredecessor][x] + input.droneTimes[x][cus] - input.droneTimes[cusPredecessor][cus];
                    double d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                    if (gain - d > bestGain)
                    {
                        solution.droneTripList[droneIndex][tripIndex].insert(
                            solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                        if (tmp[1] == 0 && tmp[2] == 0)
                        {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                            bestSol.droneTripList = solution.droneTripList;
                            bestSol.techTripList = solution.techTripList;
                        }
                        else if (level + 1 <= config.maxEjectionLevel)
                        {
                            for (int yIndex = 0;
                                 yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++)
                            {
                                Solution s = solution;

                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(droneIndex, DRONE);
                                if (tmp[1] == 0 && tmp[2] == 0)
                                {
                                    ejection(solution, {droneIndex, tripIndex, yIndex},
                                             DRONE,
                                             customerX,
                                             gain,
                                             bestGain,
                                             level,
                                             shiftSequence,
                                             bestShiftSequence);
                                }
                            }
                        }
                        bool flag = false;
                        for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                                if (flag == true){
                                    break;
                                }
                                for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                    if (*it == x) {
                                        flag = true;
                                        it = solution.droneTripList[xx][yy].erase(it);  
                                        break;
                                    } else {
                                        ++it;
                                    }
                                }
                            }
                        }
                        for (int xx = 0; xx < solution.techTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                                if (*it == x) {
                                    flag = true;
                                    it = solution.techTripList[xx].erase(it);  
                                    break;
                                } else {
                                    ++it;
                                }
                            }
                        }

                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }

                    if (cusIndex == solution.droneTripList[droneIndex][tripIndex].size() - 1)
                    {
                        delta = input.droneTimes[cus][x] + input.droneTimes[x][input.numCus + 1] - input.droneTimes[cus][input.numCus + 1];
                        d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                        if (gain - d > bestGain)
                        {
                            solution.droneTripList[droneIndex][tripIndex].insert(
                                solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex + 1, x);
                            gain -= d;

                            shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex + 1}});
                            level++;
                            std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                            if (tmp[1] == 0 && tmp[2] == 0)
                            {
                                bestShiftSequence = shiftSequence;
                                bestGain = gain;
                            }
                            else if (level + 1 <= config.maxEjectionLevel)
                            {
                                for (int yIndex = 0;
                                     yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++)
                                {
                                    Solution s = solution;

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                    tmp = s.getScoreATrip(droneIndex, DRONE);
                                    if (tmp[1] == 0 && tmp[2] == 0)
                                    {
                                        ejection(solution, {droneIndex, tripIndex, yIndex},
                                                 DRONE,
                                                 customerX,
                                                 gain,
                                                 bestGain,
                                                 level,
                                                 shiftSequence,
                                                 bestShiftSequence);
                                    }
                                }
                            }
                            bool flag = false;
                            for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                                if (flag == true){
                                    break;
                                }
                                for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                                    if (flag == true){
                                        break;
                                    }
                                    for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                        if (*it == x) {
                                            flag = true;
                                            it = solution.droneTripList[xx][yy].erase(it);  
                                            break;
                                        } else {
                                            ++it;
                                        }
                                    }
                                }
                            }
                            for (int xx = 0; xx < solution.techTripList.size(); xx++){
                                if (flag == true){
                                    break;
                                }
                                for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                                    if (*it == x) {
                                        flag = true;
                                        it = solution.techTripList[xx].erase(it);  
                                        break;
                                    } else {
                                        ++it;
                                    }
                                }
                            }

                            gain += d;
                            shiftSequence.pop_back();
                            level--;
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < solution.techTripList.size(); techIndex++)
        {
            for (int cusIndex = 0; cusIndex < solution.techTripList[techIndex].size(); cusIndex++)
            {
                int cus = solution.techTripList[techIndex][cusIndex];
                int cusPredecessor = (cusIndex == 0) ? 0 : solution.techTripList[techIndex][cusIndex - 1];
                double delta = input.techTimes[cusPredecessor][x] + input.techTimes[x][cus] - input.techTimes[cusPredecessor][cus];
                double d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                if (gain - d > bestGain)
                {
                    solution.techTripList[techIndex].insert(
                        solution.techTripList[techIndex].begin() + cusIndex, x);
                    gain -= d;

                    shiftSequence.push_back({xIndex, {techIndex, cusIndex}});
                    level++;
                    std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                    if (tmp[1] == 0 && tmp[2] == 0)
                    {
                        bestShiftSequence = shiftSequence;
                        bestGain = gain;
                    }
                    else if (level + 1 <= config.maxEjectionLevel)
                    {
                        for (int yIndex = 0;
                             yIndex < solution.techTripList[techIndex].size(); yIndex++)
                        {
                            Solution s = solution;

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + yIndex);
                            tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                            if (tmp[1] == 0 && tmp[2] == 0)
                            {
                                ejection(solution, {techIndex, yIndex},
                                         TECHNICIAN,
                                         customerX,
                                         gain,
                                         bestGain,
                                         level,
                                         shiftSequence,
                                         bestShiftSequence);
                            }
                        }
                    }
                    bool flag = false;
                    for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                        if (flag == true){
                            break;
                        }
                        for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                            if (flag == true){
                                break;
                            }
                            for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                if (*it == x) {
                                    flag = true;
                                    it = solution.droneTripList[xx][yy].erase(it);  
                                    break;
                                } else {
                                    ++it;
                                }
                            }
                        }
                    }
                    for (int xx = 0; xx < solution.techTripList.size(); xx++){
                        if (flag == true){
                            break;
                        }
                        for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                            if (*it == x) {
                                flag = true;
                                it = solution.techTripList[xx].erase(it);  
                                break;
                            } else {
                                ++it;
                            }
                        }
                    }
                    gain += d;
                    shiftSequence.pop_back();
                    level--;
                }

                if (cusIndex == solution.techTripList[techIndex].size() - 1)
                {
                    delta = input.techTimes[cus][x] + input.techTimes[x][input.numCus + 1] - input.techTimes[cus][input.numCus + 1];
                    d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                    if (gain - d > bestGain)
                    {
                        solution.techTripList[techIndex].insert(
                            solution.techTripList[techIndex].begin() + cusIndex + 1, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {techIndex, cusIndex + 1}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                        if (tmp[1] == 0 && tmp[2] == 0)
                        {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        }
                        else if (level + 1 <= config.maxEjectionLevel)
                        {
                            for (int yIndex = 0;
                                 yIndex < solution.techTripList[techIndex].size(); yIndex++)
                            {
                                Solution s = solution;

                                s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                                if (tmp[1] == 0 && tmp[2] == 0)
                                {
                                    ejection(solution, {techIndex, yIndex},
                                             TECHNICIAN,
                                             customerX,
                                             gain,
                                             bestGain,
                                             level,
                                             shiftSequence,
                                             bestShiftSequence);
                                }
                            }
                        }

                        bool flag = false;
                        for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                                if (flag == true){
                                    break;
                                }
                                for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                    if (*it == x) {
                                        flag = true;
                                        it = solution.droneTripList[xx][yy].erase(it);  
                                        break;
                                    } else {
                                        ++it;
                                    }
                                }
                            }
                        }
                        for (int xx = 0; xx < solution.techTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                                if (*it == x) {
                                    flag = true;
                                    it = solution.techTripList[xx].erase(it);  
                                    break;
                                } else {
                                    ++it;
                                }
                            }
                        }
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }
                }
            }
        }

        solution.droneTripList[xIndex[0]][xIndex[1]].insert(
            solution.droneTripList[xIndex[0]][xIndex[1]].begin() + xIndex[2], x);
    }
    else
    {
        times = input.techTimes;
        x = solution.techTripList[xIndex[0]][xIndex[1]];
        customerX.push_back(x);
        predecessor = (xIndex[1] == 0) ? 0 : solution.techTripList[xIndex[0]][xIndex[1] - 1];
        successor = (xIndex[1] == solution.techTripList[xIndex[0]].size() - 1)
                        ? input.numCus + 1
                        : solution.techTripList[xIndex[0]][xIndex[1] + 1];
        techScores[xIndex[0]] -= times[predecessor][x] + times[x][successor] - times[predecessor][successor];

        double cScore = std::max(maxDroneScore, *std::max_element(techScores.begin(), techScores.end()));
        double g = fScore - cScore;
        solution.techTripList[xIndex[0]].erase(
            solution.techTripList[xIndex[0]].begin() + xIndex[1]);
        gain += g;

        for (int droneIndex = 0; droneIndex < solution.droneTripList.size(); droneIndex++)
        {
            for (int tripIndex = 0; tripIndex < solution.droneTripList[droneIndex].size(); tripIndex++)
            {
                for (int cusIndex = 0; cusIndex < solution.droneTripList[droneIndex][tripIndex].size(); cusIndex++)
                {
                    int cus = solution.droneTripList[droneIndex][tripIndex][cusIndex];
                    int cusPredecessor = (cusIndex == 0) ? 0 : solution.droneTripList[droneIndex][tripIndex][cusIndex - 1];
                    double delta = input.droneTimes[cusPredecessor][x] + input.droneTimes[x][cus] - input.droneTimes[cusPredecessor][cus];
                    double d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                    if (gain - d > bestGain)
                    {
                        solution.droneTripList[droneIndex][tripIndex].insert(
                            solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                        if (tmp[1] == 0 && tmp[2] == 0)
                        {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        }
                        else if (level + 1 <= config.maxEjectionLevel)
                        {
                            for (int yIndex = 0;
                                 yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++)
                            {
                                Solution s = solution;

                                s.droneTripList[droneIndex][tripIndex].erase(
                                    s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(droneIndex, DRONE);
                                if (tmp[1] == 0 && tmp[2] == 0)
                                {
                                    ejection(solution, {droneIndex, tripIndex, yIndex},
                                             DRONE,
                                             customerX,
                                             gain,
                                             bestGain,
                                             level,
                                             shiftSequence,
                                             bestShiftSequence);
                                }
                            }
                        }

                        bool flag = false;
                        for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                                if (flag == true){
                                    break;
                                }
                                for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                    if (*it == x) {
                                        flag = true;
                                        it = solution.droneTripList[xx][yy].erase(it);  
                                        break;
                                    } else {
                                        ++it;
                                    }
                                }
                            }
                        }
                        for (int xx = 0; xx < solution.techTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                                if (*it == x) {
                                    flag = true;
                                    it = solution.techTripList[xx].erase(it);  
                                    break;
                                } else {
                                    ++it;
                                }
                            }
                        }
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }

                    if (cusIndex == solution.droneTripList[droneIndex][tripIndex].size() - 1)
                    {
                        delta = input.droneTimes[cus][x] + input.droneTimes[x][input.numCus + 1] - input.droneTimes[cus][input.numCus + 1];
                        d = std::max(cScore, droneScores[droneIndex] + delta) - cScore;

                        if (gain - d > bestGain)
                        {
                            solution.droneTripList[droneIndex][tripIndex].insert(
                                solution.droneTripList[droneIndex][tripIndex].begin() + cusIndex + 1, x);
                            gain -= d;

                            shiftSequence.push_back({xIndex, {droneIndex, tripIndex, cusIndex + 1}});
                            level++;
                            std::vector<double> tmp = solution.getScoreATrip(droneIndex, DRONE);
                            if (tmp[1] == 0 && tmp[2] == 0)
                            {
                                bestShiftSequence = shiftSequence;
                                bestGain = gain;
                            }
                            else if (level + 1 <= config.maxEjectionLevel)
                            {
                                for (int yIndex = 0;
                                     yIndex < solution.droneTripList[droneIndex][tripIndex].size(); yIndex++)
                                {
                                    Solution s = solution;

                                    s.droneTripList[droneIndex][tripIndex].erase(
                                        s.droneTripList[droneIndex][tripIndex].begin() + yIndex);
                                    tmp = s.getScoreATrip(droneIndex, DRONE);
                                    if (tmp[1] == 0 && tmp[2] == 0)
                                    {
                                        ejection(solution, {droneIndex, tripIndex, yIndex},
                                                 DRONE,
                                                 customerX,
                                                 gain,
                                                 bestGain,
                                                 level,
                                                 shiftSequence,
                                                 bestShiftSequence);
                                    }
                                }
                            }

                            bool flag = false;
                            for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                                if (flag == true){
                                    break;
                                }
                                for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                                    if (flag == true){
                                        break;
                                    }
                                    for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                        if (*it == x) {
                                            flag = true;
                                            it = solution.droneTripList[xx][yy].erase(it);  
                                            break;
                                        } else {
                                            ++it;
                                        }
                                    }
                                }
                            }
                            for (int xx = 0; xx < solution.techTripList.size(); xx++){
                                if (flag == true){
                                    break;
                                }
                                for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                                    if (*it == x) {
                                        flag = true;
                                        it = solution.techTripList[xx].erase(it);  
                                        break;
                                    } else {
                                        ++it;
                                    }
                                }
                            }
                            gain += d;
                            shiftSequence.pop_back();
                            level--;
                        }
                    }
                }
            }
        }

        for (int techIndex = 0; techIndex < solution.techTripList.size(); techIndex++)
        {
            if (techIndex == xIndex[0])
            {
                continue;
            }
            for (int cusIndex = 0; cusIndex < solution.techTripList[techIndex].size(); cusIndex++)
            {
                int cus = solution.techTripList[techIndex][cusIndex];
                int cusPredecessor = (cusIndex == 0) ? 0 : solution.techTripList[techIndex][cusIndex - 1];
                double delta = input.techTimes[cusPredecessor][x] + input.techTimes[x][cus] - input.techTimes[cusPredecessor][cus];
                double d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                if (gain - d > bestGain)
                {
                    solution.techTripList[techIndex].insert(
                        solution.techTripList[techIndex].begin() + cusIndex, x);
                    gain -= d;

                    shiftSequence.push_back({xIndex, {techIndex, cusIndex}});
                    level++;
                    std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                    if (tmp[1] == 0 && tmp[2] == 0)
                    {
                        bestShiftSequence = shiftSequence;
                        bestGain = gain;
                    }
                    else if (level + 1 <= config.maxEjectionLevel)
                    {
                        for (int yIndex = 0;
                             yIndex < solution.techTripList[techIndex].size(); yIndex++)
                        {
                            Solution s = solution;

                            s.techTripList[techIndex].erase(
                                s.techTripList[techIndex].begin() + yIndex);
                            tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                            if (tmp[1] == 0 && tmp[2] == 0)
                            {
                                ejection(solution, {techIndex, yIndex},
                                         TECHNICIAN,
                                         customerX,
                                         gain,
                                         bestGain,
                                         level,
                                         shiftSequence,
                                         bestShiftSequence);
                            }
                        }
                    }

                    bool flag = false;
                    for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                        if (flag == true){
                            break;
                        }
                        for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                            if (flag == true){
                                break;
                            }
                            for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                if (*it == x) {
                                    flag = true;
                                    it = solution.droneTripList[xx][yy].erase(it);  
                                    break;
                                } else {
                                    ++it;
                                }
                            }
                        }
                    }
                    for (int xx = 0; xx < solution.techTripList.size(); xx++){
                        if (flag == true){
                            break;
                        }
                        for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                            if (*it == x) {
                                flag = true;
                                it = solution.techTripList[xx].erase(it);  
                                break;
                            } else {
                                ++it;
                            }
                        }
                    }
                    gain += d;
                    shiftSequence.pop_back();
                    level--;
                }

                if (cusIndex == solution.techTripList[techIndex].size() - 1)
                {
                    delta = input.techTimes[cus][x] + input.techTimes[x][input.numCus + 1] - input.techTimes[cus][input.numCus + 1];
                    d = std::max(cScore, techScores[techIndex] + delta) - cScore;

                    if (gain - d > bestGain)
                    {
                        solution.techTripList[techIndex].insert(
                            solution.techTripList[techIndex].begin() + cusIndex + 1, x);
                        gain -= d;

                        shiftSequence.push_back({xIndex, {techIndex, cusIndex + 1}});
                        level++;
                        std::vector<double> tmp = solution.getScoreATrip(techIndex, TECHNICIAN);
                        if (tmp[1] == 0 && tmp[2] == 0)
                        {
                            bestShiftSequence = shiftSequence;
                            bestGain = gain;
                        }
                        else if (level + 1 <= config.maxEjectionLevel)
                        {
                            for (int yIndex = 0;
                                 yIndex < solution.techTripList[techIndex].size(); yIndex++)
                            {
                                Solution s = solution;

                                s.techTripList[techIndex].erase(
                                    s.techTripList[techIndex].begin() + yIndex);
                                tmp = s.getScoreATrip(techIndex, TECHNICIAN);
                                if (tmp[1] == 0 && tmp[2] == 0)
                                {
                                    ejection(solution, {techIndex, yIndex},
                                             TECHNICIAN,
                                             customerX,
                                             gain,
                                             bestGain,
                                             level,
                                             shiftSequence,
                                             bestShiftSequence);
                                }
                            }
                        }

                        bool flag = false;
                        for (int xx = 0; xx < solution.droneTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (int yy = 0; yy < solution.droneTripList[xx].size(); yy++){
                                if (flag == true){
                                    break;
                                }
                                for (auto it = solution.droneTripList[xx][yy].begin(); it != solution.droneTripList[xx][yy].end(); ) {
                                    if (*it == x) {
                                        flag = true;
                                        it = solution.droneTripList[xx][yy].erase(it);  
                                        break;
                                    } else {
                                        ++it;
                                    }
                                }
                            }
                        }
                        for (int xx = 0; xx < solution.techTripList.size(); xx++){
                            if (flag == true){
                                break;
                            }
                            for (auto it = solution.techTripList[xx].begin(); it != solution.techTripList[xx].end(); ) {
                                if (*it == x) {
                                    flag = true;
                                    it = solution.techTripList[xx].erase(it);  
                                    break;
                                } else {
                                    ++it;
                                }
                            }
                        }
                        gain += d;
                        shiftSequence.pop_back();
                        level--;
                    }
                }
            }
        }

        solution.techTripList[xIndex[0]].insert(
            solution.techTripList[xIndex[0]].begin() + xIndex[1], x);
    }
}


void Solution::perturbation()
{
    double swapRate = 0.01;
    int num = 0;
    for (int droneIndex = 0; droneIndex < droneTripList.size(); droneIndex++)
    {
        for (int tripIndex = 0; tripIndex < droneTripList[droneIndex].size(); tripIndex++)
        {
            for (int xIndex = 0; xIndex < droneTripList[droneIndex][tripIndex].size(); xIndex++)
            {

                // drone
                for (int droneIndex2 = 0; droneIndex2 < droneTripList.size(); droneIndex2++)
                {
                    for (int tripIndex2 = 0; tripIndex2 < droneTripList[droneIndex2].size(); tripIndex2++)
                    {
                        if (droneIndex2 == droneIndex && tripIndex == tripIndex2)
                        {
                            continue;
                        }

                        for (int yIndex = 0; yIndex < droneTripList[droneIndex2][tripIndex2].size(); yIndex++)
                        {
                            if (Random::get(0.0, 1.0) > swapRate)
                            {
                                continue;
                            }
                            std::swap(droneTripList[droneIndex][tripIndex][xIndex],
                                      droneTripList[droneIndex2][tripIndex2][yIndex]);
                            num++;
                        }
                    }
                }

                // tech
                for (auto &techIndex : techTripList)
                {
                    for (int &yIndex : techIndex)
                    {
                        if (Random::get(0.0, 1.0) > swapRate)
                        {
                            continue;
                        }
                        std::swap(droneTripList[droneIndex][tripIndex][xIndex],
                                  yIndex);
                        num++;
                    }
                }
            }
        }
    }

    for (int techIndex = 0; techIndex < techTripList.size(); techIndex++)
    {
        for (int xIndex = 0; xIndex < techTripList[techIndex].size(); xIndex++)
        {

            // tech
            for (int techIndex2 = 0; techIndex2 < techTripList.size(); techIndex2++)
            {
                if (techIndex == techIndex2)
                {
                    continue;
                }
                for (int yIndex = 0; yIndex < techTripList[techIndex2].size(); yIndex++)
                {
                    if (Random::get(0.0, 1.0) > swapRate)
                    {
                        continue;
                    }

                    std::swap(techTripList[techIndex][xIndex], techTripList[techIndex2][yIndex]);
                    num++;
                }
            }

            // drone
            if (input.cusOnlyServedByTech[techTripList[techIndex][xIndex]])
            {
                continue;
            }

            for (auto &droneIndex : droneTripList)
            {
                for (auto &tripIndex : droneIndex)
                {
                    for (int &yIndex : tripIndex)
                    {
                        if (Random::get(0.0, 1.0) > swapRate)
                        {
                            continue;
                        }

                        std::swap(techTripList[techIndex][xIndex],
                                  yIndex);
                        num++;
                    }
                }
            }
        }
    }

    for (auto &droneIndex : droneTripList)
    {
        for (auto &tripIndex : droneIndex)
        {
            for (int xIndex = 0; xIndex < (int)tripIndex.size() - 1; xIndex++)
            {
                for (int yIndex = xIndex + 1; yIndex < tripIndex.size(); yIndex++)
                {
                    if (Random::get(0.0, 1.0) > swapRate)
                    {
                        continue;
                    }

                    std::swap(tripIndex[xIndex],
                              tripIndex[yIndex]);
                    num++;
                }
            }
        }
    }

    for (auto &techIndex : techTripList)
    {
        for (int xIndex = 0; xIndex < (int)techIndex.size() - 1; xIndex++)
        {
            for (int yIndex = xIndex + 1; yIndex < techIndex.size(); yIndex++)
            {
                if (Random::get(0.0, 1.0) > swapRate)
                {
                    continue;
                }

                std::swap(techIndex[xIndex],
                          techIndex[yIndex]);
                num++;
            }
        }
    }
    std::cout << "swap: " << num << std::endl;
}

void Solution::refactorSolution()
{
    for (auto &droneTrip : droneTripList)
    {
        droneTrip.erase(std::remove_if(droneTrip.begin(),
                                       droneTrip.end(),
                                       [](const auto &trip)
                                       { return trip.size() == 0; }),
                                       droneTrip.end());
    }
}

Solution::Solution() = default;

bool Solution::check_feasible()
{
    bool test = false;
    bool flag1 = false;
    for (int i = 0; i < droneTripList.size(); i++){
        if (flag1 == true){
            break;
        }
        for (int j = 0; j < droneTripList[i].size(); j++){
            if (droneTripList[i][j].size() != 0){
                test = true;
                flag1 = true;
                break;
            }
        }
    }
    if (test == false){
        for (int i = 0; i < techTripList.size(); i++){
            if (techTripList[i].size() != 0){
                test = true;
                break;
            }
        }
    }
    if (test == false){
        return false;
    }
    std::vector<double> techCompleteTime(config.numTech, 0);
    std::vector<double> droneCompleteTime(config.numDrone, 0);
    std::vector<double> cusCompleteTime(input.numCus + 1, 0);
    std::vector<std::vector<double>> droneTripCompleteTime;

    int maxTrip = 0;
    for (auto &i : droneTripList)
    {
        if (maxTrip < i.size())
        {
            maxTrip = (int)i.size();
        }
    }

    for (int i = 0; i < config.numDrone; i++)
    {
        std::vector<double> tripTime(maxTrip, 0);
        droneTripCompleteTime.push_back(tripTime);
    }

    double tmp, tmp1;
    dz = 0, cz = 0;

    double allTechTime = 0, allDroneTime = 0;

    for (int i = 0; i < techTripList.size(); i++)
    {
        if (techTripList[i].empty())
        {
            continue;
        }

        tmp = input.techTimes[0][techTripList[i][0]];

        cusCompleteTime[techTripList[i][0]] = tmp;

        for (int j = 0; j < (int)techTripList[i].size() - 1; j++)
        {
            tmp += input.techTimes[techTripList[i][j]][techTripList[i][j + 1]];
            cusCompleteTime[techTripList[i][j + 1]] = tmp;
        }

        techCompleteTime[i] = tmp + input.techTimes[techTripList[i].back()][0];
        if (techCompleteTime[i] > allTechTime)
        {
            allTechTime = techCompleteTime[i];
        }
    }

    for (int i = 0; i < droneTripList.size(); i++)
    {
        tmp = 0;
        for (int j = 0; j < droneTripList[i].size(); j++)
        {
            if (droneTripList[i][j].empty())
            {
                continue;
            }

            tmp1 = input.droneTimes[0][droneTripList[i][j][0]];
            cusCompleteTime[droneTripList[i][j][0]] = tmp1;

            for (int k = 0; k < (int)droneTripList[i][j].size() - 1; k++)
            {
                tmp1 += input.droneTimes[droneTripList[i][j][k]][droneTripList[i][j][k + 1]];
                cusCompleteTime[droneTripList[i][j][k + 1]] = tmp1;
            }
            droneTripCompleteTime[i][j] = tmp1 + input.droneTimes[droneTripList[i][j].back()][0];
            tmp += droneTripCompleteTime[i][j];
        }
        droneCompleteTime[i] = tmp;
        if (tmp > allDroneTime)
        {
            allDroneTime = tmp;
        }
    }


    for (int i = 0; i < techTripList.size(); i++)
    {
        if (techTripList[i].empty())
        {
            continue;
        }

        for (int j : techTripList[i])
        {
            cz += std::max(0., techCompleteTime[i] - cusCompleteTime[j] - config.sampleLimitationWaitingTime);
        }
    }
    if (cz != 0) return false;
    for (int i = 0; i < droneTripList.size(); i++)
    {
        for (int j = 0; j < droneTripList[i].size(); j++)
        {
            if (droneTripList[i][j].empty())
            {
                continue;
            }
            for (int k : droneTripList[i][j])
            {
                cz += std::max(0.,
                               droneTripCompleteTime[i][j] - cusCompleteTime[k] - config.sampleLimitationWaitingTime);
            }
            dz += std::max(0., droneTripCompleteTime[i][j] - config.droneLimitationFlightTime);
        }
    }
    if ((cz != 0) or (dz != 0)){
        return false;
    }
    return true;
}