
#include "TabuSearch.h"
#include "Solution.h"
#include "Random.h"
#include <chrono>
#include "nlohmann/json.hpp"

using namespace std::chrono;

using Random = effolkronium::random_static;
using json = nlohmann::json;

void TabuSearch::setConfigandInput(Config &conf, Input &inp)
{
    this->config = conf;
    this->input = inp;
}

std::tuple<double, Solution, std::vector<std::vector<int>>> TabuSearch::run(json &log, std::string &path, Input &input, Solution solution)
{
    setConfigandInput(config, input);
    solution.setInput(input);
    for (int i = 0; i < solution.droneTripList.size(); i++){
        if (solution.droneTripList[i].size() == 0){
            solution.droneTripList[i].push_back({});
        }
    }

    //alpha1 = 0;
    //alpha2 = 0;
    json feasible;
    std::vector<std::vector<int>> matrix;
    matrix.resize(input.numCus + 1);
    for (int i = 0; i < matrix.size(); i++)
    {
        matrix[i].resize(input.numCus + 1);
    }
    std::map<NeighborhoodType, std::vector<std::string>> tabuLists;
    tabuLists[MOVE_10] = std::vector<std::string>();
    tabuLists[MOVE_11] = std::vector<std::string>();
    tabuLists[MOVE_20] = std::vector<std::string>();
    tabuLists[MOVE_21] = std::vector<std::string>();
    tabuLists[TWO_OPT] = std::vector<std::string>();

    Solution bestFeasibleSolution;
    bestFeasibleSolution.setInput(input);
    bestSolution.setInput(input);
    int notImproveIter = 0;
    currentSolution = solution;
    currentSolution.setInput(input);
    double currentScore = solution.getScore();
    bestSolution = solution;
    double bestScore = currentScore;
    double bestFeasibleScore = 999999;
    NeighborhoodType neighborhoodType;
    int actOrd;
    Solution *s;
    feasible["best feasible score"] = std::to_string(9999999);
    if (currentSolution.check_feasible()){
        json jDrone(currentSolution.droneTripList);
        json jTech(currentSolution.techTripList);
        feasible["best feasible score"] = std::to_string(currentScore);
        feasible["best feasible"] = std::to_string(currentScore) + jDrone.dump() + " || " + jTech.dump();
        bestFeasibleSolution = currentSolution;
        bestFeasibleScore = currentScore;
        bestFeasibleSolution.setInput(input);
    }

    //start Tabu
    int notFeasibleImprove = 0;
    auto start = high_resolution_clock::now();
    int actOrderCycle = -1;
    //config.isCycle = false;
    for (int it = 0; it < config.tabuMaxIter; it++)
    {
        //std::cout << std::endl << "it = " << it << ", score = " << bestFeasibleScore;
        /*if (it % 5 == 0){
            //std::cout << std::endl << "False";
            std::random_device rd;
            std::mt19937 mt(rd());
            std::shuffle(input.neighborhoodOrders.begin(), input.neighborhoodOrders.end(), mt);
        }*/
        //std::cout << std::endl << "Is Feasible ?? = " << currentSolution.check_feasible();
        //std::cout<<std::endl<<"Ite "<<it<<" | Score = "<<bestFeasibleSolution.getScore() << " || " << "Current Score = " << currentScore;
        //std::cout << std::endl << "alpha1 = " << alpha1 << " || " << "alpha2 = " << alpha2;
        //std::cout << std::endl << "cz = " << currentSolution.cz << " || " << "dz = " << currentSolution.dz;

        json jDrone(currentSolution.droneTripList);
        json jTech(currentSolution.techTripList);

        actOrderCycle = (actOrderCycle + 1) % 5;
        if (config.isCycle)
        {
            actOrd = actOrderCycle + 1;
        }
        else
        {
            actOrd = Random::get(1, 5);
        }
        //actOrd = input.neighborhoodOrders[actOrd - 1];
        //std::cout << std::endl << "actOrd = " << actOrd;
        Solution bestFeasibleBefore = bestFeasibleSolution;
        //actOrd = Random::get(1, 5);
        //std::cout<<std::endl<<"actord is: "<<actOrd;
        //config.isCycle = 1;
        
        //        actOrd = 4;
        //        std::cout << "act: " << actOrd << std::endl;
        //        auto start = high_resolution_clock::now();
        switch (actOrd)
        {
        case MOVE_10:
        {
            neighborhoodType = MOVE_10;
            s = currentSolution.relocate(tabuLists[MOVE_10], bestFeasibleSolution);
            break;
        }
        case MOVE_11:
        {
            neighborhoodType = MOVE_11;
            s = currentSolution.exchange(tabuLists[MOVE_11], bestFeasibleSolution);
            break;
        }
        case MOVE_20:
        {
            neighborhoodType = MOVE_20;
            s = currentSolution.orOpt(tabuLists[MOVE_20], bestFeasibleSolution);
            break;
        }
        case MOVE_21:
        {
            neighborhoodType = MOVE_21;
            s = currentSolution.crossExchange(tabuLists[MOVE_21], bestFeasibleSolution);
            break;
        }
        case TWO_OPT:
        {
            neighborhoodType = TWO_OPT;
            s = currentSolution.twoOpt(tabuLists[TWO_OPT], bestFeasibleSolution);
            break;
        }
        default:
        {
            s = nullptr;
            break;
        }
        }
        if (s != nullptr)
        {
            //s->refactorSolution();
            tabuLists[neighborhoodType].push_back(s->ext["state"]);
            while (tabuLists[neighborhoodType].size() > tabuDuration)
            {
                tabuLists[neighborhoodType].erase(tabuLists[neighborhoodType].begin());
            }
            currentSolution = *s;
            json jDroneOld(currentSolution.droneTripList);
            json jTechOld(currentSolution.techTripList);
            currentScore = currentSolution.getScore();
            json jDrone(currentSolution.droneTripList);
            json jTech(currentSolution.techTripList);
            updatePenalty(currentSolution.dz, currentSolution.cz);
            currentSolution.alpha1 = alpha1;
            currentSolution.alpha2 = alpha2;
            double besttScore = bestFeasibleSolution.getScore();
            if (besttScore - bestFeasibleScore < config.tabuEpsilon)
            {
                feasible["best feasible score"] = std::to_string(currentScore);
                feasible["best feasible"] = std::to_string(currentScore) + jDrone.dump() + " || " + jTech.dump();
                bestFeasibleScore = besttScore;
                bestSolution = bestFeasibleBefore;
                notImproveIter = 0;
                config.isCycle = true;
            }else{
                notImproveIter++;
                if (notImproveIter > config.tabuNotImproveIter)
                {
                    break;
                }
            }
            json jDroneBest(bestSolution.droneTripList);
            json jTechBest(bestSolution.techTripList);
        }

        //        auto stop = high_resolution_clock::now();
        //        std::cout << it << ": "
        //                  << bestScore << "-" << neighborhoodType << " time: "
        //                  << duration_cast<seconds>(stop - start).count() << "s" << std::endl;

        //        std::cout << it << ": " << itLog.dump(4) << std::endl;
    }
    bestFeasibleSolution.refactorSolution();
    auto stop = high_resolution_clock::now();
    json jDroneBest(bestSolution.droneTripList);
    json jTechBest(bestSolution.techTripList);
    return std::make_tuple(bestFeasibleScore, bestFeasibleSolution, matrix);
}

TabuSearch::TabuSearch(Config &conf, Input &inp)
{
    this->config = conf;
    this->input = inp;
    this->tabuDuration = config.tabuDuration;
    this->alpha1 = conf.tabuAlpha1;
    this->alpha2 = conf.tabuAlpha2;
    Solution *init = Solution::initSolution(this->config, this->input, InitType::MIX, alpha1, alpha2);
    if (init == nullptr)
    {
        return;
    }
    initSolution = *init;
}

void TabuSearch::updatePenalty(double dz, double cz)
{
    if (std::abs(dz) > std::abs(config.tabuEpsilon))
    {
        alpha2 *= (1 + config.tabuBeta);
    }
    else
    {
        alpha2 /= (1 + config.tabuBeta);
    }

    if (std::abs(cz) > std::abs(config.tabuEpsilon))
    {
        alpha1 *= (1 + config.tabuBeta);
    }
    else
    {
        alpha1 /= (1 + config.tabuBeta);
    }
}

Solution TabuSearch::runPostOptimization(json &log, Solution solution)
{
    //auto start = high_resolution_clock::now();

    Solution solutionx = solution;
    solutionx = runEjection(solutionx);
    if (solutionx.check_feasible() && solution.getScore() < solutionx.getScore()){
        solution = solutionx;
    }
    //json jDroneBestEjection(solution.droneTripList);
    //json jTechBestEjection(solution.techTripList);
    //log["best_ejection"] = std::to_string(bestSolution.getScore()) + " == " + jDroneBestEjection.dump() + " || " + jTechBestEjection.dump();

    //    std::cout << "Ejection: " << log["best_ejection"].dump(4) << std::endl;

    solution = runInterRoute(solution);
    //json jDroneBestInter(solution.droneTripList);
    //json jTechBestInter(solution.techTripList);
    //log["best_inter"] = std::to_string(bestSolution.getScore()) + " == " + jDroneBestInter.dump() + " || " + jTechBestInter.dump();

    //    std::cout << "Inter: " << log["best_inter"].dump(4) << std::endl;

    solution = runIntraRoute(solution);


    //json jDroneBestIntra(bestSolution.droneTripList);
    //json jTechBestIntra(bestSolution.techTripList);
    //log["best_intra"] = std::to_string(bestSolution.getScore()) + " == " + jDroneBestIntra.dump() + " || " + jTechBestIntra.dump();
    //    std::cout << "Intra: " << log["best_intra"].dump(4) << std::endl;
    auto stop = high_resolution_clock::now();
    //log["post_optimization_time"] = duration_cast<milliseconds>(stop - start).count();
    return solution;
}

Solution TabuSearch::runInterRoute(Solution &solution)
{
    Config config;
    auto rng = std::default_random_engine(std::chrono::system_clock::now()
                                              .time_since_epoch()
                                              .count());
    std::vector<InterRouteType> order{INTER_RELOCATE, INTER_CROSS_EXCHANGE, INTER_EXCHANGE, INTER_OR_OPT,
                                      INTER_TWO_OPT};

    double score = solution.getScore();
    Solution solution1 = solution;
    double newScore;

    Solution *s;
    while (true)
    {
        std::shuffle(order.begin(), order.end(), rng);
        bool hasImprove = false;

        for (InterRouteType type : order)
        {
            switch (type)
            {
            case INTER_RELOCATE:
            {
                s = solution1.relocate({}, solution, INTER);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_EXCHANGE:
            {
                s = solution1.exchange({}, solution, INTER);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_OR_OPT:
            {
                s = solution1.orOpt({}, solution, INTER);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_TWO_OPT:
            {
                s = solution1.twoOpt({}, solution, INTER);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTER_CROSS_EXCHANGE:
            {
                s = solution1.crossExchange({}, solution, INTER);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            }
        }
        solution.refactorSolution();

        if (!hasImprove)
        {
            std::cout<<std::endl<<" | "<<solution.getScore();
            break;
        }
    }
    return solution;
}

Solution TabuSearch::runIntraRoute(Solution &solution)
{
    Config config;
    auto rng = std::default_random_engine(std::chrono::system_clock::now()
                                              .time_since_epoch()
                                              .count());
    std::vector<IntraRouteType> order{INTRA_RELOCATE, INTRA_EXCHANGE, INTRA_OR_OPT, INTRA_TWO_OPT};

    double score = solution.getScore();
    Solution solution1 = solution;
    double newScore;
    Solution *s;
    while (true)
    {
        std::shuffle(order.begin(), order.end(), rng);
        bool hasImprove = false;

        for (IntraRouteType type : order)
        {
            switch (type)
            {
            case INTRA_RELOCATE:
            {
                s = solution1.relocate({}, solution, INTRA);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTRA_EXCHANGE:
            {
                s = solution1.exchange({}, solution, INTRA);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTRA_TWO_OPT:
            {
                s = solution1.twoOpt({}, solution, INTRA);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }
                break;
            }
            case INTRA_OR_OPT:
            {
                s = solution1.orOpt({}, solution, INTRA);
                if (s != nullptr)
                {
                    double newScore = solution.getScore();
                    if (newScore < score)
                    {
                        solution1.droneTripList = s->droneTripList;
                        solution1.techTripList = s->techTripList;
                        score = newScore;
                        hasImprove = true;
                    }
                }

                break;
            }
            }
        }
        solution.refactorSolution();

        if (!hasImprove)
        {
            std::cout<<std::endl<<" | "<<solution.getScore();
            break;
        }
    }
    return solution;
}

Solution TabuSearch::runEjection(Solution &solution)
{
    Config config;
    Solution solution1 = solution;
    double score = solution1.getScore();
    Solution *s;

    while (true){
        bool hasImprove = false;
        s = solution1.ejectionNeighborhoodAdd(solution);
        if (s != nullptr)
        {
            double newScore = solution.getScore();
            if (newScore < score)
            {
                solution1.droneTripList = s->droneTripList;
                solution1.techTripList = s->techTripList;
                score = newScore;
                hasImprove = true;
            }
        }
        if (!hasImprove)
        {
            break;
        }
    }
    solution.refactorSolution();
    return solution;
}

TabuSearch::TabuSearch() = default;