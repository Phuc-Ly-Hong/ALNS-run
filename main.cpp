#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <map>
#include <numeric>    // thêm cho std::accumulate
#include <algorithm>  // thêm cho std::min_element

#include "src/nlohmann/json.hpp"
#include "src/Config.h"
#include "src/Input.h"
#include "src/Solution.h"
#include "src/MultiLevel.h"
#include "src/LargeNeighborhoodSearch.h"
#include "src/Utils.h"
#include "src/Random.h"

using namespace std::chrono;
using json = nlohmann::json;
using Random = effolkronium::random_static;

std::vector<std::string> file_browsing(std::string folderPath, std::string numcus){
    std::vector<std::string> file_browse;
    for (const auto& ite : std::filesystem::directory_iterator(folderPath)) {
        std::string number;
        for (int i = 0; i < (int)ite.path().filename().string().size(); i++){
            if (ite.path().filename().string()[i] == '.') break;
            number = number + ite.path().filename().string()[i];
        }
        if (ite.is_regular_file() && number == numcus){
            file_browse.push_back(ite.path().filename().string());
        }
    }
    return file_browse;
}

int main(int argc, char **argv)
{    
    Config config;
    std::string configFilePath = "Config.json";
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) { return 1; }
    json jsConfig = json::parse(configFile);
    
    config.droneVelocity               = jsConfig["parameter"]["droneVelocity"].get<double>();
    config.techVelocity                = jsConfig["parameter"]["techVelocity"].get<double>();
    config.numDrone                    = jsConfig["parameter"]["numDrone"].get<int>();
    config.numTech                     = jsConfig["parameter"]["numTech"].get<int>();
    config.droneLimitationFlightTime   = jsConfig["parameter"]["droneLimitationFlightTime"].get<double>();
    config.sampleLimitationWaitingTime = jsConfig["parameter"]["sampleLimitationWaitingTime"].get<double>();
    config.use_ejection                = jsConfig["parameter"]["use_ejection"].get<bool>();
    config.use_inter                   = jsConfig["parameter"]["use_inter"].get<bool>();
    config.use_intra                   = jsConfig["parameter"]["use_intra"].get<bool>();
    config.num_level                   = jsConfig["multiLevel_para"]["num_level"].get<int>();
    config.percent_match               = jsConfig["multiLevel_para"]["percent_match"].get<double>();
    config.percent_select              = jsConfig["multiLevel_para"]["percent_select"].get<double>();
    config.tabuMaxIter                 = jsConfig["multiLevel_para"]["tabuMaxIter"].get<int>();
    config.tabuAlpha1                  = jsConfig["tabu_para"]["tabuAlpha1"].get<double>();
    config.tabuAlpha2                  = jsConfig["tabu_para"]["tabuAlpha2"].get<double>();
    config.tabuEpsilon                 = jsConfig["tabu_para"]["tabuEpsilon"].get<double>();
    config.ws                          = jsConfig["ws"].get<std::string>();
    config.dataPath                    = jsConfig["dataPath"].get<std::string>();
    config.MultiLevelresultFolder      = jsConfig["MultiLevelresultFolder"].get<std::string>();
    config.NumRunPerDataSet            = jsConfig["NumRunPerDataSet"].get<int>();
    config.overwrite                   = jsConfig["overwrite"].get<bool>();
    config.plus1                       = jsConfig["plus1"].get<int>();

    std::vector<int> listDataType = {50, 100};
    std::map<int, std::pair<int, int>> numberVehicle;
    
    if (config.plus1 == 1){
        numberVehicle[6]    = {3, 3};
        numberVehicle[10]   = {3, 3};
        numberVehicle[12]   = {3, 3};
        numberVehicle[20]   = {3, 3};
        numberVehicle[50]   = {3, 3};
        numberVehicle[100]  = {4, 4};
        numberVehicle[200]  = {3, 3};
        numberVehicle[500]  = {3, 3};
        numberVehicle[1000] = {3, 3};
    } else {
        numberVehicle[6]   = {2, 2}; 
        numberVehicle[10]  = {2, 2};
        numberVehicle[50]  = {3, 3};
        numberVehicle[100] = {4, 4};
    }

    json summaryAll;
    int total_ML_wins = 0, total_LNS_wins = 0, total_draws = 0;

    for (int value : listDataType){
        std::cout << "\n\n========== Dataset size: " << value << " ==========" << std::endl;
        config.numDrone = numberVehicle[value].first;
        config.numTech  = numberVehicle[value].second;
        
        std::vector<std::string> dataList = file_browsing(config.ws + config.dataPath, std::to_string(value));
        std::string output_path = config.MultiLevelresultFolder + "COMPARE_" + std::to_string(value) + ".json";
        json logDataSet;

        for (int ite = 0; ite < (int)dataList.size(); ite++)
        {
            std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, dataList[ite]);
            
            for (const std::string &path : paths)
            {
                Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);

                // Kiểm tra feasibility 1 lần trước
                Solution* feasCheck = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);
                if (feasCheck == nullptr || (feasCheck->droneTripList.empty() && feasCheck->techTripList.empty())) {
                    std::cout << "[SKIP] Infeasible: " << input.dataSet << std::endl;
                    delete feasCheck; continue;
                }
                delete feasCheck;

                std::cout << "\n-- Instance: " << input.dataSet
                          << "  (" << config.NumRunPerDataSet << " runs)" << std::endl;

                std::vector<double> scoresML, scoresLNS, timesML, timesLNS;

                for (int run = 0; run < config.NumRunPerDataSet; run++)
                {
                    // Mỗi run init lại solution độc lập
                    Solution* initSol = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);
                    if (initSol == nullptr || (initSol->droneTripList.empty() && initSol->techTripList.empty())) {
                        delete initSol; continue;
                    }
                    Solution startSolution = *initSol;
                    delete initSol;

                    // --- 1. MULTI-LEVEL ---
                    MultiLevel multilev(config, input);
                    auto beginML = high_resolution_clock::now();
                    std::tuple<double, Solution, std::vector<double>> resultML = multilev.run(config, input);
                    auto endML = high_resolution_clock::now();
                    double timeML  = duration_cast<milliseconds>(endML - beginML).count() / 1000.0;
                    double scoreML = std::get<0>(resultML);

                    // --- 2. LNS THUẦN (cùng time budget) ---
                    LargeNeighborhoodSearch lnsOnly(config, input);
                    Solution lnsStartSol = startSolution;
                    lnsStartSol.setInput(input);
                    auto beginLNS = high_resolution_clock::now();
                    std::tuple<double, Solution> resultLNS = lnsOnly.runWithTimeLimit(input, lnsStartSol, timeML);
                    auto endLNS = high_resolution_clock::now();
                    double timeLNS  = duration_cast<milliseconds>(endLNS - beginLNS).count() / 1000.0;
                    double scoreLNS = std::get<0>(resultLNS);

                    scoresML.push_back(scoreML);
                    scoresLNS.push_back(scoreLNS);
                    timesML.push_back(timeML);
                    timesLNS.push_back(timeLNS);

                    std::cout << "   Run " << (run + 1) << ": "
                              << "[ML] " << std::fixed << std::setprecision(4) << scoreML
                              << " (" << std::setprecision(2) << timeML << "s)  "
                              << "[LNS] " << std::setprecision(4) << scoreLNS
                              << " (" << std::setprecision(2) << timeLNS << "s)" << std::endl;
                }

                int n = (int)scoresML.size();
                if (n == 0) continue;

                // --- Tính avg và best (bài toán minimize → best = min) ---
                double avg_ML    = std::accumulate(scoresML.begin(),  scoresML.end(),  0.0) / n;
                double avg_LNS   = std::accumulate(scoresLNS.begin(), scoresLNS.end(), 0.0) / n;
                double avg_tML   = std::accumulate(timesML.begin(),   timesML.end(),   0.0) / n;
                double avg_tLNS  = std::accumulate(timesLNS.begin(),  timesLNS.end(),  0.0) / n;
                double best_ML   = *std::min_element(scoresML.begin(),  scoresML.end());
                double best_LNS  = *std::min_element(scoresLNS.begin(), scoresLNS.end());

                // GAP: > 0 nghĩa là LNS cao hơn (tệ hơn) → ML thắng
                double gap_avg  = (avg_LNS  - avg_ML)  / avg_ML  * 100.0;
                double gap_best = (best_LNS - best_ML) / best_ML * 100.0;

                auto decide = [](double gap) -> std::string {
                    if (gap > 0) return "Multi-Level";
                    if (gap < 0) return "LNS Only";
                    return "Draw";
                };
                std::string winner_avg  = decide(gap_avg);
                std::string winner_best = decide(gap_best);

                // Đếm win dựa trên avg
                if      (gap_avg > 0) total_ML_wins++;
                else if (gap_avg < 0) total_LNS_wins++;
                else                  total_draws++;

                std::cout << "   --- Summary (" << n << " runs) ---" << std::endl;
                std::cout << "   [ML ] Avg=" << std::setprecision(4) << avg_ML
                          << "  Best=" << best_ML
                          << "  AvgTime=" << std::setprecision(2) << avg_tML << "s" << std::endl;
                std::cout << "   [LNS] Avg=" << std::setprecision(4) << avg_LNS
                          << "  Best=" << best_LNS
                          << "  AvgTime=" << std::setprecision(2) << avg_tLNS << "s" << std::endl;
                std::cout << "   GAP_avg=" << std::setprecision(4) << gap_avg
                          << "%  GAP_best=" << gap_best
                          << "%  Winner(avg)=" << winner_avg << std::endl;

                json logEntry;
                logEntry["runs"]            = n;
                logEntry["ML_Avg_Score"]    = avg_ML;
                logEntry["ML_Best_Score"]   = best_ML;
                logEntry["ML_Avg_Time_s"]   = avg_tML;
                logEntry["LNS_Avg_Score"]   = avg_LNS;
                logEntry["LNS_Best_Score"]  = best_LNS;
                logEntry["LNS_Avg_Time_s"]  = avg_tLNS;
                logEntry["GAP_Avg_percent"] = gap_avg;
                logEntry["GAP_Best_percent"]= gap_best;
                logEntry["Winner_by_Avg"]   = winner_avg;
                logEntry["Winner_by_Best"]  = winner_best;

                logDataSet[input.dataSet] = logEntry;
                summaryAll[input.dataSet] = logEntry;
            }
        }

        std::ofstream output_file(output_path);
        output_file << std::setw(4) << logDataSet << std::endl;
        output_file.close();
        std::cout << "\nSaved: " << output_path << std::endl;
    }

    json overall;
    overall["Total_ML_wins"]  = total_ML_wins;
    overall["Total_LNS_wins"] = total_LNS_wins;
    overall["Total_draws"]    = total_draws;
    summaryAll["_OVERALL"]    = overall;

    std::string summary_path = config.MultiLevelresultFolder + "SUMMARY_ALL.json";
    std::ofstream summary_file(summary_path);
    summary_file << std::setw(4) << summaryAll << std::endl;
    summary_file.close();

    std::cout << "\n========== DONE ==========" << std::endl;
    std::cout << "Summary saved: " << summary_path << std::endl;
    std::cout << "ML wins : " << total_ML_wins  << std::endl;
    std::cout << "LNS wins: " << total_LNS_wins << std::endl;
    std::cout << "Draws   : " << total_draws     << std::endl;

    return 0;
}