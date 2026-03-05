#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <map>

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

    // File tổng hợp tất cả instance
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
                
                Solution* initSolCheck = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);
                if (initSolCheck == nullptr || (initSolCheck->droneTripList.empty() && initSolCheck->techTripList.empty())) {
                    std::cout << "[SKIP] Infeasible: " << input.dataSet << std::endl;
                    delete initSolCheck; continue;
                }
                Solution startSolution = *initSolCheck;
                delete initSolCheck;

                std::cout << "\n-- Instance: " << input.dataSet << std::endl;
                std::cout << "   Init Score: " << startSolution.getScore() << std::endl;

                // --- 1. MULTI-LEVEL ---
                MultiLevel multilev(config, input);
                auto beginML = high_resolution_clock::now();
                std::tuple<double, Solution, std::vector<double>> resultML = multilev.run(config, input);
                auto endML = high_resolution_clock::now();
                double timeML  = duration_cast<milliseconds>(endML - beginML).count() / 1000.0;
                double scoreML = std::get<0>(resultML);

                // --- 2. LNS THUẦN ---
                LargeNeighborhoodSearch lnsOnly(config, input);
                Solution lnsStartSol = startSolution;
                lnsStartSol.setInput(input);
                auto beginLNS = high_resolution_clock::now();
                std::tuple<double, Solution> resultLNS = lnsOnly.runWithTimeLimit(input, lnsStartSol, timeML);
                auto endLNS = high_resolution_clock::now();
                double timeLNS  = duration_cast<milliseconds>(endLNS - beginLNS).count() / 1000.0;
                double scoreLNS = std::get<0>(resultLNS);

                // --- 3. SO SÁNH ---
                double gap = ((scoreLNS - scoreML) / scoreML) * 100.0;
                std::string winner;
                if      (gap > 0) { winner = "Multi-Level"; total_ML_wins++;  }
                else if (gap < 0) { winner = "LNS Only";    total_LNS_wins++; }
                else              { winner = "Draw";         total_draws++;    }

                // In ra console
                std::cout << "   [Multi-Level] Score: " << std::fixed << std::setprecision(4) << scoreML
                          << "  |  Time: " << std::setprecision(2) << timeML << "s" << std::endl;
                std::cout << "   [LNS Only   ] Score: " << std::setprecision(4) << scoreLNS
                          << "  |  Time: " << std::setprecision(2) << timeLNS << "s" << std::endl;
                std::cout << "   GAP: " << std::setprecision(4) << gap << "%  =>  " << winner << std::endl;

                // Ghi vào COMPARE riêng
                json logEntry;
                logEntry["ML_Score"]      = scoreML;
                logEntry["ML_Time_s"]     = timeML;
                logEntry["LNS_Score"]     = scoreLNS;
                logEntry["LNS_Time_s"]    = timeLNS;
                logEntry["GAP_percent"]   = gap;
                logEntry["Winner"]        = winner;
                logDataSet[input.dataSet] = logEntry;

                // Ghi vào SUMMARY tổng hợp (key = tên instance, VD: "50.10.1")
                summaryAll[input.dataSet] = logEntry;
            }
        }

        // Ghi file COMPARE riêng từng size
        std::ofstream output_file(output_path);
        output_file << std::setw(4) << logDataSet << std::endl;
        output_file.close();
        std::cout << "\nSaved: " << output_path << std::endl;
    }

    // Thêm dòng tổng kết cuối file summary
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