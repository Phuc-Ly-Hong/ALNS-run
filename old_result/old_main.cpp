#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>

#include "src/nlohmann/json.hpp"
#include "src/Config.h"
#include "src/Input.h"
#include "src/Solution.h"
#include "src/MultiLevel.h"
#include "src/LargeNeighborhoodSearch.h" // Thêm thư viện LNS
#include "src/Utils.h"
#include "src/Random.h"

using namespace std::chrono;
using json = nlohmann::json;
using Random = effolkronium::random_static;

std::vector<std::string> file_browsing(std::string folderPath, std::string numcus){
    std::vector<std::string> file_browse;
    for (const auto& ite : std::filesystem::directory_iterator(folderPath)) {
        std::string number;
        for (int i = 0; i < ite.path().filename().string().size(); i++){
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
    std::vector<std::map<int, std::vector<int>>> mapLevel;
    std::vector<std::vector<std::vector<double>>> DistanceMatrixLevel;
    std::vector<std::vector<bool>> C1Level;
    // --- SỬA LẠI ĐƯỜNG DẪN VÀ KIỂM TRA FILE ---
    std::string configFilePath = "Config.json"; 
    std::ifstream configFile(configFilePath);
    
    if (!configFile.is_open()) {
        std::cerr << "LOI: Khong the mo file " << configFilePath << std::endl;
        std::cerr << "Hay chac chan file Config.json nam cung thu muc voi file main.exe" << std::endl;
        return 1;
    }

    // Kiểm tra file rỗng
    if (configFile.peek() == std::ifstream::traits_type::eof()) {
        std::cerr << "LOI: File " << configFilePath << " bi rong!" << std::endl;
        return 1;
    }

    json jsConfig;
    try {
        jsConfig = json::parse(configFile);
    } catch (json::parse_error& e) {
        std::cerr << "LOI CU PHAP JSON trong Config.json: " << e.what() << std::endl;
        return 1;
    }
    // -------------------------------------------
    config.droneVelocity = jsConfig["parameter"]["droneVelocity"].get<double>();
    config.techVelocity = jsConfig["parameter"]["techVelocity"].get<double>();
    config.numDrone = jsConfig["parameter"]["numDrone"].get<int>();
    config.numTech = jsConfig["parameter"]["numTech"].get<int>();
    config.droneLimitationFlightTime = jsConfig["parameter"]["droneLimitationFlightTime"].get<double>();
    config.sampleLimitationWaitingTime = jsConfig["parameter"]["sampleLimitationWaitingTime"].get<double>();
    config.use_ejection = jsConfig["parameter"]["use_ejection"].get<bool>();
    config.use_inter = jsConfig["parameter"]["use_inter"].get<bool>();
    config.use_intra = jsConfig["parameter"]["use_intra"].get<bool>();
    config.num_level = jsConfig["multiLevel_para"]["num_level"].get<int>();
    config.percent_match = jsConfig["multiLevel_para"]["percent_match"].get<double>();
    config.percent_select = jsConfig["multiLevel_para"]["percent_select"].get<double>();
    config.tabu_size = jsConfig["multiLevel_para"]["tabu_size"].get<int>();
    config.tabuMaxIter = jsConfig["multiLevel_para"]["tabuMaxIter"].get<int>();
    config.tabuNumRunPerDataSet = 1;
    config.tabuNotImproveIter = jsConfig["tabu_para"]["tabuNotImproveIter"].get<int>();
    config.tabuAlpha1 = jsConfig["tabu_para"]["tabuAlpha1"].get<double>();
    config.tabuAlpha2 = jsConfig["tabu_para"]["tabuAlpha2"].get<double>();
    config.tabuBeta = jsConfig["tabu_para"]["tabuBeta"].get<double>();
    config.tabuEpsilon = jsConfig["tabu_para"]["tabuEpsilon"].get<double>();
    config.maxEjectionLevel = jsConfig["tabu_para"]["maxEjectionLevel"].get<int>();
    config.minTabuDuration = jsConfig["tabu_para"]["minTabuDuration"].get<int>();
    config.maxTabuDuration = jsConfig["tabu_para"]["maxTabuDuration"].get<int>();
    config.isCycle = jsConfig["tabu_para"]["isCycle"].get<bool>();
    config.ws = jsConfig["ws"].get<std::string>();;
    config.dataName = jsConfig["dataName"].get<std::string>();
    config.dataPath = jsConfig["dataPath"].get<std::string>();
    config.lnsresultFolder = jsConfig["TaburesultFolder"].get<std::string>();
    config.MultiLevelresultFolder = jsConfig["MultiLevelresultFolder"].get<std::string>();
    config.addEjectionType = jsConfig["addEjectionType"].get<int>();
    config.NumRunPerDataSet = jsConfig["NumRunPerDataSet"].get<int>();
    config.multiData = jsConfig["multiData"].get<bool>();
    config.dataType = jsConfig["dataType"].get<std::string>();
    config.ejectionIte = jsConfig["ejectionIte"].get<int>();
    config.overwrite = jsConfig["overwrite"].get<bool>();
    config.tabuDuration = Random::get(config.minTabuDuration, config.maxTabuDuration);
    config.run_type = jsConfig["run_type"].get<int>();
    config.plus1 = jsConfig["plus1"].get<int>();
    config.plustech = jsConfig["plustech"].get<int>();
    std::vector<std::string> dataList = file_browsing(config.ws + config.dataPath, config.dataType);
    if (config.multiData == false)
    {
        dataList.resize(0);
        std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, config.dataName);
        for (const std::string &path : paths)
        {
            Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
            dataList.push_back(input.dataSet + ".txt");
        }
    }
    std::vector<int> listDataType;

    listDataType = {6,10,12,20}; // Chỉ chạy các file bắt đầu bằng số 6 (như 6.5.1.txt)
    std::map<int, std::pair<int, int>> numberVehicle;
    if (config.plus1 == 0){
        numberVehicle[6] = {0, 4};
        numberVehicle[10] = {0, 4};
        numberVehicle[12] = {0, 4};
        numberVehicle[20] = {0, 4};
        numberVehicle[50] = {0, 6};
        numberVehicle[100] = {0, 8};
        numberVehicle[150] = {0, 10};
        numberVehicle[200] = {0, 12};
        numberVehicle[500] = {0, 14};
        numberVehicle[1000] = {0, 16};
    } else if (config.plus1 == 1) {
        numberVehicle[6] = {3, 3};
        numberVehicle[10] = {3, 3};
        numberVehicle[12] = {3, 3};
        numberVehicle[20] = {3, 3};
        numberVehicle[50] = {3, 3};
        numberVehicle[100] = {1, 8};
        numberVehicle[150] = {1, 10};
        numberVehicle[200] = {1, 12};
        numberVehicle[500] = {1, 14};
        numberVehicle[1000] = {1, 16};
    } else if (config.plus1 == 2) {
        numberVehicle[6] = {0, 5};
        numberVehicle[10] = {0, 5};
        numberVehicle[12] = {0, 5};
        numberVehicle[20] = {0, 5};
        numberVehicle[50] = {0, 7};
        numberVehicle[100] = {0, 9};
        numberVehicle[150] = {0, 11};
        numberVehicle[200] = {0, 13};
        numberVehicle[500] = {0, 15};
        numberVehicle[1000] = {0, 17};
    } else if (config.plus1 == 3) {
        numberVehicle[6] = {0, 6};
        numberVehicle[10] = {0, 6};
        numberVehicle[12] = {0, 6};
        numberVehicle[20] = {0, 6};
        numberVehicle[50] = {0, 8};
        numberVehicle[100] = {0, 10};
        numberVehicle[150] = {0, 12};
        numberVehicle[200] = {0, 14};
        numberVehicle[500] = {0, 16};
        numberVehicle[1000] = {0, 18};
    }

    for (int value : listDataType){
        std::cout << std::endl << "Num Drone = " << numberVehicle[value].first << " | num Tech = " << numberVehicle[value].second;
        dataList = file_browsing(config.ws + config.dataPath, std::to_string(value));
        std::vector<std::string> dataList1 = file_browsing(config.ws + config.dataPath, std::to_string(value));
        if (config.run_type != 0){
            dataList = {};
            for (int i = 0; i < 4; ++i){
                dataList.push_back(dataList1[4 * (config.run_type - 1) + i]);
            }
        }

        std::string output_path = config.MultiLevelresultFolder + "NEWW_" + std::to_string(value) + "_"
                    + std::to_string(int(numberVehicle[value].first)) + "_" + std::to_string(int(numberVehicle[value].second))
                    + "_" + std::to_string(config.run_type) + "_"
                    + std::to_string(int(config.percent_match*100)) + "%" + "_"
                    + std::to_string(int(config.percent_select*100)) + "%" + ".json";
        std::vector<std::string> coppy = dataList;
        std::cout << std::endl << "Data List = ";
        for (std::string data : dataList){
            std::cout << std::endl << data;
        }
        std::cout << std::endl << "Output path = " << output_path;
        config.numDrone = numberVehicle[value].first;
        config.numTech = numberVehicle[value].second;

        json logDataSet;

        for (int ite = 0; ite < dataList.size(); ite++)
        {
            std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, dataList[ite]);
            json logData;
            for (const std::string &path : paths)
            {
                Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
                double best = 9999999;
                double average = 0;
                std::vector<double> acc;
                if (config.overwrite == false){
                    std::ifstream file(output_path);
                    if (file) {
                        file >> logDataSet;
                        file.close();

                        if (logDataSet.find(input.dataSet + ".txt") != logDataSet.end()) {
                            continue;
                        }
                    }
                }
                for (int run = 0; run < config.NumRunPerDataSet; run++)
                {
                    config.tabuMaxIter = std::max(50, input.numCus/2);
                    config.num_level = 3;
                    Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
                    std::cout << "Run set: " << path << "." << run << std::endl;

                    // --- THAY ĐỔI: Sử dụng Solution::initSolution trực tiếp để kiểm tra tính khả thi ---
                    // Thay vì gọi TabuSearch tabuSearch(config, input);
                    Solution* initSol = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);

                    if (initSol == nullptr || (initSol->droneTripList.empty() && initSol->techTripList.empty()))
                        {
                            std::cout << "Infeasible!" << std::endl;
                            delete initSol;
                            return 1;
                        }
                    delete initSol; // Giải phóng bộ nhớ sau khi kiểm tra
                    // ----------------------------------------------------------------------------------

                    json log;
                    log["Num_Drone: "] = config.numDrone;
                    log["Num_Tech: "] = config.numTech;

                    std::string path_e;
                    MultiLevel multilev(config, input);
                    auto begin = high_resolution_clock::now();
                    std::vector<double> score_list;
                    std::tuple<double, Solution, std::vector<double>> result = multilev.run(config, input);
                    score_list = std::get<2> (result);
                    std::vector<double> Listt;
                    std::string Scoree = "Level 0: ";
                    Listt.push_back(score_list[0]);
                    Scoree = Scoree + std::to_string(score_list[0]) + " | ";
                    for (int i = 1; i <= config.num_level + 1; i++){
                        Listt.push_back((score_list[i] - score_list[i-1]) / score_list[i-1]);
                        if (i == config.num_level + 1){
                            Scoree = Scoree + "Post Optimization: " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                        }else{
                            Scoree = Scoree + "Level " + std::to_string(i) + ": " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                        }
                    }
                    for (int i = config.num_level + 2; i < score_list.size(); i++){
                        Listt.push_back((score_list[i] - score_list[i-1]) / score_list[i-1]);
                        if (i == score_list.size() - 1){
                            Scoree = Scoree + "Post Optimization: " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                        }else{
                            Scoree = Scoree + "Level " + std::to_string(score_list.size() - i - 2 ) + ": " + std::to_string((score_list[i] - score_list[i-1]) / score_list[i-1]) + " | ";
                        }
                    }
                    std::cout<<std::endl<<"Score List = ";
                    for (int b = 0; b < score_list.size(); b++){
                        std::cout<<score_list[b]<<" | ";
                    }
                    Solution bestSol = std::get<1> (result);

                    std::cout << std::endl;
                    auto end = high_resolution_clock::now();
                    json jDroneBest(bestSol.droneTripList);
                    json jTechBest(bestSol.techTripList);
                    if (std::get<0> (result) < best)
                    {
                        best = std::get<0> (result);
                    }
                    acc.push_back(std::get<0> (result));
                    average += std::get<0> (result);
                    log["Best Score: "] = std::get<0> (result);
                    std::cout<<"Best score: " <<std::get<0> (result);
                    log["Best Solution: "] = std::to_string(std::get<0> (result)) + " == " + jDroneBest.dump() + " || " + jTechBest.dump();
                    log["Run Time: "] = duration_cast<milliseconds> (end - begin).count() / 1000.0;
                    log["Score List: "] = Scoree;
                    log["num level"] = config.num_level;
                    std::string name = dataList[ite] + "." + std::to_string(run + 1);
                    logData[name] = log;
                }
                double std = 0;
                average = average / config.NumRunPerDataSet;
                if (config.NumRunPerDataSet == 1){
                    std = 0;
                }else{
                    for (int h = 0; h < acc.size(); h++){
                        std = std + pow(acc[h] - average, 2);
                    }
                    std = std/(config.NumRunPerDataSet - 1);
                    std = sqrt(std);
                }
                std::cout<<std::endl<<"Dataset is: "<<input.dataSet;
                logDataSet[dataList[ite]] = logData;
                std::string tempp = input.dataSet + " | Best: ";
                std::string tempu = input.dataSet + " | Average: ";
                std::string tempv = input.dataSet + " | STD: ";
                logDataSet[tempp] = best;
                logDataSet[tempu] = average;
                logDataSet[tempv] = std;

                std::ofstream output_file(output_path);
                output_file << std::setw(4) << logDataSet << std::endl;
                output_file.close();

            }
        }

    }

    return 0;
}
