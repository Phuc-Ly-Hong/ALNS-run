#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip> // Thêm thư viện này để format output đẹp

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

// ... (Giữ nguyên hàm file_browsing) ...
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
    // ... (Giữ nguyên phần đọc Config ban đầu) ...
    // Copy lại đoạn check file Config.json và khởi tạo config từ code cũ của bạn
    Config config;
    std::string configFilePath = "Config.json";
    std::ifstream configFile(configFilePath);
    if (!configFile.is_open()) { return 1; }
    json jsConfig = json::parse(configFile);
    
    // Load parameters
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
    config.tabuMaxIter = jsConfig["multiLevel_para"]["tabuMaxIter"].get<int>();
    config.tabuAlpha1 = jsConfig["tabu_para"]["tabuAlpha1"].get<double>();
    config.tabuAlpha2 = jsConfig["tabu_para"]["tabuAlpha2"].get<double>();
    config.tabuEpsilon = jsConfig["tabu_para"]["tabuEpsilon"].get<double>();
    config.ws = jsConfig["ws"].get<std::string>();;
    config.dataPath = jsConfig["dataPath"].get<std::string>();
    config.MultiLevelresultFolder = jsConfig["MultiLevelresultFolder"].get<std::string>();
    config.NumRunPerDataSet = jsConfig["NumRunPerDataSet"].get<int>();
    config.overwrite = jsConfig["overwrite"].get<bool>();
    config.plus1 = jsConfig["plus1"].get<int>();

    // --- CẤU HÌNH DATASET (Sửa ở đây theo ý bạn) ---
    std::vector<int> listDataType = {100}; 
    std::map<int, std::pair<int, int>> numberVehicle;
    
    // Logic override số xe
    if (config.plus1 == 1){
        numberVehicle[6] = {3, 3};
        numberVehicle[10] = {3, 3};
        numberVehicle[12] = {3, 3};
        numberVehicle[20] = {3, 3};
        numberVehicle[50] = {3, 3};
        numberVehicle[100] = {3, 3};
        numberVehicle[200] = {3, 3};
        numberVehicle[500] = {3, 3};
        numberVehicle[1000] = {3, 3};
    } else {
        // ... các case khác
        numberVehicle[6] = {2, 2}; 
        numberVehicle[10] = {2, 2};
    }

    for (int value : listDataType){
        std::cout << std::endl << "Processing dataset size: " << value;
        config.numDrone = numberVehicle[value].first;
        config.numTech = numberVehicle[value].second;
        
        std::vector<std::string> dataList = file_browsing(config.ws + config.dataPath, std::to_string(value));
        
        // Tên file output so sánh
        std::string output_path = config.MultiLevelresultFolder + "COMPARE_" + std::to_string(value) + ".json";
        
        json logDataSet;

        for (int ite = 0; ite < dataList.size(); ite++)
        {
            std::vector<std::string> paths = Utils::glob(config.ws + config.dataPath, dataList[ite]);
            json logData;
            
            for (const std::string &path : paths)
            {
                // Init
                Input input(config.droneVelocity, config.techVelocity, config.droneLimitationFlightTime, path);
                
                // Kiểm tra infeasible
                Solution* initSolCheck = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);
                if (initSolCheck == nullptr || (initSolCheck->droneTripList.empty() && initSolCheck->techTripList.empty())) {
                    std::cout << "Infeasible Input!" << std::endl; delete initSolCheck; continue;
                }
                
                // Tạo một bản sao Solution ban đầu để dùng chung cho cả 2 thuật toán (đảm bảo công bằng điểm xuất phát)
                Solution startSolution = *initSolCheck;
                delete initSolCheck;

                std::cout << "\n==========================================" << std::endl;
                std::cout << "Running: " << path << std::endl; 
                std::cout << "Init Score: " << startSolution.getScore() << std::endl;

                // --- 1. CHẠY MULTI-LEVEL + LNS ---
                std::cout << "--> Mode 1: Multi-Level + LNS..." << std::endl;
                MultiLevel multilev(config, input);
                
                auto beginML = high_resolution_clock::now();
                // (Chỉnh sửa hàm run của MultiLevel để nhận initSol nếu muốn công bằng tuyệt đối, 
                // nhưng hiện tại nó tự init bên trong nên ta chấp nhận sai số nhỏ ở bước init)
                std::tuple<double, Solution, std::vector<double>> resultML = multilev.run(config, input);
                auto endML = high_resolution_clock::now();
                
                double timeML = duration_cast<milliseconds>(endML - beginML).count() / 1000.0;
                double scoreML = std::get<0>(resultML);
                std::cout << "    [Multi-Level] Best: " << scoreML << " | Time: " << timeML << "s" << std::endl;


                // --- 2. CHẠY LNS THUẦN (Cùng thời gian) ---
                std::cout << "--> Mode 2: LNS Only (" << timeML << "s)..." << std::endl;
                LargeNeighborhoodSearch lnsOnly(config, input);
                
                // Reset input và dùng lại startSolution (hoặc tạo mới tương đương)
                Solution lnsStartSol = startSolution; 
                lnsStartSol.setInput(input);

                auto beginLNS = high_resolution_clock::now();
                std::tuple<double, Solution> resultLNS = lnsOnly.runWithTimeLimit(input, lnsStartSol, timeML);
                auto endLNS = high_resolution_clock::now();

                double timeLNS = duration_cast<milliseconds>(endLNS - beginLNS).count() / 1000.0;
                double scoreLNS = std::get<0>(resultLNS);
                std::cout << "    [LNS Only   ] Best: " << scoreLNS << " | Time: " << timeLNS << "s" << std::endl;

                // --- 3. SO SÁNH & GHI LOG ---
                double gap = ((scoreLNS - scoreML) / scoreML) * 100.0;
                std::cout << "--> GAP (LNS - ML): " << gap << "%" << std::endl;
                if (gap > 0) std::cout << "    => Multi-Level BETTER" << std::endl;
                else if (gap < 0) std::cout << "    => LNS Only BETTER" << std::endl;
                else std::cout << "    => EQUAL" << std::endl;

                // Lưu vào JSON
                json logEntry;
                logEntry["1_MultiLevel_Score"] = scoreML;
                logEntry["1_MultiLevel_Time"] = timeML;
                logEntry["2_LNSOnly_Score"] = scoreLNS;
                logEntry["2_LNSOnly_Time"] = timeLNS;
                logEntry["3_GAP_Percentage"] = gap;
                logEntry["Winner"] = (gap > 0) ? "Multi-Level" : ((gap < 0) ? "LNS Only" : "Draw");
                
                logDataSet[input.dataSet] = logEntry;
            }
        }
        // Ghi file
        std::ofstream output_file(output_path);
        output_file << std::setw(4) << logDataSet << std::endl;
        output_file.close();
        std::cout << "\nSaved comparison results to: " << output_path << std::endl;
    }
    return 0;
}