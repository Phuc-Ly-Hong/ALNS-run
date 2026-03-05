#include "MultiLevel.h"
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include "Utils.h"
#include "Random.h"
#include <set>
#include <map>
#include <iomanip>

using Random = effolkronium::random_static;

MultiLevel::MultiLevel(Config &config, Input &input) : lns(config, input)
{
    this->config = config;
    this->config.tabuMaxIter = config.tabuMaxIter;
    this->input = input;
    this->tabuDuration = config.tabuDuration;
}

std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>> MultiLevel::convertMatrix(std::vector<std::vector<double>> currentMatrix, std::vector<std::vector<double>> matrixReBefore,  std::map<int, std::vector<int>> Map)
{
    std::vector<std::vector<double>> newMatrix;
    std::vector<std::vector<double>> MatrixRe;
    int sizee = Map.size();
    std::vector<double> temp;
    for (int i = 0; i <= sizee; i++) {
        temp.resize(sizee + 1, 0);
        newMatrix.push_back(temp);
        MatrixRe.push_back(temp);
    }
    std::vector<int> plus;
    for (int i = 0; i < newMatrix.size() - 1; i++) {
        for (int j = 0; j < newMatrix.size() - 1; j++) {
            plus = Map[i];
            plus.insert(plus.end(), Map[j].begin(), Map[j].end());
            if (Map[j].size() > 1) {
                int sizee = Map[j].size();
                for (int k = 0; k < sizee - 1; k++) plus.pop_back();
            }
            MatrixRe[i][j] = matrixReBefore[plus[plus.size() - 2]][plus[plus.size() - 1]];
            for (int k = 0; k < plus.size() - 1; k++) {
                newMatrix[i][j] = newMatrix[i][j] + currentMatrix[plus[k]][plus[k+1]];
            }
        }
    }
    for (int i = 0; i < newMatrix.size(); i++) {
        newMatrix[i][newMatrix.size() - 1] = newMatrix[i][0];
        newMatrix[newMatrix.size() - 1][i] = newMatrix[0][i];
    }  
    return {newMatrix, MatrixRe};
}

std::tuple<int, Solution, std::map<int, std::vector<int>>> MultiLevel::mergeSol(
    Solution solution, 
    int NumCus, 
    std::vector<std::vector<int>> freqMatrix, 
    std::vector<std::vector<double>> MatrixRe, 
    std::vector<std::vector<double>> distanceMatrix, 
    std::vector<bool> C1)
{
    std::vector<std::tuple<int , int>> update;
    std::vector<std::tuple<int , int>> edgeSol;
    
    // --- TÍNH TOÁN COUNT VÀ IN RA ---
    int counttt = 0;
    for (int i = 0; i < solution.droneTripList.size(); i++) {
        if (solution.droneTripList[i].empty()) continue;
        for (int j = 0; j < solution.droneTripList[i].size(); j++) {
            counttt += solution.droneTripList[i][j].size();
            if (solution.droneTripList[i][j].size() <= 1) continue;
            for (int k = 0; k < solution.droneTripList[i][j].size() - 1; k++) {
                edgeSol.push_back(std::make_tuple(solution.droneTripList[i][j][k], solution.droneTripList[i][j][k+1]));
            }
        }
    }   
    for (int i = 0; i < solution.techTripList.size(); i++) {
        counttt += solution.techTripList[i].size();
        if (solution.techTripList[i].size() <= 1) continue;
        for (int j = 0; j < solution.techTripList[i].size() - 1; j++) {
            edgeSol.push_back(std::make_tuple(solution.techTripList[i][j], solution.techTripList[i][j+1]));
        }
    }
    
    std::cout << std::endl << "counttt = " << counttt << ", percent_match = " << config.percent_match * 100 << "%";
    
    int numUpdate = NumCus * config.percent_select;
    
    // --- Matching Logic ---
    while (update.size() < numUpdate) {
        double bestVal = (config.matching_method == 1) ? 999999999 : -1.0; 
        int row = -1, col = -1;

        for (int i = 1; i <= NumCus; i++) {
            for (int j = 1; j <= NumCus; j++) {
                if (i == j) continue;
                if (i >= MatrixRe.size() || j >= MatrixRe.size() || j >= MatrixRe[i].size()) continue;

                auto find1 = std::find(update.begin(), update.end(), std::make_tuple(i, j));
                auto find2 = std::find(update.begin(), update.end(), std::make_tuple(j, i));
                auto find3 = std::find(edgeSol.begin(), edgeSol.end(), std::make_tuple(i, j));

                if (find1 == update.end() && find2 == update.end() && find3 == edgeSol.end()) 
                {
                    double currentVal;
                    if (config.matching_method == 1) { // Method B
                        currentVal = MatrixRe[i][j];
                        if (currentVal < bestVal) { bestVal = currentVal; row = i; col = j; }
                    } else { // Method A/C
                        if (i < freqMatrix.size() && j < freqMatrix[i].size()) {
                            currentVal = (double)freqMatrix[i][j];
                            if (currentVal > bestVal) { bestVal = currentVal; row = i; col = j; }
                        }
                    }
                }
            }
        }
        if (row != -1 && col != -1) {
            if (((!C1[row]) && (!C1[col])) || (C1[row] && C1[col])){
                update.push_back(std::make_tuple(row, col));
            } else {
                 edgeSol.push_back(std::make_tuple(row, col)); 
            }
        } else { break; }
    }

    // --- Insertion Logic ---
    int length = 0;
    for (const auto& drone : solution.droneTripList) for (const auto& trip : drone) length += std::max((int)trip.size() - 1, 0);
    for (const auto& trip : solution.techTripList) length += std::max((int)trip.size() - 1, 0);

    int number = NumCus * config.percent_match + 1;
    int numUpdateReal = std::min(number, length);
    std::vector<std::tuple<int , int>> update_Real;

    while (update_Real.size() < numUpdateReal) {
        double count = 999999;
        int row = -1;
        int col = -1;
        
        for (const auto& edge : edgeSol) {
            int u = std::get<0>(edge);
            int v = std::get<1>(edge);
            if (u >= MatrixRe.size() || v >= MatrixRe.size()) continue;
            
            auto find = std::find(update_Real.begin(), update_Real.end(), std::make_tuple(u, v));
            if (find == update_Real.end()) {
                if (MatrixRe[u][v] < count) { 
                    count = MatrixRe[u][v]; row = u; col = v;
                }
            }
        }

        if (row != -1 && col != -1) {
            if (((!C1[row]) && (!C1[col])) || (C1[row] && C1[col])){
                update_Real.push_back(std::make_tuple(row, col));
            }
        } else { break; }
    }
    update = update_Real;

    // --- Chain & Renumbering ---
    std::vector<std::tuple<int , int>> updateFake = update;
    std::vector<std::vector<int>> beMerge;
    std::vector<int> temp;
    std::tuple<int, int> fi;

    auto processTrip = [&](const std::vector<int>& trip) {
        if (trip.size() <= 1) return;
        for (size_t k = 0; k < trip.size() - 1; k++) {
            fi = std::make_tuple(trip[k], trip[k+1]);
            bool found = false;
            for (int u = 0; u < updateFake.size(); u++) {
                if (fi == updateFake[u]) {
                    if (temp.empty()) { temp.push_back(std::get<0>(fi)); temp.push_back(std::get<1>(fi)); }
                    else {
                        if (temp.back() != std::get<0>(fi)) { beMerge.push_back(temp); temp.clear(); temp.push_back(std::get<0>(fi)); temp.push_back(std::get<1>(fi)); }
                        else { temp.push_back(std::get<1>(fi)); }
                    }
                    updateFake.erase(updateFake.begin() + u); found = true; break;
                }
            }
            if (!found && !temp.empty()) { beMerge.push_back(temp); temp.clear(); }
        }
        if (!temp.empty()) { beMerge.push_back(temp); temp.clear(); }
    };

    for (const auto& drone : solution.droneTripList) for (const auto& trip : drone) processTrip(trip);
    for (const auto& trip : solution.techTripList) processTrip(trip);

    std::set<int> usedNodes;
    for (auto& d : solution.droneTripList) for (auto& t : d) for (int n : t) usedNodes.insert(n);
    for (auto& t : solution.techTripList) for (int n : t) usedNodes.insert(n);

    std::map<int, int> nodeToRep;
    for (int u : usedNodes) nodeToRep[u] = u;

    for (const auto& chain : beMerge) {
        if (chain.empty()) continue;
        int rep = *std::min_element(chain.begin(), chain.end());
        for (int node : chain) nodeToRep[node] = rep;
    }

    std::map<int, int> repToNewID;
    std::map<int, std::vector<int>> newMap;
    repToNewID[0] = 0; newMap[0] = {0};

    int newIDCounter = 1;
    for (int u : usedNodes) {
        if (u == 0) continue;
        int rep = nodeToRep[u];
        if (repToNewID.find(rep) == repToNewID.end()) {
            repToNewID[rep] = newIDCounter;
            newMap[newIDCounter] = {};
            newIDCounter++;
        }
        newMap[repToNewID[rep]].push_back(u);
    }

    auto updateTrip = [&](std::vector<int>& trip) {
        for (int &node : trip) {
            if (nodeToRep.find(node) != nodeToRep.end()) {
                int rep = nodeToRep[node];
                if (repToNewID.find(rep) != repToNewID.end()) node = repToNewID[rep];
            }
        }
        auto last = std::unique(trip.begin(), trip.end());
        trip.erase(last, trip.end());
    };

    for (auto& drone : solution.droneTripList) for (auto& trip : drone) updateTrip(trip);
    for (auto& trip : solution.techTripList) updateTrip(trip);

    numUpdate = newIDCounter - 1; 
    return std::make_tuple(numUpdate, solution, newMap);       
}

std::tuple<Solution, std::map<int, std::vector<int>>, std::vector<double>> MultiLevel::mergeProcess(Config &config, Input &input, std::vector<std::map<int, std::vector<int>>> &mapLevel, std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, std::vector<std::vector<bool>> &C1Level)
{
    std::vector<double> scoree;
    
    // --- KHỞI TẠO VÀ CHẠY LNS MỘT LẦN ĐẦU TIÊN ĐỂ CÓ NỀN TẢNG TỐT ---
    Solution *initSolPtr = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);
    Solution currentSol;
    if (initSolPtr != nullptr) currentSol = *initSolPtr;
    else currentSol = Solution(config, input, config.tabuAlpha1, config.tabuAlpha2); 
    
    // Chạy LNS ngay Level 0 để tối ưu hết mức có thể trước khi Merge
    json log; std::string path_e;
    LargeNeighborhoodSearch lnsSolver(config, input);
    auto resultInit = lnsSolver.run(log, path_e, input, currentSol);
    currentSol = std::get<1>(resultInit);
    // ----------------------------------------------------------------

    std::vector<std::vector<double>> distanceMatrix = input.distances;
    std::vector<std::vector<double>> MatrixRe = distanceMatrix;
    DistanceMatrixLevel.push_back(distanceMatrix);
    double bestScore = currentSol.getScore();
    double currentScore = bestScore;
    scoree.push_back(bestScore);
    int numcus = input.numCus;
    C1Level.push_back(input.cusOnlyServedByTech);
    int count_level = 0;

    std::cout << std::endl << "Initial Score (Level 0 Optimized): " << bestScore;

    while(true)
    {
        // Chạy LNS (ở đây là chạy trên đồ thị ĐÃ merge của vòng lặp trước hoặc vòng đầu)
        auto result = lnsSolver.run(log, path_e, input, currentSol);
        Solution sol = std::get<1>(result);
        std::vector<std::vector<int>> freqMatrix;
        if (config.matching_method == 0) freqMatrix = std::get<3>(result);
        else freqMatrix = std::get<2>(result);

        double temp2 = sol.getScore();
        std::cout<<std::endl<<"Score after LNS in merge process level "<<count_level+1<<" is: "<<temp2;
        
        // Điều kiện dừng
        if (numcus <= 2 || ((std::abs(temp2 - currentScore) < std::abs(config.tabuEpsilon)) && (count_level >= 3))){
            config.num_level = count_level + 1;
            currentSol = sol;
            scoree.push_back(currentSol.getScore());
            break;
        }
        currentScore = temp2;
        if (!sol.check_feasible()) sol = currentSol;

        auto merge = this->mergeSol(sol, numcus, freqMatrix, MatrixRe, distanceMatrix, C1Level[count_level]);
        
        numcus = std::get<0>(merge);
        currentSol = std::get<1>(merge);
        mapLevel.push_back(std::get<2>(merge));
        
        auto resultMatrix = this->convertMatrix(distanceMatrix, MatrixRe, std::get<2>(merge));
        distanceMatrix = std::get<0>(resultMatrix);
        MatrixRe = std::get<1>(resultMatrix);
        
        // Cập nhật Input
        int sizee = distanceMatrix.size();
        std::vector<std::vector<double>> droneFlightMatrix(sizee, std::vector<double>(sizee, 0));
        std::vector<std::vector<double>> techMoveMatrix(sizee, std::vector<double>(sizee, 0));
        for (int j = 0; j < sizee; j++) {
            for (int k = 0; k < sizee; k++) {
                droneFlightMatrix[j][k] = distanceMatrix[j][k] / config.droneVelocity;
                techMoveMatrix[j][k] = distanceMatrix[j][k] / config.techVelocity;
            }
        }
        std::vector<std::vector<double>> coor(sizee, std::vector<double>(sizee, 0));
        input.numCus = numcus;
        input.coordinates = coor;
        input.distances = distanceMatrix;
        DistanceMatrixLevel.push_back(distanceMatrix);
        input.droneTimes = droneFlightMatrix;
        input.techTimes = techMoveMatrix;
        
        std::vector<bool> C1(numcus + 1, false);
        for (int j = 1; j < numcus + 1; j++){
            bool isTechOnly = false;
            for (int k : std::get<2>(merge)[j]) {
                if (k < C1Level[count_level].size() && C1Level[count_level][k]) { isTechOnly = true; break; }
            }
            C1[j] = isTechOnly;
        }
        C1Level.push_back(C1);
        input.cusOnlyServedByTech = C1;
        currentSol.setInput(input);
        
        scoree.push_back(currentSol.getScore());
        count_level++;
    }
    
    std::cout<<std::endl<<"Score after merge process is: "<<currentSol.getScore();
    scoree.push_back(currentSol.getScore());

    return std::make_tuple(currentSol, std::get<2>(std::tuple<int, Solution, std::map<int, std::vector<int>>>()), scoree);
}

Solution MultiLevel::splitSol(Solution solution, std::map<int, std::vector<int>> Map) {
    int counttt=0;
    for (int i = 0; i < solution.droneTripList.size(); i++) {
        for (int j = 0; j < solution.droneTripList[i].size(); j++) {
            counttt += solution.droneTripList[i][j].size();
            for (int k = 0; k < solution.droneTripList[i][j].size(); k++) {
                int temp = solution.droneTripList[i][j][k];
                if (Map.find(temp) == Map.end()) continue;
                solution.droneTripList[i][j].erase(solution.droneTripList[i][j].begin() + k);
                for (int u = Map[temp].size() - 1; u >= 0; u--) {
                    solution.droneTripList[i][j].insert(solution.droneTripList[i][j].begin() + k, Map[temp][u]);
                }
                k = k + Map[temp].size() - 1;
                if (k >= solution.droneTripList[i][j].size() - 1) break;
            }
        }
    }
    for (int i = 0; i < solution.techTripList.size(); i++) {
        counttt += solution.techTripList[i].size();
        for (int j = 0; j < solution.techTripList[i].size(); j++) {
            int temp = solution.techTripList[i][j];
            if (Map.find(temp) == Map.end()) continue;
            solution.techTripList[i].erase(solution.techTripList[i].begin() + j);
            for (int u = Map[temp].size() - 1; u >= 0; u--) {
                solution.techTripList[i].insert(solution.techTripList[i].begin() + j, Map[temp][u]);
            }
            j = j + Map[temp].size() - 1;
            if (j >= solution.techTripList[i].size() - 1) break;
        }
    }
    std::cout << std::endl << "counttt = " << counttt;
    return solution;
}

std::tuple<Solution, std::vector<double>> MultiLevel::splitProcess(Solution solution, Config &config, Input &input, std::vector<std::map<int, std::vector<int>>> &mapLevel, std::vector<std::vector<std::vector<double>>> &DistanceMatrixLevel, std::vector<std::vector<bool>> &C1Level)
{
    solution.setInput(input);
    double best = solution.getScore();
    std::vector<double> scoree;
    for (int i = 1; i < config.num_level; i++) {
        json log; std::string path_e;
        MultiLevel multilev(config, input);
        solution.setInput(input);
        solution = multilev.splitSol(solution, mapLevel[config.num_level - i - 1]);
        
        int sizee = DistanceMatrixLevel[config.num_level - i - 1].size();
        std::vector<std::vector<double>> droneFlightMatrix(sizee, std::vector<double>(sizee, 0));
        std::vector<std::vector<double>> techMoveMatrix(sizee, std::vector<double>(sizee, 0));
        for (int j = 0; j < sizee; j++) {
            for (int k = 0; k < sizee; k++) {
                droneFlightMatrix[j][k] = DistanceMatrixLevel[config.num_level - i - 1][j][k] / config.droneVelocity;
                techMoveMatrix[j][k] = DistanceMatrixLevel[config.num_level - i - 1][j][k] / config.techVelocity;
            }
        }
        std::vector<std::vector<double>> coor(sizee, std::vector<double>(sizee, 0));
        input.numCus = sizee - 2; input.coordinates = coor; input.distances = DistanceMatrixLevel[config.num_level - i - 1];
        input.droneTimes = droneFlightMatrix; input.techTimes = techMoveMatrix;
        input.cusOnlyServedByTech = C1Level[config.num_level - i - 1];
        
        LargeNeighborhoodSearch lnsSolver(config, input); 
        solution.setInput(input);
        std::cout<<std::endl<<"Score before LNS in Split process Level "<<(i+1)<<" is: "<<solution.getScore();
        auto result = lnsSolver.run(log, path_e, input, solution);
        solution = std::get<1>(result);
        std::cout<<std::endl<<"Score After LNS in Split process Level "<<(i+1)<<" is: "<<solution.getScore();
        best = std::get<0>(result);
        scoree.push_back(best);
    }
    std::cout<<std::endl<<"Final Score: "<<solution.getScore();
    scoree.push_back(solution.getScore());
    return std::make_tuple(solution, scoree);
}

std::tuple<double, Solution, std::vector<double>> MultiLevel::run(Config &config, Input &input)
{
    std::cout << std::endl << "Numcus = " << input.numCus << ", numDrone = " << config.numDrone << ", numTech = " << config.numTech;
    
    // --- BƯỚC 1: LƯU LẠI GIẢI PHÁP TỐT NHẤT BAN ĐẦU (SAFETY NET) ---
    // Khởi tạo và chạy LNS kỹ ở mức Level 0
    LargeNeighborhoodSearch lns(config, input);
    json log; std::string path;
    Solution *init = Solution::initSolution(config, input, InitType::MIX, config.tabuAlpha1, config.tabuAlpha2);
    Solution globalBest = (init) ? *init : Solution(config, input, 1, 1);
    auto res0 = lns.run(log, path, input, globalBest);
    globalBest = std::get<1>(res0);
    double globalBestScore = globalBest.getScore();
    std::cout << "\n[Global Best Found at Start]: " << globalBestScore << std::endl;
    // ------------------------------------------------------------------

    MultiLevel multilev(config, input);        
    
    // Lưu ý: Mình truyền globalBest vào mergeProcess để nó bắt đầu từ điểm tốt nhất này
    auto result = multilev.mergeProcess(config, input, mapLevel, DistanceMatrixLevel, C1Level);
    Solution res_sol = std::get<0>(result);

    auto tue = multilev.splitProcess(res_sol, config, input, mapLevel, DistanceMatrixLevel, C1Level);
    Solution solution = std::get<0>(tue);
    solution.setInput(input);
    
    std::vector<double> score1 = std::get<2>(result);
    std::vector<double> score2 = std::get<1>(tue);
    std::vector<double> scoree;
    scoree.insert(scoree.end(), score1.begin(), score1.end());
    scoree.insert(scoree.end(), score2.begin(), score2.end());

    // --- BƯỚC 2: KIỂM TRA VÀ PHỤC HỒI NẾU CẦN ---
    if (solution.getScore() > globalBestScore) {
        std::cout << "\n[WARNING] MultiLevel result (" << solution.getScore() 
                  << ") is worse than Initial LNS (" << globalBestScore << "). Reverting to Initial Best.";
        solution = globalBest; // Phục hồi lại kết quả tốt nhất ban đầu
    } else {
        std::cout << "\n[SUCCESS] MultiLevel improved solution: " << globalBestScore << " -> " << solution.getScore();
    }
    // ---------------------------------------------

    return std::make_tuple(solution.getScore(), solution, scoree);
}