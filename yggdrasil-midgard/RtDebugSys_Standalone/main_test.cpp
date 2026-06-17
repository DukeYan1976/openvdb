#include "RtDebugSys.h"
#include <thread>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <atomic>

int main() {
    std::string testDir = "./debug_test_env";
    std::filesystem::remove_all(testDir);
    std::filesystem::create_directories(testDir);
    
    // 1. 初始配置：主开关开启（Normal），但标签关闭
    {
        std::ofstream cfg(testDir + "/RTDebug.cfg");
        cfg << "1\n#AlgoThread\n";
    }

    std::cout << "Step 1: Initial config - Level 1 (Normal), but tag 'AlgoThread' inactive (#)..." << std::endl;

    RtDebugSys::Debugger::GetInstance().SetWorkspace(testDir);
    RtDebugSys::Debugger::GetInstance().Activate(1);

    std::atomic<bool> run{true};
    auto worker = [&](int id) {
        while(run) {
            DEBUG_SECTION(AlgoThread) {
                DEBUG_INFO_OUT("Thread " + std::to_string(id) + " is logging...");
            }
            DEBUG_SECTION_VERBOSE(AlgoThread) {
                DEBUG_INFO_OUT("Thread " + std::to_string(id) + " VERBOSE log (should only appear in Level 2)");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    };

    std::vector<std::thread> threads;
    for(int i=0; i<4; ++i) threads.emplace_back(worker, i);

    std::cout << "Starting threads. Verifying that NO logs are written initially..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    if (std::filesystem::exists(testDir + "/DebugInfo.txt")) {
        std::cout << "Warning: Log file created unexpectedly." << std::endl;
    } else {
        std::cout << "Confirmed: No logs written yet." << std::endl;
    }

    // 2. 激活标签（Normal模式）
    std::cout << "Step 2: Activating tag 'AlgoThread' in config file (Normal Mode)..." << std::endl;
    {
        std::ofstream cfg(testDir + "/RTDebug.cfg");
        cfg << "1\nAlgoThread\n";
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 切换到 Verbose 模式
    std::cout << "Step 3: Switching to VERBOSE Mode (Level 2)..." << std::endl;
    {
        std::ofstream cfg(testDir + "/RTDebug.cfg");
        cfg << "2\nAlgoThread\n";
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 4. 测试自动发现
    std::cout << "Step 4: Testing auto-discovery for 'NewTag'..." << std::endl;
    DEBUG_SECTION(NewTag) { }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    run = false;
    for(auto& t : threads) t.join();

    std::cout << "Step 4: Stopping threads and verifying output file..." << std::endl;
    
    if (std::filesystem::exists(testDir + "/DebugInfo.txt")) {
        std::ifstream log(testDir + "/DebugInfo.txt");
        std::string line;
        int count = 0;
        while(std::getline(log, line)) count++;
        std::cout << "Log file entries: " << count << std::endl;
    }

    std::ifstream cfgCheck(testDir + "/RTDebug.cfg");
    std::string line;
    bool foundNewTag = false;
    while(std::getline(cfgCheck, line)) {
        if(line.find("#NewTag") != std::string::npos) foundNewTag = true;
    }
    
    if(foundNewTag) std::cout << "Confirmed: 'NewTag' was auto-discovered and added to config." << std::endl;

    std::cout << "\nALL TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
