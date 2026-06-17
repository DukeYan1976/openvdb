#include "RtDebugSys.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <filesystem>
#include <sstream>

namespace RtDebugSys {
    Debugger* Debugger::_instance = nullptr;
    std::mutex Debugger::_instanceMutex;

    Debugger& Debugger::GetInstance() {
        std::lock_guard<std::mutex> lock(_instanceMutex);
        if (!_instance) _instance = new Debugger();
        return *_instance;
    }

    Debugger::Debugger() : _debugLevel(0), _stopThread(false) {
        _lastCheckTime = std::chrono::steady_clock::now();
        _flushThread = std::thread(&Debugger::_FlushThreadWorker, this);
    }

    Debugger::~Debugger() {
        _stopThread = true;
        _cv.notify_all();
        if (_flushThread.joinable()) _flushThread.join();
    }

    void Debugger::SetWorkspace(const std::string& path) {
        std::filesystem::path p(path);
        if (p.empty()) return;
        
        if (std::filesystem::is_directory(p) || !p.has_extension()) {
            _cfgPath = (p / "RTDebug.cfg").string();
            _logPath = (p / "DebugInfo.txt").string();
        } else {
            _cfgPath = p.string();
            _logPath = (p.parent_path() / "DebugInfo.txt").string();
        }
        
        _ReadConfigFile();
    }
    
    bool Debugger::IsTagActive(const std::string& tag) { 
        if (_debugLevel.load() == 0) return false;

        _ReloadConfigIfNeeded();

        {
            std::shared_lock<std::shared_mutex> lock(_tagsMutex);
            auto it = _tagsMap.find(tag);
            if (it != _tagsMap.end()) return it->second;
        }

        // Auto-discovery: Tag not found, append to config as inactive
        std::lock_guard<std::shared_mutex> lock(_tagsMutex);
        auto it = _tagsMap.find(tag);
        if (it == _tagsMap.end()) {
            _tagsMap[tag] = false;
            if (!_cfgPath.empty()) {
                std::ofstream outFile(_cfgPath, std::ios::app);
                if (outFile.is_open()) {
                    outFile << "\n#" << tag;
                }
            }
            return false;
        }
        return it->second; 
    }
    
    bool Debugger::IsRtDebugActive() { 
        return _debugLevel.load() > 0; 
    }

    bool Debugger::IsVerbose() {
        return _debugLevel.load() >= 2;
    }
    
    void Debugger::OutputDebugInfo(const std::string& info, bool isTimeStamp) {
        if (_debugLevel.load() == 0) return;
        
        std::string finalMsg = isTimeStamp ? "[" + _GetTimestamp() + "] " + info : info;
        {
            std::lock_guard<std::mutex> lock(_queueMutex);
            _msgQueue.push(finalMsg);
        }
        _cv.notify_one();
    }
    
    void Debugger::Activate(int level) { 
        _debugLevel.store(level); 
    }
    
    void Debugger::Deactivate() { 
        _debugLevel.store(0); 
    }
    
    void Debugger::_ReloadConfigIfNeeded() {
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastCheckTime).count() < 500) return;
        
        _lastCheckTime = now;
        if (_cfgPath.empty() || !std::filesystem::exists(_cfgPath)) return;

        try {
            auto ftime = std::filesystem::last_write_time(_cfgPath);
            if (ftime != _lastWriteTime) {
                _ReadConfigFile();
            }
        } catch (...) {
            // File might be busy
        }
    }
    
    void Debugger::_ReadConfigFile() {
        if (_cfgPath.empty() || !std::filesystem::exists(_cfgPath)) return;

        std::ifstream inFile(_cfgPath);
        if (!inFile.is_open()) return;

        std::unordered_map<std::string, bool> newTagsMap;
        std::string line;
        int newLevel = 0;
        bool firstLine = true;

        while (std::getline(inFile, line)) {
            // Basic trim
            line.erase(0, line.find_first_not_of(" \t\r\n"));
            line.erase(line.find_last_not_of(" \t\r\n") + 1);

            if (line.empty()) continue;

            if (firstLine) {
                if (line == "2" || line == "VERBOSE") {
                    newLevel = 2;
                } else if (line == "1" || line == "true" || line == "ON") {
                    newLevel = 1;
                } else {
                    newLevel = 0;
                }
                firstLine = false;
                continue;
            }

            if (line[0] == '#') {
                std::string tag = line.substr(1);
                tag.erase(0, tag.find_first_not_of(" \t\r\n"));
                tag.erase(tag.find_last_not_of(" \t\r\n") + 1);
                if (!tag.empty()) newTagsMap[tag] = false;
            } else {
                newTagsMap[line] = true;
            }
        }

        _debugLevel.store(newLevel);
        
        {
            std::lock_guard<std::shared_mutex> lock(_tagsMutex);
            _tagsMap = std::move(newTagsMap);
        }

        try {
            _lastWriteTime = std::filesystem::last_write_time(_cfgPath);
        } catch (...) {}
    }
    
    void Debugger::_FlushThreadWorker() {
        while (true) {
            std::unique_lock<std::mutex> lock(_queueMutex);
            _cv.wait(lock, [this] { return !_msgQueue.empty() || _stopThread; });

            if (_stopThread && _msgQueue.empty()) break;

            std::queue<std::string> localQueue;
            std::swap(localQueue, _msgQueue);
            lock.unlock();

            if (!localQueue.empty() && !_logPath.empty()) {
                std::ofstream logFile(_logPath, std::ios::app);
                if (logFile.is_open()) {
                    while(!localQueue.empty()) {
                        logFile << localQueue.front() << "\n";
                        localQueue.pop();
                    }
                }
            }
        }
    }
    
    std::string Debugger::_GetTimestamp() { 
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::tm bt;
#if defined(_WIN32)
        localtime_s(&bt, &in_time_t);
#else
        localtime_r(&in_time_t, &bt);
#endif

        std::ostringstream ss;
        ss << std::put_time(&bt, "%Y-%m-%d %H:%M:%S")
           << "." << std::setfill('0') << std::setw(3) << ms.count();
        return ss.str();
    }
}

