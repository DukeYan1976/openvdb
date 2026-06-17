#ifndef RT_DEBUG_SYS_H
#define RT_DEBUG_SYS_H

#include <string>
#include <unordered_map>
#include <filesystem>
#include <shared_mutex>
#include <mutex>
#include <atomic>
#include <queue>
#include <thread>
#include <condition_variable>

namespace RtDebugSys {
    class Debugger {
    public:
        static Debugger& GetInstance();
        bool IsTagActive(const std::string& tag);
        bool IsRtDebugActive();
        bool IsVerbose();
        void OutputDebugInfo(const std::string& info, bool isTimeStamp = true);
        void SetWorkspace(const std::string& path);
        
        // Manual control
        void Activate(int level = 1);
        void Deactivate();

    private:
        Debugger();
        ~Debugger();
        void _ReloadConfigIfNeeded();
        void _ReadConfigFile();
        void _FlushThreadWorker();
        std::string _GetTimestamp();

        static Debugger* _instance;
        static std::mutex _instanceMutex;

        std::atomic<int> _debugLevel{0}; // 0: Off, 1: On, 2: Verbose
        std::string _cfgPath;
        std::string _logPath;
        
        std::unordered_map<std::string, bool> _tagsMap;
        mutable std::shared_mutex _tagsMutex;

        std::filesystem::file_time_type _lastWriteTime;
        std::chrono::steady_clock::time_point _lastCheckTime;

        // Async Logger
        std::queue<std::string> _msgQueue;
        std::mutex _queueMutex;
        std::condition_variable _cv;
        std::thread _flushThread;
        std::atomic<bool> _stopThread{false};
    };
}

#define DEBUG_SECTION(_tag) if (RtDebugSys::Debugger::GetInstance().IsTagActive(#_tag))
#define DEBUG_SECTION_VERBOSE(_tag) if (RtDebugSys::Debugger::GetInstance().IsVerbose() && RtDebugSys::Debugger::GetInstance().IsTagActive(#_tag))
#define CHECK_SECTION() if (RtDebugSys::Debugger::GetInstance().IsRtDebugActive())
#define DEBUG_INFO_OUT(_info) RtDebugSys::Debugger::GetInstance().OutputDebugInfo(_info, true)
#define DEBUG_INFO_OUT_NO_TIME(_info) RtDebugSys::Debugger::GetInstance().OutputDebugInfo(_info, false)

#endif
