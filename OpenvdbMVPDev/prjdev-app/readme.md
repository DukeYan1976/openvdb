 构建命令（供你后续使用）：

  cmd /c "call "C:\Program Files (x86)\Microsoft Visual
  Studio\2022\BuildTools\VC\Auxiliary\Build\vcvarsall.bat" amd64 >nul 2>&1 && cmake -S
  D:\WorkSpace\02_Develop\SourceCode_2\openvdb\yggdrasil\app -B
  D:\WorkSpace\02_Develop\SourceCode_2\openvdb\yggdrasil\app\build -G Ninja -DCMAKE_BUILD_TYPE=Release
  && cmake --build D:\WorkSpace\02_Develop\SourceCode_2\openvdb\yggdrasil\app\build"