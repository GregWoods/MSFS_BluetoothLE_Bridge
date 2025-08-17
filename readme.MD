create build configuration (not needed now I have presets)
```cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug```

Now, if CmakeLists.txt or cache variables change, I run
```cmake --preset debug ``` or ```cmake --preset release```



build project
```cmake --build --preset debug```
or
```cmake --build --preset release```