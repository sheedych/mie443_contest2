## Simulation Launch Instructions

## Regular Launch Instructions

## Regenerating Compile Commands
Use the following command to regenerate the compile commands used by VSCode for code linting.
```
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && mv build/compile_commands.json src/mie443_contest2
```
This should be done every time a new file is added to the project. To apply linting in VSCode, add the following line to ```.vscode/c_cpp_properties.json``` under the configurations section: ```"compileCommands": "${workspaceFolder}/compile_commands.json"```

## Building a Single Package
From time to time it may be useful to build just a single package in your ```catkin_ws/src/``` directory. To do so, run:
```
catkin_make --only-pkg-with-deps <target_package>
```
This sets a variable in ```CMakeCache.txt``` that whitelists the package specified. This whitelist persists between calls to ```catkin_make```, and can only be cleared with the following command:
```
catkin_make -DCATKIN_WHITELIST_PACKAGES=""
```

## Creating a New Code File
To add a new pair of .cpp and .hpp files to the package, add the .cpp file to ```src/``` and the .hpp file to ```include/```. Then, in ```CMakeLists.txt```, append the name of the file to the list of files in the ```add_executable()``` line