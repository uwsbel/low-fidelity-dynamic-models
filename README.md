# low-fidelity-dynamic-models
A library of fast and accurate low fidelity dynamic models for applications in robotics


## Contribution Guidelines
### The obvious
1) **Code formatting** - Provided in the root directory of this repository is a _.clang_format_ which helps ensure consistent formatting of source code. Before making a contribution, please ensure that the code is appropriately formated by using the format file. If you use vscode, [here](https://marketplace.visualstudio.com/items?itemName=xaver.clang-format) is a extension that can help automate formatting.

2) **Adding data files to git** - The repository uses git lfs in order to track data files of the csv and text format. It is thus okay to add data files to git. However, we only track reference data csv files and not csv files produced as model output. Please ensure that these are not added and commited. The _.gitingore_ automatically ignores all files in the _output_ folders/subfolders, thus you might benefit from ensuring that all model output is stored in such a directory. 
