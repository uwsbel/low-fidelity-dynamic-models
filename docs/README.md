# Building Documentation with Doxygen

## Prerequisites

Before you begin, you need to have Doxygen installed on your system. Doxygen is a tool that generates documentation from annotated source code. If you don't have Doxygen installed, you can download it from [the official Doxygen website](http://www.doxygen.nl/download.html) or install it using a package manager on your operating system.

For example, on Ubuntu, you can install Doxygen using the following command:
```bash
sudo apt-get install doxygen
```
## Building Documentation
Once you have `doxygen` installed and set-up, you can build the documentation by first moving to this directory by opening a terminal and running
```bash
cd PATH_TO_REPO/docs/
```
Then, using `doxygen` you can build the documentation by running
```bash
doxygen Doxyfile
```

## Viewing Documentation
Once you run the above command, you will see a folder called `html` appear in your CWD. To view the documentation using a browser (shown below is an example using firefox), run 
```bash
firefox html/index.html
```
You can also view the documentation by going into the `html` folder and finding the `index.html` file and double-clicking on it.
