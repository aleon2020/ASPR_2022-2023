# Arquitectura Software para Robots 2022-2023

Welcome! This repository contains all the materials for the subject **Arquitectura Software para Robots**.

Below is a brief description of all the content found in this repository, intended to facilitate preparation for the final exam for the subject (many of which are omitted due to the obviousness of their name).

**IMPORTANT**: IF YOU NOTICE ANY MISTAKES OR MISSING IN ANY FILE UPLOADED TO THIS REPOSITORY (OR IF THERE IS ANY DOUBT REGARDING UNDERSTANDING), LEAVE ME AN ISSUE AND I WILL TRY TO RESOLVE THE PROBLEM AS SOON AS POSSIBLE. DON'T FORGET TO LEAVE ME A STAR AND I HOPE THAT ALL THIS MATERIAL IS OF GREAT HELP TO YOU.

Clone this repository by running the following command:

```sh
git clone https://<token>@github.com/aleon2020/ASPR_2022-2023.git
```

**IMPORTANT**: Add your token exactly as shown. This is so you don't have to enter the token in the terminal every time you want to update the repository using the 'git pull' command.

If you've already cloned this repository, run the following command before starting to work with it, as new changes or modifications may have been added. This is to ensure you have cloned the most recent version of the repository:

```sh
git pull
```

## 1. Summary of the theory contents

File ['Resumen Teoría ASPR.pdf'](https://github.com/aleon2020/ASR_2022-2023/blob/main/Resumen%20Teor%C3%ADa%20ASPR.pdf): Theory summary in PDF format.

**IMPORTANT**: To use the interactive index included in the summary, you must download the document in PDF format.

## 2. Packages and practices

Directory ['Paquetes'](https://github.com/aleon2020/ASR_2022-2023/tree/main/Paquetes): Contains all the example packages seen in class throughout the course.

Directory ['Prácticas'](https://github.com/aleon2020/ASR_2022-2023/tree/main/Pr%C3%A1cticas): Contains all the example packages seen in class throughout the course.

## 3. ROS2 activation in university laboratories

Open a terminal IN YOUR HOME DIRECTORY and run the following commands:

```sh
nano ./bashrc
```

Inside this file, write the following line:

```sh
source /opt/ros/jazzy/setup.bash
```

Save the changes, close the file, and close the terminal. This will save all the changes you made to the .bashrc file.

Once this is done, open a new terminal and check that ROS2 is working correctly by running the following command:

```sh
ros2
```

## 4. Creating a workspace, using and running a package

### 4.1 Creating and activating a workspace

It is recommended to open the terminal from the HOME (personal folder).

```sh
mkdir -p <my_workspace>/src
```

```sh
cd <my_workspace>/src/
```

```sh
git clone https://github.com/fmrico/book_ros2.git
```

```sh
cd ..
```

```sh
colcon build --symlink-install
```

Once you have finished compiling the workspace with colcon, add the following line to your .bashrc file.

```sh
source ~/<my_workspace>/install/setup.bash
```

Below is an example of program execution once all the previous steps have been completed:

```sh
ros2 run br2_basics logger
```

### 4.2 Creating and Running a Package

```sh
cd ~/<my_workspace>/src
```

```sh
ros2 pkg create <my_package> --dependencies <dependencies>
```

Once we have developed our package (programs, CMake, etc.), we do the following:

```sh
cd ~/<my_workspace>
```

```sh
colcon build --packages-select <my_package>
```

```sh
source ~/install/setup.bash
```

```sh
ros2 run <my_package> <executable>
```
