#!/bin/bash

Color_off='\033[0m'       # Text Reset
Black='\033[0;30m'        # Black
Red='\033[0;31m'          # Red
Green='\033[0;32m'        # Green
Yellow='\033[0;33m'       # Yellow
Blue='\033[0;34m'         # Blue
Purple='\033[0;35m'       # Purple
Cyan='\033[0;36m'         # Cyan
White='\033[0;37m'        # White

Bold_Color_off='\033[1m'  # Text Reset
Bold_Black='\033[1;30m'   # Bold Black
Bold_Red='\033[1;31m'     # Bold Red
Bold_Green='\033[1;32m'   # Bold Green
Bold_Yellow='\033[1;33m'  # Bold Yellow
Bold_Blue='\033[1;34m'    # Bold Blue
Bold_Purple='\033[1;35m'  # Bold Purple
Bold_Cyan='\033[1;36m'    # Bold Cyan0
Bold_White='\033[1;37m'   # Bold White

#=============================================================================
# echo with color function
#=============================================================================
echo_with_color () {
  printf '%b\n' "$1$2$Color_off" >&2
}

#=============================================================================
# logo function
#=============================================================================
logo () {
    echo_with_color ${Bold_White} "_________    ___.                          ________              ________                 "
    echo_with_color ${Bold_White} "\____    /____\_ |_________  ____           \______ \   _______  _\_____  \ ______  ______"
    echo_with_color ${Bold_White} "  /     // __ \| __ \_  __ \/  _ \   ______  |    |  \_/ __ \  \/ //   |   \\____ \/  ___/"
    echo_with_color ${Bold_Blue} " /     /\  ___/| \_\ \  | \(  <_> ) /_____/  |    \`   \  ___/\   //    |    \  |_> >___ \ "
    echo_with_color ${Bold_Blue} "/_______ \___  >___  /__|   \____/          /_______  /\___  >\_/ \_______  /   __/___   >"
    echo_with_color ${Bold_White} "        \/   \/    \/                               \/     \/             \/|__|       \/ "
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "--------------------------------------------------------Bad Ass DevOps for Space Robots----"
    echo_with_color ${Bold_Blue} ""
}


#=============================================================================
# home function
#=============================================================================
home () {
    clear
    logo
    cd /MDP
    }

#=============================================================================
# tools function
#=============================================================================
tools () {
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_White} "Toolchain included in this distribution:"
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Ansible : ansible --help"
    echo_with_color ${Color_off} "Ansible is an open-source software provisioning, configuration management, and  "
    echo_with_color ${Color_off} "application-deployment tool enabling infrastructure as code. It runs on many Unix-like"
    echo_with_color ${Color_off} "systems, and can configure both Unix-like systems as well as Microsoft Windows."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Azure CLI : az --help"
    echo_with_color ${Color_off} "The Azure command-line interface (Azure CLI) is a set of commands used to create and manage"
    echo_with_color ${Color_off} "Azure resources. The Azure CLI is available across Azure services and is designed to get"
    echo_with_color ${Color_off} "you working quickly with Azure, with an emphasis on automation."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Bazel : bazel --help"
    echo_with_color ${Color_off} "Bazel is a free software tool for the automation of building and testing of software."
    echo_with_color ${Color_off} "The company Google uses the build tool Blaze internally and released an open-sourced port"
    echo_with_color ${Color_off} "of the Blaze tool as Bazel, named as an anagram of Blaze."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "clang-format : clang-format --help"
    echo_with_color ${Color_off} "Clang-Format is a widely-used C++ code formatter. As it provides an option to define code"
    echo_with_color ${Color_off} "style options in YAML-formatted files — named .clang-format or _clang-format — these files"
    echo_with_color ${Color_off} "often become a part of your project where you keep all code style rules."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "clang-tidy : clang-tidy --help"
    echo_with_color ${Color_off} "clang-tidy is a clang-based C++ “linter” tool. Its purpose is to provide an extensible"
    echo_with_color ${Color_off} "framework for diagnosing and fixing typical programming errors, like style violations,"
    echo_with_color ${Color_off} "interface misuse, or bugs that can be deduced via static analysis. clang-tidy is"
    echo_with_color ${Color_off} "modular and provides a convenient interface for writing new checks."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Cppcheck : cppcheck --help"
    echo_with_color ${Color_off} "Cppcheck is a static analysis tool for C/C++ code. It provides unique code analysis to"
    echo_with_color ${Color_off} "detect bugs and focuses on detecting undefined behaviour and dangerous coding constructs."
    echo_with_color ${Color_off} "The goal is to have very few false positives. Cppcheck is designed to be able to analyze"
    echo_with_color ${Color_off} "your C/C++ code even if it has non-standard syntax (common in embedded projects)."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Cpplint : cpplint --help"
    echo_with_color ${Color_off} "cpplint or cpplint.py is an open source lint-like tool developed by Google, designed to"
    echo_with_color ${Color_off} "ensure that C++ code conforms to Google's coding style guides. Therefore cpplint implements"
    echo_with_color ${Color_off} "what Google considers best practices in C++ coding. The script cpplint.py reads source code"
    echo_with_color ${Color_off} "files and flags deviations from the style guide. It also identifies syntax errors."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Docker : docker --help"
    echo_with_color ${Color_off} "Docker is a set of platform as a service products that use OS-level virtualization to"
    echo_with_color ${Color_off} "deliver software in packages called containers."
    echo_with_color ${Bold_Green} "  Quickstart Commands:"
    echo_with_color ${Bold_Green} "    * sudo service docker start"
    echo_with_color ${Bold_Green} "    * docker run hello-world"
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Git : git --help"
    echo_with_color ${Color_off} "Git is software for tracking changes in any set of files, usually used for coordinating"
    echo_with_color ${Color_off} "work among programmers collaboratively developing source code during software development."
    echo_with_color ${Color_off} "Its goals include speed, data integrity, and support for distributed, non-linear workflows."
    echo_with_color ${Bold_Green} "  Quickstart Commands:"
    echo_with_color ${Bold_Green} "    * git clone git@gitlab.tudelft.nl:lunar-zebro/strelka-monorepo.git ~/strelka-monorepo"
    echo_with_color ${Bold_Green} "    * cd ~/strelka-monorepo"
    echo_with_color ${Bold_Green} "    * git branch -v -a"
    echo_with_color ${Bold_Green} "    * git checkout name-of-branch-goes-here"
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "GLab : glab --help"
    echo_with_color ${Color_off} "GLab is an open source GitLab CLI tool bringing GitLab to your terminal next to where you"
    echo_with_color ${Color_off} "are already working with git and your code without switching between windows and browser "
    echo_with_color ${Color_off} "tabs. Work with issues, merge requests, watch running pipelines directly from your CLI."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Helm Charts : helm --help"
    echo_with_color ${Color_off} "Helm helps you manage Kubernetes applications — Helm Charts help you define, install, and"
    echo_with_color ${Color_off} "upgrade even the most complex Kubernetes application."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "htop : htop --help"
    echo_with_color ${Color_off} "htop is an interactive system-monitor process-viewer and process-manager."
    echo_with_color ${Color_off} "It is designed as an alternative to the Unix program top."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "include-what-you-use : iwyu --help"
    echo_with_color ${Color_off} "A tool for use with clang to analyze #includes in C and C++ source files."
    echo_with_color ${Color_off} "Include what you use means this: for every symbol (type, function variable, or macro)"
    echo_with_color ${Color_off} "that you use in foo.cc, either foo.cc or foo. h should #include a .h file that exports"
    echo_with_color ${Color_off} "the declaration of that symbol."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "k9s : k9s --help"
    echo_with_color ${Color_off} "K9s is a terminal based UI to interact with your Kubernetes clusters. The aim of this"
    echo_with_color ${Color_off} "project is to make it easier to navigate, observe and manage your deployed applications"
    echo_with_color ${Color_off} "in the wild."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "Kubernetes Command Line Tool : kubectl --help"
    echo_with_color ${Color_off} "The Kubernetes command-line tool, kubectl, allows you to run commands against Kubernetes"
    echo_with_color ${Color_off} "clusters. You can use kubectl to deploy applications, inspect and manage cluster resources,"
    echo_with_color ${Color_off} "and view logs."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "LunarVim : lvim --help"
    echo_with_color ${Color_off} "An IDE layer for Neovim with sane defaults. Completely free and community driven."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "pre-commit : pre-commit --help"
    echo_with_color ${Color_off} "A framework for managing and maintaining multi-language pre-commit hooks."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "pstree : pstree --help"
    echo_with_color ${Color_off} "pstree is a Linux command that shows the running processes as a tree. It is used as a more"
    echo_with_color ${Color_off} "visual alternative to the ps command. The root of the tree is either init or the process"
    echo_with_color ${Color_off} "with the given pid."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "tree : tree --help"
    echo_with_color ${Color_off} "tree is a cross-platform command-line program used to recursively list or display the "
    echo_with_color ${Color_off} "content of a directory in a tree-like format. It outputs the directory paths and files in"
    echo_with_color ${Color_off} "each sub-directory and a summary of a total number of sub-directories and files."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "uncrustify : uncrustify --help"
    echo_with_color ${Color_off} "Banish crusty code with the Uncrustify C/C++/C#/D/Java/Pawn source code beautifier."
    echo_with_color ${Color_off} "It indents, adds newlines, aligns, etc, and is highly configurable."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    echo_with_color ${Bold_Blue} "lunar-zebro-cli : lz --help"
    echo_with_color ${Color_off} "A command line utility to interact with a lunar-zebro prototype robot."
    echo_with_color ${Bold_Blue} "-------------------------------------------------------------------------------------------"
    }
