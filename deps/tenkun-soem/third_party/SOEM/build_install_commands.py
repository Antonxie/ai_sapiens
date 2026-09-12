#!/usr/bin/env python3

def build_directory(source_dir : str) -> str:
    # build directory
    return source_dir + "/build"

def build_commands(source_dir : str, build_dir : str, search_prefix : list, install_prefix : str, clean_first : bool) -> list:
    # build commands to run in build directory
    cmd = [
        "cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=\"" + install_prefix + "\" -DCMAKE_PREFIX_PATH=\"" + ";".join(search_prefix) + "\" \"" + source_dir + "\"",
        "cmake --build \"" + build_dir + "\"" + (" --clean-first" if clean_first else "")
    ]
    return cmd

def install_commands(source_dir : str, build_dir : str, search_prefix : str, install_prefix : str, clean_first : bool) -> list:
    # install commands to run in build directory
    cmd = [
        "cmake --build \"" + build_dir + "\" --target install"
    ]
    return cmd
