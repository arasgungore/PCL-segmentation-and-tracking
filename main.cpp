#include "lib.h"


int main() {


    auto file_pairs = find_files_by_extensions(basePath, ".jpg", ".pgm");
    std::vector<std::string> rgb_filenames = file_pairs.first;
    std::vector<std::string> depth_filenames = file_pairs.second;
    int num_frames = 20; //In this configuration, to show object tracking, it uses the first and the twentieth frames.

    process_frames(basePath, rgb_filenames, depth_filenames, num_frames);

    return 0;
}

