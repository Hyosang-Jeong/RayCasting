#include"Material.h"

#define STB_IMAGE_IMPLEMENTATION
#define STBI_FAILURE_USERMSG
#include "stb_image.h"
#include <fstream>
Texture::Texture(const std::string& bpath) : id(0)
{
    // Replace backslashes with forward slashes -- Good for Linux, and maybe Windows?
    std::string path = bpath;
    std::string bs = "\\";
    std::string fs = "/";
    while (path.find(bs) != std::string::npos) {
        path.replace(path.find(bs), 1, fs);
    }

    // Does the file exist?
    std::ifstream find_it(path.c_str());
    if (find_it.fail()) {
        std::cerr << "Texture file not found: " << path << std::endl;
        exit(-1);
    }
    else {
        // Read image, and check for success
        stbi_set_flip_vertically_on_load(true);
        image = stbi_load(path.c_str(), &width, &height, &depth, 3);
        printf("%d %d %d %s\n", depth, width, height, path.c_str());
        if (!image) {
            printf("\nRead error on file %s:\n  %s\n\n", path.c_str(), stbi_failure_reason());
            exit(-1);
        }
    }
}