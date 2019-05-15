#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "stb_image_write.h"

#include <assert.h>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "pathfinding.h"

//////////////////////////////////////////////////////////
// simple example using an image file as graph topology //
/////////////////////////////////////////////////////////

// macro for suppresing compiler warnings for unused parameters

// function for extracting pixel value
unsigned char getPixel(std::vector<unsigned char> image, int x, int y, int width = 256) {
    assert(x >= 0);
    assert(y >= 0);
    assert(x < width);

    return image[y * width + x];
}

// function computing manhattan distance
int l1Dist(int x1, int y1, int x2, int y2) {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

float smoothstep(float x) {
    assert(x >= 0);
    assert(x <= 1);

    float x_sq = x*x;

    return 3 * x_sq - 2 * (x_sq * x);
}

struct Pixel {
    int x, y;

    Pixel(int xCoord = 0, int yCoord = 0)
        : x(xCoord)
        , y(yCoord)
    { }

    Pixel(const Pixel& p)
        : x(p.x)
        , y(p.y)
    { }

    Pixel& operator=(const Pixel& other) {
        x = other.x;
        y = other.y;

        return *this;
    }

    bool operator==(const Pixel& other) const {
        return (x == other.x) && (y == other.y);
    }

    bool operator<(const Pixel& other) const {
        return (y != other.y) ? (y < other.y) : (x < other.x);
    }
};

int main() {

    // create an input image where intensities smoothly increase towards the center
    int height = 256;
    int width = 256;

    std::vector<unsigned char> image;
    image.resize(width * height);

    assert(width == height);

    int center = width / 2;
    int radiusX = width / 2;
    int radiusY = static_cast<int>(static_cast<float>(width) / 2.5f);
    float radiusX_rcp = 1.f / static_cast<float>(radiusX);
    float radiusY_rcp = 1.f / static_cast<float>(radiusY);
    for (int j = 0; j < height; ++j) {
        float yNorm = static_cast<float>(j - center) * radiusY_rcp;
        float yNorm_sq = yNorm * yNorm;
        for (int i = 0; i < width; ++i) {
            float xNorm = static_cast<float>(i - center) * radiusX_rcp;
            float xNorm_sq = xNorm * xNorm;

            // compute (normalized) squared distance, outside of circle clamp to [0, 1]
            float normSqDist = std::min(std::max(xNorm_sq + yNorm_sq, 0.f), 1.f);
            // invert (i.e., 1-x) and use as input to smoothstep to get image intensity
            float invNormSqDist = 1.f - (normSqDist);

            float smoothedDist = smoothstep(invNormSqDist);
            float intensity = 255.f * smoothedDist;
            // cast and write to pixel
            image[j * width + i] = static_cast<unsigned int>(intensity);
        }
    }

    // we look for a path from the middle top to the middle bottom
    Pixel start(128, 0);
    Pixel end(128, 255);

    // heuristic is L1 Norm (Manhattan metric)
    PathFinding<Pixel>::HeuristicFunction hFunc = [](Pixel a, Pixel b) { return static_cast<float>(l1Dist(a.x, a.y, b.x, b.y)); };
    // cost from one pixel to its neighbor is the squared difference of intensities (but 1 at least)
    PathFinding<Pixel>::EdgeCostFunction costFunc = [&](Pixel a, Pixel b) {
        float v1 = static_cast<float>(getPixel(image, a.x, a.y));
        float v2 = static_cast<float>(getPixel(image, b.x, b.y));
        float diff = v2 - v1;
        return 1.f + diff * diff;
    };
    // neighbors are the pixels given by 4-neighborhood
    PathFinding<Pixel>::GetOutgoingNeighborsFunction neighborFunc = [&](Pixel p) {
        std::vector<Pixel> result;
        if (p.x > 0)
           result.push_back(Pixel(p.x - 1, p.y));
        if (p.x < width-1)
           result.push_back(Pixel(p.x + 1, p.y));
        if (p.y > 0)
           result.push_back(Pixel(p.x, p.y - 1));
        if (p.y < width-1)
           result.push_back(Pixel(p.x, p.y + 1));

        return result;
    };

    // compute the shortest path
    PathFinding<Pixel> pathFinder(hFunc, costFunc, neighborFunc);
    std::vector<Pixel> path = pathFinder.getShortestPath(start, end);

    // create a result image
    std::vector<unsigned char> result;
    result.resize(width * height * 3);

    // copy the original image as RGB components
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            unsigned char val = image[j * width + i];
            for (int c = 0; c < 3; ++c) {
                result[j * width * 3 + i * 3 + c] = val;
            }
        }
    }

    // paint shortest path as red pixels into result image with yellow start and blue destination
    // start
    result[start.y * width * 3 + start.x * 3 + 0] = 255;
    result[start.y * width * 3 + start.x * 3 + 1] = 255;
    result[start.y * width * 3 + start.x * 3 + 2] = 0;

    // path
    for (auto& p : path) {
        result[p.y * width * 3 + p.x * 3 + 0] = 255;
        result[p.y * width * 3 + p.x * 3 + 1] = 0;
        result[p.y * width * 3 + p.x * 3 + 2] = 0;
    }

    // destination
    result[end.y * width * 3 + end.x * 3 + 0] = 0;
    result[end.y * width * 3 + end.x * 3 + 1] = 0;
    result[end.y * width * 3 + end.x * 3 + 2] = 255;

    // export result image
    stbi_write_png("result_path.png", width, height, 3, (void*) &result[0], 0);

    return 0;
}
