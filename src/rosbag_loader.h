#pragma once
#include "onlinefusionviewer.hpp"

void loadBag(   const std::string &filename,
                float minDepth,
                FusionParameter par,
                volatile bool *newMesh,
                volatile bool *fusionActive,
                volatile bool *fusionAlive);
