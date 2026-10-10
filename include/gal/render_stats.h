// This program source code file is part of KiCad, a free EDA CAD application.
//
// Copyright The KiCad Developers, see AUTHORS.txt for contributors.
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

/// @file render_stats.h
/// @brief Per-frame render statistics of GAL canvases for benchmarks.  Collected only while a
/// collector is active (RENDER_STATS::SetActive); otherwise every hook is a single pointer test.

#pragma once

#include <gal/gal.h>

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace KIGFX
{

/// @brief One frame drawn by EDA_DRAW_PANEL_GAL::DoRePaint.  Times are CPU wall time in ms; the GPU
/// works asynchronously, so its cost shows up in later stages (usually swap) unless the collector
/// synchronises after each stage (RENDER_STATS::gpuSync).
struct RENDER_FRAME_STATS
{
    double total = 0;       ///< DoRePaint from start to end
    double update = 0;      ///< VIEW::UpdateItems: changed items re-tessellated into the cache, R-tree
    double rtree = 0;       ///< part of update: bulk R-tree rebuild (many items changed)
    double redraw = 0;      ///< VIEW::Redraw: layer walk, cached groups queued, non-cached items tessellated
    double begin = 0;       ///< drawing context: context made current, GAL BeginDrawing
    double end = 0;         ///< drawing context released: GAL EndDrawing as a whole
    double noncached = 0;   ///< EndDrawing: non-cached vertices drawn from client memory
    double cached = 0;      ///< EndDrawing: index list of the cached groups built and drawn
    double overlay = 0;     ///< EndDrawing: overlay target drawn
    double composite = 0;   ///< EndDrawing: buffers composited, presented, cursor
    double swap = 0;        ///< SwapBuffers (waits for vsync / the GPU)

    uint64_t layers = 0;             ///< view layers walked
    uint64_t itemsCached = 0;        ///< item-layer pairs drawn from the cache
    uint64_t itemsImmediate = 0;     ///< item-layer pairs tessellated for this frame only
    uint64_t itemsRecached = 0;      ///< item-layer pairs tessellated into the cache
    uint64_t itemsUpdated = 0;       ///< items with a pending update processed by UpdateItems
    uint64_t cachedIndices = 0;      ///< vertex indices drawn from the cache
    uint64_t noncachedVertices = 0;  ///< vertices drawn from the non-cached target
    uint64_t overlayVertices = 0;    ///< vertices drawn from the overlay target
    uint64_t drawCalls = 0;          ///< glDrawArrays / glDrawElements calls
    uint64_t cacheVertices = 0;      ///< capacity of the cache (VBO) in vertices after the frame
};


/// @brief Time and items of one view layer, summed over the frames of a collection
struct RENDER_LAYER_STATS
{
    double   ms = 0;
    uint64_t items = 0;
    uint64_t frames = 0;
};


/// @brief Collector of a benchmark run.  Main thread only.
class GAL_API RENDER_STATS
{
public:
    /// @return the active collector, nullptr when nothing collects
    static RENDER_STATS* Active() { return s_active; }

    /// @brief Make aStats the collector (nullptr stops collecting)
    static void SetActive( RENDER_STATS* aStats ) { s_active = aStats; }

    /// @brief Ends the current frame: appends it to frames and starts an empty one
    void EndFrame()
    {
        frames.push_back( frame );
        frame = RENDER_FRAME_STATS();
    }

    /// @brief Drops all collected frames and layer sums
    void Clear()
    {
        frames.clear();
        layers.clear();
        frame = RENDER_FRAME_STATS();
    }

    /// @return milliseconds since aStart
    static double Since( std::chrono::steady_clock::time_point aStart )
    {
        return std::chrono::duration<double, std::milli>( std::chrono::steady_clock::now() - aStart )
                .count();
    }

    RENDER_FRAME_STATS              frame;      ///< the frame being drawn
    std::vector<RENDER_FRAME_STATS> frames;     ///< finished frames
    std::map<int, RENDER_LAYER_STATS> layers;   ///< per view layer id

    bool        gpuSync = false;   ///< glFinish after each EndDrawing stage (GPU time per stage)
    std::string glRenderer;        ///< GL_RENDERER of the canvas, filled on the first frame
    std::string glVersion;         ///< GL_VERSION of the canvas, filled on the first frame

private:
    static RENDER_STATS* s_active;
};

} // namespace KIGFX
