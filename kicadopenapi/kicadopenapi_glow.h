/// @file kicadopenapi_glow.h
/// @brief kicadopenapi GUI glow manager: what an API edit changed - drawn or only a property - glows for
/// a moment, so a person watching sees where the agent worked.
///
/// One manager per process (kicommon), shared by every editor: one collection of glowing items
/// with the time each glow started, in that order, and one timer that redraws the fading halos
/// and dims entries in the same order once their time is up.  An item glowing again gets a newer
/// entry; only its newest entry dims it.  Each editor kiface registers a target that finds its
/// items by uuid (again on every tick, so an undo or a closed editor is harmless), brightens
/// them and draws their halo (kicadopenapi_glow_view.h).  Headless processes have no editor
/// frames: targets report not alive and nothing is collected.  Main thread only.
#ifndef KICADOPENAPI_GLOW_H
#define KICADOPENAPI_GLOW_H

#include <kicommon.h>
#include <kiid.h>

#include <utility>
#include <vector>


/// @brief One editor's side of the glow
class KICOMMON_API KOPENAPI_GLOW_TARGET
{
public:
    virtual ~KOPENAPI_GLOW_TARGET() = default;

    /// @brief The editor is open (GUI); false headless or closed
    virtual bool Alive() = 0;

    /// @brief Brighten / restore an item; false when it no longer exists
    virtual bool Brighten( const KIID& aItem, bool aOn ) = 0;

    /// @brief Redraw the halos: each item with its strength 0..1; empty clears the overlay
    virtual void Halo( const std::vector<std::pair<KIID, double>>& aItems ) = 0;

    /// @brief The first item starts / the last one ends glowing in this editor
    virtual void Begin() {}
    virtual void End() {}
};


class KICOMMON_API KOPENAPI_GLOW_MANAGER
{
public:
    static constexpr int GLOW_MS = 2000;

    static KOPENAPI_GLOW_MANAGER& Get();

    /// @brief Items an edit changed in the editor behind aTarget.  aStaggerMs > 0: they light up one
    /// after another, that far apart, in the given order (and dim in that order), so a batch
    /// reads as a sequence.
    void Add( KOPENAPI_GLOW_TARGET* aTarget, const std::vector<KIID>& aItems, int aStaggerMs = 0 );

    /// @brief Items glowing now (all editors), for tests
    size_t Count() const;

private:
    KOPENAPI_GLOW_MANAGER();
    class IMPL;
    IMPL* m_impl;
};

#endif
