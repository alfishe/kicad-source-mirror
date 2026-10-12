/// @file kicadopenapi_undo.h
/// @brief Marks in a GUI editor's undo stack that stay exact when the stack is at its limit
/// (KOPENAPI_UNDO_MARK), shared by the schematic and board kifaces.
#ifndef KICADOPENAPI_UNDO_H
#define KICADOPENAPI_UNDO_H

#include <kicadopenapi_registry.h>

#include <optional>
#include <string>


// Templates over the frame (EDA_BASE_FRAME: GetUndoCommandCount, GetUndoCommandSerial,
// GetUndoDropCount, GetMaxUndoItems) so the logic is tested without a window.

/// @brief The point aFrame's undo stack is at now
template <typename FRAME>
KOPENAPI_UNDO_MARK KopenapiUndoMark( const FRAME& aFrame )
{
    KOPENAPI_UNDO_MARK mark;
    mark.depth = aFrame.GetUndoCommandCount();
    mark.top = mark.depth > 0 ? aFrame.GetUndoCommandSerial( mark.depth - 1 ) : 0;
    mark.dropped = aFrame.GetUndoDropCount();
    return mark;
}


/// @brief How deep aFrame's undo stack must be to be at aMark again, or nullopt and aWhy when it
/// cannot get there: the command on top at the mark is no longer in the stack, or (for a mark on
/// an empty stack) commands made since have been dropped.  Never a guess.
template <typename FRAME>
std::optional<int> KopenapiUndoTarget( const FRAME& aFrame, const KOPENAPI_UNDO_MARK& aMark, std::string& aWhy )
{
    const int count = aFrame.GetUndoCommandCount();

    if( aMark.top == 0 )
    {
        if( aFrame.GetUndoDropCount() != aMark.dropped )
        {
            aWhy = "the undo stack has dropped " + std::to_string( aFrame.GetUndoDropCount() - aMark.dropped )
                   + " command(s) made since the mark (undo limit " + std::to_string( aFrame.GetMaxUndoItems() )
                   + ", or the stack was cleared)";
            return std::nullopt;
        }

        return 0;
    }

    for( int i = count - 1; i >= 0; --i )
    {
        if( aFrame.GetUndoCommandSerial( i ) == aMark.top )
            return i + 1;
    }

    aWhy = "the undo command at the mark is no longer in the undo stack: dropped at the undo limit ("
           + std::to_string( aFrame.GetMaxUndoItems() ) + "), cleared, or replaced after an undo";
    return std::nullopt;
}

#endif
