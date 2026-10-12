/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// kicadopenapi: undo marks at the undo limit, timed-out calls, argument validation

#include <boost/test/unit_test.hpp>

#include <kicadopenapi_args.h>
#include <kicadopenapi_call_ticket.h>
#include <kicadopenapi_undo.h>
#include <undo_redo_container.h>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>


namespace
{

/// The undo side of EDA_BASE_FRAME, with a real limit: pushing past it drops the oldest command
/// and counts it, as EDA_BASE_FRAME::PushCommandToUndoList() does
class FAKE_FRAME
{
public:
    explicit FAKE_FRAME( int aMax ) : m_max( aMax ) {}

    void Edit() { push( std::make_unique<PICKED_ITEMS_LIST>() ); }

    void Undo()
    {
        m_redo.push_back( std::move( m_undo.back() ) );
        m_undo.pop_back();
    }

    void Redo()
    {
        push( std::move( m_redo.back() ) );
        m_redo.pop_back();
    }

    void Clear()
    {
        m_dropped += m_undo.size();
        m_undo.clear();
        m_redo.clear();
    }

    int      GetUndoCommandCount() const { return (int) m_undo.size(); }
    int      GetMaxUndoItems() const { return m_max; }
    uint64_t GetUndoDropCount() const { return m_dropped; }

    uint64_t GetUndoCommandSerial( int aIndex ) const
    {
        return aIndex >= 0 && aIndex < (int) m_undo.size() ? m_undo[aIndex]->GetSerial() : 0;
    }

private:
    void push( std::unique_ptr<PICKED_ITEMS_LIST> aCommand )
    {
        m_undo.push_back( std::move( aCommand ) );

        while( (int) m_undo.size() > m_max )
        {
            m_undo.erase( m_undo.begin() );
            m_dropped++;
        }
    }

    int                                             m_max;
    uint64_t                                        m_dropped = 0;
    std::vector<std::unique_ptr<PICKED_ITEMS_LIST>> m_undo;
    std::vector<std::unique_ptr<PICKED_ITEMS_LIST>> m_redo;
};

} // namespace


BOOST_AUTO_TEST_SUITE( KicadOpenApi )


BOOST_AUTO_TEST_CASE( CommandSerialsAreUniqueAndCopiesAreNewCommands )
{
    PICKED_ITEMS_LIST a;
    PICKED_ITEMS_LIST b;
    PICKED_ITEMS_LIST c( a );

    BOOST_CHECK_NE( a.GetSerial(), 0u );
    BOOST_CHECK_NE( a.GetSerial(), b.GetSerial() );
    BOOST_CHECK_NE( a.GetSerial(), c.GetSerial() );

    const uint64_t bSerial = b.GetSerial();
    b = a;
    BOOST_CHECK_EQUAL( b.GetSerial(), bSerial );   // assignment keeps the command's own serial
}


BOOST_AUTO_TEST_CASE( MarkBelowTheLimitIsFound )
{
    FAKE_FRAME frame( 10 );
    frame.Edit();
    frame.Edit();

    const KOPENAPI_UNDO_MARK mark = KopenapiUndoMark( frame );
    frame.Edit();
    frame.Edit();

    std::string why;
    BOOST_CHECK( KopenapiUndoTarget( frame, mark, why ) == std::optional<int>( 2 ) );
}


// The bug: at the limit the depth stays the same while the commands under it change, so a mark by
// depth "succeeds" without undoing anything
BOOST_AUTO_TEST_CASE( MarkAtTheLimitFollowsItsCommand )
{
    FAKE_FRAME frame( 5 );

    for( int i = 0; i < 5; ++i )
        frame.Edit();

    const KOPENAPI_UNDO_MARK mark = KopenapiUndoMark( frame );   // depth 5, at the limit
    frame.Edit();
    frame.Edit();                                                // two oldest dropped, depth still 5

    BOOST_CHECK_EQUAL( frame.GetUndoCommandCount(), mark.depth );

    std::string              why;
    const std::optional<int> target = KopenapiUndoTarget( frame, mark, why );

    BOOST_REQUIRE( target );
    BOOST_CHECK_EQUAL( *target, 3 );   // the mark's command moved down by the two dropped
}


BOOST_AUTO_TEST_CASE( MarkDroppedAtTheLimitIsRefused )
{
    FAKE_FRAME frame( 3 );
    frame.Edit();

    const KOPENAPI_UNDO_MARK mark = KopenapiUndoMark( frame );

    for( int i = 0; i < 3; ++i )
        frame.Edit();   // the mark's command is pushed out

    std::string why;
    BOOST_CHECK( !KopenapiUndoTarget( frame, mark, why ) );
    BOOST_CHECK( why.find( "no longer in the undo stack" ) != std::string::npos );
}


BOOST_AUTO_TEST_CASE( MarkOnEmptyStack )
{
    FAKE_FRAME frame( 3 );
    const KOPENAPI_UNDO_MARK mark = KopenapiUndoMark( frame );

    frame.Edit();
    frame.Edit();

    std::string why;
    BOOST_CHECK( KopenapiUndoTarget( frame, mark, why ) == std::optional<int>( 0 ) );

    frame.Edit();
    frame.Edit();   // one made after the mark is dropped: back to empty is no longer the mark

    BOOST_CHECK( !KopenapiUndoTarget( frame, mark, why ) );
    BOOST_CHECK( why.find( "dropped" ) != std::string::npos );
}


BOOST_AUTO_TEST_CASE( MarkAfterClearIsRefused )
{
    FAKE_FRAME frame( 10 );
    frame.Edit();

    const KOPENAPI_UNDO_MARK mark = KopenapiUndoMark( frame );
    frame.Clear();
    frame.Edit();

    std::string why;
    BOOST_CHECK( !KopenapiUndoTarget( frame, mark, why ) );
}


BOOST_AUTO_TEST_CASE( MarkSurvivesUndoAndRedoButNotABranch )
{
    FAKE_FRAME frame( 10 );
    frame.Edit();
    frame.Edit();

    const KOPENAPI_UNDO_MARK mark = KopenapiUndoMark( frame );
    std::string              why;

    frame.Undo();
    frame.Redo();   // the same command is back on top
    BOOST_CHECK( KopenapiUndoTarget( frame, mark, why ) == std::optional<int>( 2 ) );

    frame.Undo();
    frame.Edit();   // a different command where the mark's was
    BOOST_CHECK( !KopenapiUndoTarget( frame, mark, why ) );
}


BOOST_AUTO_TEST_CASE( TicketExactlyOneSideWins )
{
    {
        KOPENAPI_CALL_TICKET t;
        BOOST_CHECK( t.Abandon() );
        BOOST_CHECK( !t.Start() );   // a withdrawn call never runs
        BOOST_CHECK_EQUAL( t.State(), KOPENAPI_CALL_TICKET::ABANDONED );
    }
    {
        KOPENAPI_CALL_TICKET t;
        BOOST_CHECK( t.Start() );
        BOOST_CHECK( !t.Abandon() );   // a started call cannot be withdrawn
        BOOST_CHECK_EQUAL( t.State(), KOPENAPI_CALL_TICKET::STARTED );
    }

    // racing: whichever wins, the other loses, every time
    for( int i = 0; i < 2000; ++i )
    {
        KOPENAPI_CALL_TICKET t;
        std::atomic<bool>    started{ false }, abandoned{ false };
        std::thread          a( [&] { started = t.Start(); } );
        std::thread          b( [&] { abandoned = t.Abandon(); } );
        a.join();
        b.join();
        BOOST_REQUIRE( started != abandoned );
    }
}


namespace
{

const nlohmann::json ROUTE_SCHEMA = R"json({
    "type":"object", "required":["from"],
    "properties":{
        "from":{"type":"string"},
        "layer":{"type":"string","default":"F.Cu"},
        "mode":{"type":"string","enum":["walkaround","shove"]},
        "width_mm":{"type":"number","minimum":0},
        "points":{"type":"array","items":{"type":"array","items":{"type":"number"},"minItems":2,"maxItems":2}},
        "dry_run":{"type":"boolean"}}})json"_json;

} // namespace


BOOST_AUTO_TEST_CASE( ArgsValid )
{
    BOOST_CHECK_EQUAL( KopenapiValidateArgs( "t_route", ROUTE_SCHEMA,
                                             { { "from", "U1.3" }, { "layer", "B.Cu" }, { "mode", "shove" },
                                               { "points", { { 1.0, 2.0 }, { 3, 4 } } } } ),
                       "" );
}


BOOST_AUTO_TEST_CASE( ArgsUnknownNameSuggestsNearest )
{
    const std::string bad = KopenapiValidateArgs( "t_route", ROUTE_SCHEMA, { { "from", "U1.3" }, { "layers", "B.Cu" } } );

    BOOST_CHECK( bad.find( "unknown argument 'layers'" ) != std::string::npos );
    BOOST_CHECK( bad.find( "did you mean 'layer'" ) != std::string::npos );

    const std::string far = KopenapiValidateArgs( "t_route", ROUTE_SCHEMA, { { "from", "U1.3" }, { "colour", 1 } } );
    BOOST_CHECK( far.find( "unknown argument 'colour'" ) != std::string::npos );
    BOOST_CHECK( far.find( "did you mean" ) == std::string::npos );
}


BOOST_AUTO_TEST_CASE( ArgsSchemaIsApplied )
{
    BOOST_CHECK( !KopenapiValidateArgs( "t_route", ROUTE_SCHEMA, { { "from", "U1.3" }, { "mode", "teleport" } } ).empty() );
    BOOST_CHECK( !KopenapiValidateArgs( "t_route", ROUTE_SCHEMA, { { "from", "U1.3" }, { "width_mm", -1 } } ).empty() );
    BOOST_CHECK( !KopenapiValidateArgs( "t_route", ROUTE_SCHEMA, { { "from", "U1.3" }, { "points", { { 1, "x" } } } } )
                          .empty() );
    BOOST_CHECK( !KopenapiValidateArgs( "t_route", ROUTE_SCHEMA, { { "layer", "F.Cu" } } ).empty() );   // required
}


BOOST_AUTO_TEST_CASE( ArgsIntegersAcceptIntegralNumbers )
{
    const nlohmann::json schema = R"json({"type":"object",
                                          "properties":{"steps":{"type":"integer","minimum":1}}})json"_json;

    BOOST_CHECK_EQUAL( KopenapiValidateArgs( "t_int", schema, { { "steps", 2 } } ), "" );
    BOOST_CHECK_EQUAL( KopenapiValidateArgs( "t_int", schema, { { "steps", 2.0 } } ), "" );
    BOOST_CHECK( !KopenapiValidateArgs( "t_int", schema, { { "steps", 2.5 } } ).empty() );
    BOOST_CHECK( !KopenapiValidateArgs( "t_int", schema, { { "steps", 0 } } ).empty() );
}


BOOST_AUTO_TEST_CASE( ArgsOpenSchemasStayOpen )
{
    const nlohmann::json open = R"json({"type":"object","additionalProperties":true,
                                        "properties":{"a":{"type":"string"}}})json"_json;
    const nlohmann::json bare = R"json({"type":"object"})json"_json;

    BOOST_CHECK_EQUAL( KopenapiValidateArgs( "t_open", open, { { "b", 1 } } ), "" );
    BOOST_CHECK_EQUAL( KopenapiValidateArgs( "t_bare", bare, { { "anything", 1 } } ), "" );
}


BOOST_AUTO_TEST_CASE( ArgsFormatsAreNotChecked )
{
    const nlohmann::json withFormat = R"json({"type":"object",
                                              "properties":{"path":{"type":"string","format":"uri"}}})json"_json;

    BOOST_CHECK_EQUAL( KopenapiValidateArgs( "t_format", withFormat, { { "path", "/not a uri" } } ), "" );
}


BOOST_AUTO_TEST_SUITE_END()
