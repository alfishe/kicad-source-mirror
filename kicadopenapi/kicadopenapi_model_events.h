/// @file kicadopenapi_model_events.h
/// @brief One hub for "the model changed" notifications to whoever registers interest (the
/// annotation layer today), guarded so a faulty subscriber cannot break or stall the editor:
/// - delivered on the main thread, after the change (the model's listeners call Post() from
///   their commit notifications), never re-entrantly: a notification raised while one is being
///   delivered is dropped and journaled;
/// - an exception from a callback is caught and journaled; three failures disable the subscriber;
/// - a callback slower than 20 ms is journaled; three slower than 200 ms disable the subscriber;
/// - a subscription lives as long as its token: destroying the token unsubscribes, so the hub
///   never calls a dead object.
#ifndef KICADOPENAPI_MODEL_EVENTS_H
#define KICADOPENAPI_MODEL_EVENTS_H

#include <json_common.h>
#include <kicommon.h>
#include <kiid.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>


class KICOMMON_API KOPENAPI_MODEL_EVENTS
{
public:
    struct EVENT
    {
        enum class KIND
        {
            ADDED,
            REMOVED,
            CHANGED,
            RESET     ///< the whole document was replaced or reloaded
        };

        std::string       domain;   ///< "pcb", "sch"
        KIND              kind = KIND::CHANGED;
        std::vector<KIID> items;
    };

    using CALLBACK = std::function<void( const EVENT& )>;

    /// @brief Keeps a subscription alive; destroying it unsubscribes
    class KICOMMON_API TOKEN
    {
    public:
        explicit TOKEN( int aId ) : m_id( aId ) {}
        ~TOKEN();

        TOKEN( const TOKEN& ) = delete;
        TOKEN& operator=( const TOKEN& ) = delete;

    private:
        int m_id;
    };

    /**
     * @param aName names the subscriber in the journal and Status().
     * @param aDomain receives only this domain's events ("" = all).
     */
    static std::unique_ptr<TOKEN> Subscribe( const std::string& aName, const std::string& aDomain,
                                             CALLBACK aCallback );

    /// @brief Delivers an event to the subscribers (main thread)
    static void Post( const EVENT& aEvent );

    /// @brief Subscribers with their counts (delivered, failures, slow) and whether disabled
    static nlohmann::json Status();
};

#endif
