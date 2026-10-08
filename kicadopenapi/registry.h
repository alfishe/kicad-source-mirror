/*
 * kicadopenapi method registry: auto-indexing of handlers declared via macros.
 *
 * A handler declares itself once:
 *
 *   static std::string h_open_project( KOPENAPI_HOST& aHost, const std::string& aArgs );
 *   KOPENAPI_REGISTER( "open_project", "Open a .kicad_pro in the GUI", h_open_project );
 *
 * The registry is the MCP surface: /mcp/search scans names+summaries,
 * /mcp/invoke validates the name and dispatches the JSON body.
 */
#ifndef KICADOPENAPI_REGISTRY_H
#define KICADOPENAPI_REGISTRY_H

#include <functional>
#include <map>
#include <string>
#include <vector>

class KIWAY;
class wxWindow;

/**
 * Minimal host interface handed to every handler. Implemented by the owning
 * frame (or PGM window) so handlers can reach the Kiway and their window.
 */
class KOPENAPI_HOST
{
public:
    virtual ~KOPENAPI_HOST() = default;

    virtual KIWAY* Ki() const = 0;
    virtual wxWindow* Window() const = 0;
};

using KOPENAPI_HANDLER = std::function<std::string( KOPENAPI_HOST&, const std::string& aArgs )>;

struct KOPENAPI_METHOD
{
    std::string name;
    std::string summary;
    KOPENAPI_HANDLER handler;
};

class KOPENAPI_REGISTRY
{
public:
    static KOPENAPI_REGISTRY& Get()
    {
        static KOPENAPI_REGISTRY inst;
        return inst;
    }

    static bool Add( const std::string& aName, const std::string& aSummary,
                     KOPENAPI_HANDLER aHandler )
    {
        Get().m_methods[aName] = { aName, aSummary, std::move( aHandler ) };
        return true;
    }

    const std::map<std::string, KOPENAPI_METHOD>& Methods() const { return m_methods; }

    const KOPENAPI_METHOD* Find( const std::string& aName ) const
    {
        auto it = m_methods.find( aName );
        return it == m_methods.end() ? nullptr : &it->second;
    }

    std::vector<KOPENAPI_METHOD> Search( const std::string& aQuery, size_t aLimit = 10 ) const
    {
        std::vector<KOPENAPI_METHOD> hits;
        const std::string q = [&]
        {
            std::string lower = aQuery;
            for( char& ch : lower )
                ch = (char) tolower( (unsigned char) ch );
            return lower;
        }();

        if( !q.empty() )
        {
            for( const auto& [name, method] : m_methods )
            {
                std::string hay = name + " " + method.summary;
                for( char& ch : hay )
                    ch = (char) tolower( (unsigned char) ch );
                if( hay.find( q ) != std::string::npos )
                    hits.push_back( method );
            }
        }

        if( hits.size() > aLimit )
            hits.resize( aLimit );
        return hits;
    }

private:
    KOPENAPI_REGISTRY() = default;
    std::map<std::string, KOPENAPI_METHOD> m_methods;
};

/**
 * Auto-registers a handler at static initialization:
 *   static std::string my_handler( KOPENAPI_HOST&, const std::string& );
 *   KOPENAPI_REGISTER( "my_method", "does something", my_handler );
 */
#define KOPENAPI_REGISTER( aName, aSummary, aFn )                                        \
    static bool kopenapi_reg_##aFn = KOPENAPI_REGISTRY::Add( aName, aSummary, aFn )

#endif
