#include "kicadopenapi_registry.h"


KOPENAPI_REGISTRY& KOPENAPI_REGISTRY::Get()
{
    static KOPENAPI_REGISTRY instance;
    return instance;
}


bool KOPENAPI_REGISTRY::Add( KOPENAPI_METHOD aMethod )
{
    KOPENAPI_REGISTRY& reg = Get();
    std::lock_guard<std::mutex> lock( reg.m_mutex );

    std::string name = aMethod.name;
    reg.m_methods[name] = std::move( aMethod );
    return true;
}


std::vector<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Snapshot() const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    std::vector<KOPENAPI_METHOD> out;

    for( const auto& [name, method] : m_methods )
        out.push_back( method );

    return out;
}


std::optional<KOPENAPI_METHOD> KOPENAPI_REGISTRY::Find( const std::string& aName ) const
{
    std::lock_guard<std::mutex> lock( m_mutex );
    auto it = m_methods.find( aName );

    if( it == m_methods.end() )
        return std::nullopt;

    return it->second;
}
