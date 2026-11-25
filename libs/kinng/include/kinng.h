/*
 * This program source code file is part of KiCad, a free EDA CAD application.
 *
 * Copyright (C) 2023 Jon Evans <jon@craftyjon.com>
 * Copyright The KiCad Developers, see AUTHORS.txt for contributors.
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
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef KICAD_KINNG_H
#define KICAD_KINNG_H

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>


/**
 * The URL to hand nng for an API socket URL. KiCad names its sockets ipc://<path> on every platform, as
 * nng 1.x did, where ipc:// on Windows was a named pipe (\\.\pipe\<path>). nng 2 names that pipe
 * winpipe://<path> and means AF_UNIX by ipc:// there, so on Windows ipc:// becomes winpipe:// and clients
 * built with nng 1.x keep reaching the same pipe.
 */
std::string KinngTransportUrl( const std::string& aSocketUrl );


class KINNG_REQUEST_SERVER
{
public:
    KINNG_REQUEST_SERVER( const std::string& aSocketUrl );

    ~KINNG_REQUEST_SERVER();

    bool Start();

    void Stop();

    bool Running() const;

    void SetCallback( std::function<void(std::string*)> aFunc ) { m_callback = aFunc; }

    void Reply( const std::string& aReply );

    const std::string& SocketPath() const { return m_socketUrl; }

private:
    void listenThread();

    std::thread m_thread;

    std::atomic<bool> m_shutdown;

    std::string m_socketUrl;

    std::function<void(std::string*)> m_callback;

    std::string m_sharedMessage;

    std::string m_pendingReply;

    std::condition_variable m_replyReady;

    std::mutex m_mutex;
};

#endif //KICAD_KINNG_H
