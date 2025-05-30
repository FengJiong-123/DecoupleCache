/*
 * Copyright (c) 2020 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 1999-2008 Mark D. Hill and David A. Wood
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"

#include "base/trace.hh"
#include "debug/RubyCache.hh"

namespace gem5
{

namespace ruby
{

AbstractCacheEntry::AbstractCacheEntry() : ReplaceableEntry()
{
    m_Permission = AccessPermission_NotPresent;
    m_Address = 0;
    m_locked = -1;
    m_last_touch_tick = 0;
    m_htmInReadSet = false;
    m_htmInWriteSet = false;
    m_complete = false;
    m_waitEvict = false;
    m_is_backup = false;
    m_sharer_num= 0;
}

AbstractCacheEntry::~AbstractCacheEntry()
{
}

// Get cache permission
AccessPermission
AbstractCacheEntry::getPermission() const
{
    return m_Permission;
}

void
AbstractCacheEntry::changePermission(AccessPermission new_perm)
{
    m_Permission = new_perm;
    if ((new_perm == AccessPermission_Invalid) ||
        (new_perm == AccessPermission_NotPresent)) {
        m_locked = -1;
    }
}

void
AbstractCacheEntry::setLocked(int context)
{
    DPRINTF(RubyCache, "Setting Lock for addr: %#x to %d\n", m_Address, context);
    m_locked = context;
}

void
AbstractCacheEntry::clearLocked()
{
    DPRINTF(RubyCache, "Clear Lock for addr: %#x\n", m_Address);
    m_locked = -1;
}

bool
AbstractCacheEntry::isLocked(int context) const
{
    DPRINTF(RubyCache, "Testing Lock for addr: %#llx cur %d con %d\n",
            m_Address, m_locked, context);
    return m_locked == context;
}

void
AbstractCacheEntry::insertDirBk(Addr address, MachineID owner)
{
    DirInCacheEntry* new_dir_entry = new DirInCacheEntry{
        .address = address,
        .owner = owner,
        .state = "NP",
        .waitEvict = false,
        .isBlocked = true,
        .backupL2 = false
    };
    m_dir_bkup.push_back(new_dir_entry);
}

void
AbstractCacheEntry::setInHtmReadSet(bool val)
{
    m_htmInReadSet = val;
}

void
AbstractCacheEntry::setInHtmWriteSet(bool val)
{
    m_htmInWriteSet = val;
}

bool
AbstractCacheEntry::getInHtmReadSet() const
{
    return m_htmInReadSet;
}

bool
AbstractCacheEntry::getInHtmWriteSet() const
{
    return m_htmInWriteSet;
}

} // namespace ruby
} // namespace gem5
