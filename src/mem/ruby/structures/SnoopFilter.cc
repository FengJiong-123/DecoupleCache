/*
 * Copyright (c) 2017,2019 ARM Limited
 * All rights reserved.
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
 * Copyright (c) 2017 Google Inc.
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

#include "mem/ruby/structures/SnoopFilter.hh"

#include "base/addr_range.hh"
#include "base/intmath.hh"
#include "debug/RubySF.hh"
#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "sim/system.hh"

namespace gem5
{

namespace ruby
{

SnoopFilter::SnoopFilter(const Params &p)
    : SimObject(p),
      m_dir_bit_num(p.dir_bit_num),
      m_dir_ass_num(p.dir_ass_num)
{
    assert(m_dir_bit_num != 0);
    assert(m_dir_ass_num <= m_dir_bit_num);
    m_dir_num_set = 1 << (m_dir_bit_num - m_dir_ass_num);
    m_dir_num_way = 1 << m_dir_ass_num;
    DPRINTF(RubySF, "Create dir with set:%lld, way:%lld\n"
                  , m_dir_num_set, m_dir_num_way);
}

void
SnoopFilter::init()
{
    m_dir.resize(m_dir_num_set,
                 std::vector<AbstractCacheEntry*>(m_dir_num_way, nullptr));
}

SnoopFilter::~SnoopFilter()
{
    for (int i = 0; i < m_dir_num_set; i++) {
        for (int j = 0; j < m_dir_num_way; j++) {
            delete m_dir[i][j];
        }
    }
}

void
SnoopFilter::print(std::ostream& out) const
{
}

AbstractCacheEntry* 
SnoopFilter::lookup(Addr addr, bool need_update_access_time)
{
    assert(addr == makeLineAddress(addr));
    int64_t dirSet = addressToCacheSet(addr);
    int loc = findTagInSet(dirSet, addr);
    if (loc == -1) return NULL;
    if (need_update_access_time) {
        m_dir[dirSet][loc]->setLastAccess(curTick());
    }
    return m_dir[dirSet][loc];
}

const AbstractCacheEntry*
SnoopFilter::lookup(Addr addr) const
{
    assert(addr == makeLineAddress(addr));
    int64_t dirSet = addressToCacheSet(addr);
    int loc = findTagInSet(dirSet, addr);
    if (loc == -1) return NULL;
    //m_dir[dirSet][loc]->setLastAccess(curTick());
    return m_dir[dirSet][loc];
}

bool
SnoopFilter::allocate(Addr addr, AbstractCacheEntry * new_entry)
{
    assert(addr == makeLineAddress(addr));
    assert(!isPresent(addr));
    //assert(setAvail(addr));
    DPRINTF(RubySF, "allocate::address: %#x\n", addr);

    if (!setAvail(addr)) {
        DPRINTF(RubySF, "allocate::set is full, need replacement\n");
        return false;
    }

    // Find the first open slot
    int64_t dirSet = addressToCacheSet(addr);
    std::vector<AbstractCacheEntry*> &set = m_dir[dirSet];
    for (int i = 0; i < m_dir_num_way; i++) {
        if (!set[i] || set[i]->m_Permission == AccessPermission_NotPresent) {
            if (set[i] && (set[i] != new_entry)) {
                warn_once("This protocol contains a cache entry handling bug: "
                    "Entries in the cache should never be NotPresent! If\n"
                    "this entry (%#x) is not tracked elsewhere, it will memory "
                    "leak here. Fix your protocol to eliminate these!",
                    addr);
            }
            set[i] = new_entry;  // Init entry
            set[i]->m_Address = addr;
            set[i]->m_Permission = AccessPermission_Invalid;
            DPRINTF(RubySF, "allocate::Allocate clearing lock for addr: %x\n",
                    addr);
            set[i]->m_locked = -1;
            m_tag_index[addr] = i;
            set[i]->setPosition(dirSet, i);
            //set[i]->replacementData = replacement_data[dirSet][i];
            set[i]->setLastAccess(curTick());

            // Call reset function here to set initial value for different
            // replacement policies.
            //m_replacementPolicy_ptr->reset(entry->replacementData);

            return true;
        }
    }
    panic("Allocate didn't find an available entry");
    //return false;
}

bool
SnoopFilter::isPresent(Addr addr)
{
    const AbstractCacheEntry* const entry = lookup(addr);
    if (entry == nullptr) {
        // We didn't find the tag
        DPRINTF(RubySF, "No tag match for address: %#x\n", addr);
        return false;
    }
    DPRINTF(RubySF, "isPresent::address: %#x found\n", addr);
    return true;
}

void 
SnoopFilter::deallocate(Addr addr)
{
    DPRINTF(RubySF, "deallocate::address: %#x\n", addr);
    AbstractCacheEntry* entry = lookup(addr, false);
    assert(entry != nullptr);
    //m_replacementPolicy_ptr->invalidate(entry->replacementData);
    uint32_t dirSet = entry->getSet();
    uint32_t assoc = entry->getWay();
    delete entry;
    m_dir[dirSet][assoc] = NULL;
    m_tag_index.erase(addr);
}

Addr 
SnoopFilter::cacheProbe(Addr addr)
{
    assert(addr == makeLineAddress(addr));
    assert(!setAvail(addr));
    DPRINTF(RubySF, "cacheProbe::address: %#x\n", addr);

    int64_t dirSet = addressToCacheSet(addr);
    Tick temp_tick = 0xffffffffffffffff;
    Addr addr_victim;
    for (auto it = m_dir[dirSet].begin(); it != m_dir[dirSet].end(); it ++) {
        assert((*it) != NULL);
        DPRINTF(RubySF, "cacheProbe::temp_tick=%lld, curr_tick=%lld, complete=%d\n"
                       , temp_tick, (*it)->getLastAccess(), (*it)->m_complete);
        if(temp_tick > (*it)->getLastAccess() && (*it)->m_complete) {
            addr_victim = (*it)->m_Address;
            temp_tick = (*it)->getLastAccess();
        }
    }
    DPRINTF(RubySF, "cacheProbe::newaddr=%x, victim_addr=%x\n"
                   , addr, addr_victim);
    return addr_victim;
}

void 
SnoopFilter::makeComplete(Addr addr)
{
    DPRINTF(RubySF, "makeComplete::address: %#x\n", addr);
    AbstractCacheEntry* entry = lookup(addr, false);
    assert(entry != nullptr);
    //m_replacementPolicy_ptr->invalidate(entry->replacementData);
    uint32_t dirSet = entry->getSet();
    uint32_t assoc = entry->getWay();
    m_dir[dirSet][assoc]->m_complete = true;

}

void 
SnoopFilter::makeInComplete(Addr addr)
{
    DPRINTF(RubySF, "makeInComplete::address: %#x\n", addr);
    AbstractCacheEntry* entry = lookup(addr, false);
    assert(entry != nullptr);
    //m_replacementPolicy_ptr->invalidate(entry->replacementData);
    uint32_t dirSet = entry->getSet();
    uint32_t assoc = entry->getWay();
    m_dir[dirSet][assoc]->m_complete = false;

}

// convert a Address to its location in the cache
int64_t
SnoopFilter::addressToCacheSet(Addr address) const
{
    assert(address == makeLineAddress(address));
    return bitSelect(address, 6,
                     6 + (m_dir_bit_num - m_dir_ass_num) - 1);
}

int
SnoopFilter::findTagInSet(int64_t dirSet, Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end())
        if (m_dir[dirSet][it->second]->m_Permission !=
            AccessPermission_NotPresent)
            return it->second;
    return -1; // Not found
}

bool
SnoopFilter::setAvail(Addr address) const
{
    assert(address == makeLineAddress(address));

    int64_t dirSet = addressToCacheSet(address);

    for (int i = 0; i < m_dir_num_way; i++) {
        AbstractCacheEntry* entry = m_dir[dirSet][i];
        if (entry != NULL) {
            if (entry->m_Address == address ||
                entry->m_Permission == AccessPermission_NotPresent) {
                // Already in the cache or we found an empty entry
                return true;
            }
        } else {
            return true;
        }
    }
    return false;
}

} // namespace ruby
} // namespace gem5
