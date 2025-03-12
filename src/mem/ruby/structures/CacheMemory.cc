/*
 * Copyright (c) 2020-2021 ARM Limited
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
 * Copyright (c) 1999-2012 Mark D. Hill and David A. Wood
 * Copyright (c) 2013 Advanced Micro Devices, Inc.
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

#include "mem/ruby/structures/CacheMemory.hh"

#include "base/compiler.hh"
#include "base/intmath.hh"
#include "base/logging.hh"
#include "debug/HtmMem.hh"
#include "debug/RubyCache.hh"
#include "debug/RubyCacheTrace.hh"
#include "debug/RubyResourceStalls.hh"
#include "debug/RubyStats.hh"
#include "mem/cache/replacement_policies/weighted_lru_rp.hh"
#include "mem/ruby/protocol/AccessPermission.hh"
#include "mem/ruby/system/RubySystem.hh"

namespace gem5
{

namespace ruby
{

std::ostream&
operator<<(std::ostream& out, const CacheMemory& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

CacheMemory::CacheMemory(const Params &p)
    : SimObject(p),
    dataArray(p.dataArrayBanks, p.dataAccessLatency,
              p.start_index_bit, p.ruby_system),
    tagArray(p.tagArrayBanks, p.tagAccessLatency,
             p.start_index_bit, p.ruby_system),
    atomicALUArray(p.atomicALUs, p.atomicLatency *
             p.ruby_system->clockPeriod()),
    cacheMemoryStats(this)
{
    m_cache_size = p.size;
    m_cache_assoc = p.assoc;
    m_cache_expand = p.expand;
    m_cache_block_num = p.block_num;
    m_cache_backup_entry = p.cache_backup;
    m_dir_tag_per_line = p.dir_tag_per_line;
    m_replacementPolicy_ptr = p.replacement_policy;
    m_new_replacement = p.new_replacement_policy;
    m_start_index_bit = p.start_index_bit;
    m_is_instruction_only_cache = p.is_icache;
    m_resource_stalls = p.resourceStalls;
    m_block_size = p.block_size;  // may be 0 at this point. Updated in init()
    m_use_occupancy = dynamic_cast<replacement_policy::WeightedLRU*>(
                                    m_replacementPolicy_ptr) ? true : false;
}

void
CacheMemory::init()
{
    if (m_block_size == 0) {
        m_block_size = RubySystem::getBlockSizeBytes();
    }
    m_cache_num_sets = (m_cache_size / m_cache_assoc) / m_block_size;
    assert(m_cache_num_sets > 1);
    m_cache_num_set_bits = floorLog2(m_cache_num_sets);
    assert(m_cache_num_set_bits > 0);
    m_cache_assoc = m_cache_assoc * m_cache_expand - m_cache_block_num;

    m_cache.resize(m_cache_num_sets,
                    std::vector<AbstractCacheEntry*>(m_cache_assoc, nullptr));
    replacement_data.resize(m_cache_num_sets,
                               std::vector<ReplData>(m_cache_assoc, nullptr));
    m_set_pending_addr.resize(m_cache_num_sets);
    // instantiate all the replacement_data here
    for (int i = 0; i < m_cache_num_sets; i++) {
        for ( int j = 0; j < m_cache_assoc; j++) {
            replacement_data[i][j] =
                                m_replacementPolicy_ptr->instantiateEntry();
        }
    }
    DPRINTF(RubyCache, "init::m_cache_size=%s, m_cache_assoc=%d, expand=%d, block_num=%d, m_cache_num_sets=%d\n"
                     , m_cache_size, m_cache_assoc, m_cache_expand, m_cache_block_num, m_cache_num_sets);
}

CacheMemory::~CacheMemory()
{
    if (m_replacementPolicy_ptr)
        delete m_replacementPolicy_ptr;
    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            delete m_cache[i][j];
        }
    }
}

// convert a Address to its location in the cache
int64_t
CacheMemory::addressToCacheSet(Addr address) const
{
    assert(address == makeLineAddress(address));
    return bitSelect(address, m_start_index_bit,
                     m_start_index_bit + m_cache_num_set_bits - 1);
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
int
CacheMemory::findTagInSet(int64_t cacheSet, Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end()) {
        if (m_cache[cacheSet][it->second]->m_Permission != AccessPermission_NotPresent) {
            DPRINTF(RubyCache, "findTagInSet::addr=%x, pos=%d\n"
                             , tag, it->second);
            return it->second;
        }
    }
    return -1; // Not found
}

// Given a cache index: returns the index of the tag in a set.
// returns -1 if the tag is not found.
int
CacheMemory::findTagInSetIgnorePermissions(int64_t cacheSet,
                                           Addr tag) const
{
    assert(tag == makeLineAddress(tag));
    // search the set for the tags
    auto it = m_tag_index.find(tag);
    if (it != m_tag_index.end())
        return it->second;
    return -1; // Not found
}

// Given an unique cache block identifier (idx): return the valid address
// stored by the cache block.  If the block is invalid/notpresent, the
// function returns the 0 address
Addr
CacheMemory::getAddressAtIdx(int idx) const
{
    Addr tmp(0);

    int set = idx / m_cache_assoc;
    assert(set < m_cache_num_sets);

    int way = idx - set * m_cache_assoc;
    assert (way < m_cache_assoc);

    AbstractCacheEntry* entry = m_cache[set][way];
    if (entry == NULL ||
        entry->m_Permission == AccessPermission_Invalid ||
        entry->m_Permission == AccessPermission_NotPresent) {
        return tmp;
    }
    return entry->m_Address;
}

bool
CacheMemory::tryCacheAccess(Addr address, RubyRequestType type,
                            DataBlock*& data_ptr)
{
    DPRINTF(RubyCache, "trying to access address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    if (entry != nullptr) {
        // Do we even have a tag match?
        m_replacementPolicy_ptr->touch(entry->replacementData);
        entry->setLastAccess(curTick());
        data_ptr = &(entry->getDataBlk());

        if (entry->m_Permission == AccessPermission_Read_Write) {
            DPRINTF(RubyCache, "Have permission to access address: %#x\n",
                        address);
            return true;
        }
        if ((entry->m_Permission == AccessPermission_Read_Only) &&
            (type == RubyRequestType_LD || type == RubyRequestType_IFETCH)) {
            DPRINTF(RubyCache, "Have permission to access address: %#x\n",
                        address);
            return true;
        }
        // The line must not be accessible
    }
    DPRINTF(RubyCache, "Do not have permission to access address: %#x\n",
                address);
    data_ptr = NULL;
    return false;
}

bool
CacheMemory::testCacheAccess(Addr address, RubyRequestType type,
                             DataBlock*& data_ptr)
{
    DPRINTF(RubyCache, "testing address: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    if (entry != nullptr) {
        // Do we even have a tag match?
        m_replacementPolicy_ptr->touch(entry->replacementData);
        entry->setLastAccess(curTick());
        data_ptr = &(entry->getDataBlk());

        DPRINTF(RubyCache, "have permission for address %#x?: %d\n",
                    address,
                    entry->m_Permission != AccessPermission_NotPresent);
        return entry->m_Permission != AccessPermission_NotPresent;
    }

    DPRINTF(RubyCache, "do not have permission for address %#x\n",
                address);
    data_ptr = NULL;
    return false;
}

// tests to see if an address is present in the cache
bool
CacheMemory::isTagPresent(Addr address) const
{
    const AbstractCacheEntry* const entry = lookup(address);
    if (entry == nullptr) {
        // We didn't find the tag
        DPRINTF(RubyCache, "isTagPresent::No tag match for addr=%x\n", address);
        return false;
    }
    DPRINTF(RubyCache, "isTagPresent::addr=%x found\n", address);
    return true;
}

// Returns true if there is:
//   a) a tag match on this address or there is
//   b) an unused line in the same cache "way"
//   c) cache line has been pending
bool
CacheMemory::cacheAvail(Addr address) const
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "cacheAvail::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    assert(m_set_pending_addr[cacheSet].size() <= m_cache_assoc);

    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        DPRINTF(RubyCache, "cacheAvail::hit pending::addr=%x, set=%x\n"
                         , address, cacheSet);
        return true;
    }

    //debug
    // auto itit = m_tag_index.find(0x1f0600);
    // if (itit != m_tag_index.end()) {DPRINTF(RubyCache, "cacheAvail::check for 0x1f0600 at pos=%d\n", itit->second);}

    int set_used_line = 0;
    for (int i = 0; i < m_cache_assoc; i++) {
        AbstractCacheEntry* entry = m_cache[cacheSet][i];
        if (entry != NULL) {
            if (entry->m_Address == address ||
                entry->m_Permission == AccessPermission_NotPresent) {
                // Already in the cache or we found an empty entry
                DPRINTF(RubyCache, "cacheAvail::has free entry::addr=%x, set=%x, pos=%d\n"
                                 , address, cacheSet, i);
                return true;
            }
        } else {
            //return true;
            set_used_line ++;
        }
    }
    DPRINTF(RubyCache, "cacheAvail::addr=%x, set=%x, pending size=%d, set_used_line=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size(), set_used_line);
    assert(set_used_line <= m_cache_assoc);
    //assert( (m_set_pending_addr[cacheSet].size() + set_used_line) <= m_cache_assoc);
    return  (set_used_line > m_set_pending_addr[cacheSet].size());
}

bool
CacheMemory::cacheFullbuthasPend(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "cacheFullbuthasPend::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    // cache full?
    for (int i = 0; i < m_cache_assoc; i++) {
        AbstractCacheEntry* entry = m_cache[cacheSet][i];
        if (entry == NULL) {
            return false;
        }
    }
    // pending?
    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        DPRINTF(RubyCache, "cacheFullbuthasPend::hit pending::addr=%x, set=%x\n"
                         , address, cacheSet);
        return true;
    }
    return false;
}

bool
CacheMemory::cachehasOtherPend(Addr address, MachineID requestor) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "cachehasOtherPend::addr=%x, set=%x, pending size=%d, requestor=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size(), requestor);
    // pending?
    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        DPRINTF(RubyCache, "cachehasOtherPend::hit pending::addr=%x, set=%x, pending requestor=%d\n"
                         , address, cacheSet, it->second);
        if (it->second != requestor) {
            return true;
        }
    }
    return false;
}

bool
CacheMemory::cacheBlock(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "cacheBlock::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    assert(m_set_pending_addr[cacheSet].size() <= m_cache_assoc);

    if (m_set_pending_addr[cacheSet].size() < m_cache_assoc) {
        return false;
    }
    // for (auto it = m_set_pending_addr[cacheSet].begin(); it != m_set_pending_addr[cacheSet].end();) {
    //     if ((*it) == address) {
    //         DPRINTF(RubyCache, "cacheBlock::hit pending::addr=%x, set=%x\n"
    //                          , address, cacheSet);
    //         return false;
    //     } else {
    //         it ++;
    //     }
    // }
    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        DPRINTF(RubyCache, "cacheBlock::hit pending::addr=%x, set=%x\n"
                         , address, cacheSet);
        return false;
    }
    return true;
}

AbstractCacheEntry*
CacheMemory::allocate(Addr address, AbstractCacheEntry *entry)
{
    assert(address == makeLineAddress(address));
    assert(!isTagPresent(address));
    assert(cacheAvail(address));
    DPRINTF(RubyCache, "allocating address: %#x\n", address);
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "allocate::remove pend if need::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    assert(m_set_pending_addr[cacheSet].size() <= m_cache_assoc);

    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        m_set_pending_addr[cacheSet].erase(it);
    }

    //debug
    // auto itit = m_tag_index.find(0x1f0600);
    // if (itit != m_tag_index.end()) {DPRINTF(RubyCache, "allocate::check for 0x1f0600 at pos=%d\n", itit->second);}

    // Find the first open slot
    std::vector<AbstractCacheEntry*> &set = m_cache[cacheSet];
    for (int i = 0; i < m_cache_assoc; i++) {
        // for debug only
        // if (!set[i]) {
        //     DPRINTF(RubyCache, "allocate::this pos %d is free \n", i);
        // } else {
        //     DPRINTF(RubyCache, "allocate::this pos %d has addr=%x with permission=%d\n"
        //                      , i, set[i]->m_Address, set[i]->m_Permission);
        // }
        if (!set[i] || set[i]->m_Permission == AccessPermission_NotPresent) {
            if (set[i] && (set[i] != entry)) {
                warn_once("This protocol contains a cache entry handling bug: "
                    "Entries in the cache should never be NotPresent! If\n"
                    "this entry (%#x) is not tracked elsewhere, it will memory "
                    "leak here. Fix your protocol to eliminate these!",
                    address);
            }
            set[i] = entry;  // Init entry
            set[i]->m_Address = address;
            set[i]->m_Permission = AccessPermission_Invalid;
            DPRINTF(RubyCache, "Allocate clearing lock for addr=%x pos=%d\n",
                    address, i);
            set[i]->m_locked = -1;
            m_tag_index[address] = i;
            set[i]->setPosition(cacheSet, i);
            //set[i]->replacementData = replacement_data[cacheSet][i];
            set[i]->setLastAccess(curTick());
            assert(findTagInSet(cacheSet, address) == i);

            // Call reset function here to set initial value for different
            // replacement policies.
            //m_replacementPolicy_ptr->reset(entry->replacementData);

            return entry;
        }
    }
    panic("Allocate didn't find an available entry");
}

void
CacheMemory::deallocate(Addr address)
{
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    DPRINTF(RubyCache, "deallocate::addr=%x, addr in entry=%x\n", address, entry->m_Address);
    assert(entry->m_Address == address);
    //m_replacementPolicy_ptr->invalidate(entry->replacementData);
    uint32_t cache_set = entry->getSet();
    uint32_t way = entry->getWay();
    entry->setLastAccess(0);
    assert(m_cache[cache_set][way]->m_Address == address);
    delete entry;
    m_cache[cache_set][way] = NULL;
    m_tag_index.erase(address);
}

// int64_t cacheSet = addressToCacheSet(address);
//     int pos = m_cache_assoc;
//     for (int i = 0; i < m_cache_assoc; i++) {
//         if (m_cache[cacheSet][i] == NULL) {
//             continue;
//         }
//         if (m_cache[cacheSet][i]->m_Address == address) {
//             pos = i;
//             break;
//         }
//     }
//     delete & m_cache[cacheSet][pos];
//     m_cache[cacheSet][pos] = NULL;
//     m_tag_index.erase(address);
//     DPRINTF(RubyCache, "deallocate::addr=%x, cacheSet=%d, way=%d\n"
//                      , address, cacheSet, pos);

// Returns with the physical address of the conflicting cache line
Addr
CacheMemory::cacheProbe(Addr address) const
{
    assert(address == makeLineAddress(address));
    assert(!cacheAvail(address));

    int64_t cacheSet = addressToCacheSet(address);
    std::vector<ReplaceableEntry*> candidates;
    DPRINTF(RubyCache, "cacheProbe::cacheSet=%d\n", cacheSet);

    //debug
    // auto itit = m_tag_index.find(0x1f0600);
    // if (itit != m_tag_index.end()) {DPRINTF(RubyCache, "cacheProbe::check for 0x1f0600 at pos=%d\n", itit->second);}

    // use new replacement policy to deter victim
    if (m_new_replacement == 0) {
        Tick least_tick = curTick() + 1;
        int victim_pos = m_cache_assoc;
        //auto least_tick_iter = m_cache[cacheSet].end();
        //for (auto it = m_cache[cacheSet].begin(); it != m_cache[cacheSet].end(); it ++) {
        //for (auto it : m_cache[cacheSet]) {
        for (int i = 0; i < m_cache_assoc; i ++) {
            if (m_cache[cacheSet][i] != NULL) {
                DPRINTF(RubyCache, "cacheProbe::addr=%x, pos=%d, lastAccess=%lld, waitEvict=%d\n"
                                 , m_cache[cacheSet][i]->m_Address, i
                                 , m_cache[cacheSet][i]->getLastAccess()
                                 , m_cache[cacheSet][i]->m_waitEvict);
                assert(m_cache[cacheSet][i]->getLastAccess() < curTick() + 1);
                if (m_cache[cacheSet][i]->getLastAccess() < least_tick) {
                    least_tick = m_cache[cacheSet][i]->getLastAccess();
                    victim_pos = i;
                }
            } else {
                DPRINTF(RubyCache, "cacheProbe::entry is null, pos=%d\n", i);
            }
        }
        assert(victim_pos < m_cache_assoc);
        assert(m_cache[cacheSet][victim_pos] != NULL);
        assert(m_cache[cacheSet][victim_pos]->getLastAccess() < curTick() + 1);
        m_cache[cacheSet][victim_pos]->m_waitEvict = true;
        return m_cache[cacheSet][victim_pos]->m_Address;

    } else if (m_new_replacement == 1) {
        // LRU with waitEvict
        Tick least_tick = curTick() + 1;
        int victim_pos = m_cache_assoc;
        for (int i = 0; i < m_cache_assoc; i ++) {
            if (m_cache[cacheSet][i] != NULL) {
                DPRINTF(RubyCache, "cacheProbe::addr=%x, pos=%d, lastAccess=%lld, waitEvict=%d\n"
                                 , m_cache[cacheSet][i]->m_Address, i
                                 , m_cache[cacheSet][i]->getLastAccess()
                                 , m_cache[cacheSet][i]->m_waitEvict);
                assert(m_cache[cacheSet][i]->getLastAccess() < curTick() + 1);
                if (m_cache[cacheSet][i]->getLastAccess() < least_tick &&
                   !m_cache[cacheSet][i]->m_waitEvict) {
                    least_tick = m_cache[cacheSet][i]->getLastAccess();
                    victim_pos = i;
                }
            } else {
                DPRINTF(RubyCache, "cacheProbe::entry is null, pos=%d\n", i);
            }
        }
        assert(victim_pos <= m_cache_assoc);
        if (victim_pos == m_cache_assoc) {
            // means all entry is null
            assert(m_set_pending_addr[cacheSet].size() > 0);
            Addr victim = m_set_pending_addr[cacheSet].begin()->first;
            // TODO:switch to the end
            return victim;
        }
        assert(m_cache[cacheSet][victim_pos] != NULL);
        assert(m_cache[cacheSet][victim_pos]->getLastAccess() < curTick() + 1);
        assert(!m_cache[cacheSet][victim_pos]->m_waitEvict);
        m_cache[cacheSet][victim_pos]->m_waitEvict = true;
        return m_cache[cacheSet][victim_pos]->m_Address;
        
    } else if (m_new_replacement == 2) {
        // LRU with waitEvict
        Tick least_tick = curTick() + 1;
        int victim_pos = m_cache_assoc;
        for (int i = 0; i < m_cache_assoc; i ++) {
            if (m_cache[cacheSet][i] != NULL) {
                DPRINTF(RubyCache, "cacheProbe::addr=%x, pos=%d, lastAccess=%lld, waitEvict=%d, is backup=%d\n"
                                 , m_cache[cacheSet][i]->m_Address, i
                                 , m_cache[cacheSet][i]->getLastAccess()
                                 , m_cache[cacheSet][i]->m_waitEvict
                                 , m_cache[cacheSet][i]->m_is_backup);
                assert(m_cache[cacheSet][i]->getLastAccess() < curTick() + 1);
                if (m_cache[cacheSet][i]->getLastAccess() < least_tick &&
                   !m_cache[cacheSet][i]->m_waitEvict &&
                   !m_cache[cacheSet][i]->m_is_backup) {
                    least_tick = m_cache[cacheSet][i]->getLastAccess();
                    victim_pos = i;
                }
            } else {
                DPRINTF(RubyCache, "cacheProbe::entry is null, pos=%d\n", i);
            }
        }
        assert(victim_pos <= m_cache_assoc);
        if (victim_pos == m_cache_assoc) {
            // means all entry is null
            assert(m_set_pending_addr[cacheSet].size() > 0);
            Addr victim = m_set_pending_addr[cacheSet].begin()->first;
            // TODO:switch to the end
            return victim;
        }
        assert(m_cache[cacheSet][victim_pos] != NULL);
        assert(m_cache[cacheSet][victim_pos]->getLastAccess() < curTick() + 1);
        assert(!m_cache[cacheSet][victim_pos]->m_waitEvict);
        m_cache[cacheSet][victim_pos]->m_waitEvict = true;
        return m_cache[cacheSet][victim_pos]->m_Address;
    } else {
        assert(false);
    }

    return 0;
}

// looks an address up in the cache
AbstractCacheEntry*
CacheMemory::lookup(Addr address)
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc == -1) return NULL;
    return m_cache[cacheSet][loc];
}

// looks an address up in the cache
const AbstractCacheEntry*
CacheMemory::lookup(Addr address) const
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    int loc = findTagInSet(cacheSet, address);
    if (loc == -1) return NULL;
    return m_cache[cacheSet][loc];
}

// Sets the most recently used bit for a cache block
void
CacheMemory::setMRU(Addr address)
{
    AbstractCacheEntry* entry = lookup(makeLineAddress(address));
    if (entry != nullptr) {
        //m_replacementPolicy_ptr->touch(entry->replacementData);
        entry->setLastAccess(curTick());
    }
}

void
CacheMemory::setMRU(AbstractCacheEntry *entry)
{
    assert(entry != nullptr);
    //m_replacementPolicy_ptr->touch(entry->replacementData);
    entry->setLastAccess(curTick());
}

void
CacheMemory::setMRU(Addr address, int occupancy)
{
    assert(false);
}

int
CacheMemory::getReplacementWeight(int64_t set, int64_t loc)
{
    assert(set < m_cache_num_sets);
    assert(loc < m_cache_assoc);
    int ret = 0;
    if (m_cache[set][loc] != NULL) {
        ret = m_cache[set][loc]->getNumValidBlocks();
        assert(ret >= 0);
    }

    return ret;
}

void
CacheMemory::recordCacheContents(int cntrl, CacheRecorder* tr) const
{
    uint64_t warmedUpBlocks = 0;
    [[maybe_unused]] uint64_t totalBlocks = (uint64_t)m_cache_num_sets *
                                         (uint64_t)m_cache_assoc;

    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            if (m_cache[i][j] != NULL) {
                AccessPermission perm = m_cache[i][j]->m_Permission;
                RubyRequestType request_type = RubyRequestType_NULL;
                if (perm == AccessPermission_Read_Only) {
                    if (m_is_instruction_only_cache) {
                        request_type = RubyRequestType_IFETCH;
                    } else {
                        request_type = RubyRequestType_LD;
                    }
                } else if (perm == AccessPermission_Read_Write) {
                    request_type = RubyRequestType_ST;
                }

                if (request_type != RubyRequestType_NULL) {
                    Tick lastAccessTick;
                    lastAccessTick = m_cache[i][j]->getLastAccess();
                    tr->addRecord(cntrl, m_cache[i][j]->m_Address,
                                  0, request_type, lastAccessTick,
                                  m_cache[i][j]->getDataBlk());
                    warmedUpBlocks++;
                }
            }
        }
    }

    DPRINTF(RubyCacheTrace, "%s: %lli blocks of %lli total blocks"
            "recorded %.2f%% \n", name().c_str(), warmedUpBlocks,
            totalBlocks, (float(warmedUpBlocks) / float(totalBlocks)) * 100.0);
}

void
CacheMemory::print(std::ostream& out) const
{
    out << "Cache dump: " << name() << std::endl;
    for (int i = 0; i < m_cache_num_sets; i++) {
        for (int j = 0; j < m_cache_assoc; j++) {
            if (m_cache[i][j] != NULL) {
                out << "  Index: " << i
                    << " way: " << j
                    << " entry: " << *m_cache[i][j] << std::endl;
            } else {
                out << "  Index: " << i
                    << " way: " << j
                    << " entry: NULL" << std::endl;
            }
        }
    }
}

void
CacheMemory::printData(std::ostream& out) const
{
    out << "printData() not supported" << std::endl;
}

void
CacheMemory::setLocked(Addr address, int context)
{
    DPRINTF(RubyCache, "Setting Lock for addr: %#x to %d\n", address, context);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    entry->setLocked(context);
}

void
CacheMemory::clearLocked(Addr address)
{
    DPRINTF(RubyCache, "Clear Lock for addr: %#x\n", address);
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    entry->clearLocked();
}

void
CacheMemory::clearLockedAll(int context)
{
    // iterate through every set and way to get a cache line
    for (auto i = m_cache.begin(); i != m_cache.end(); ++i) {
        std::vector<AbstractCacheEntry*> set = *i;
        for (auto j = set.begin(); j != set.end(); ++j) {
            AbstractCacheEntry *line = *j;
            if (line && line->isLocked(context)) {
                DPRINTF(RubyCache, "Clear Lock for addr: %#x\n",
                    line->m_Address);
                line->clearLocked();
            }
        }
    }
}

bool
CacheMemory::isLocked(Addr address, int context)
{
    AbstractCacheEntry* entry = lookup(address);
    assert(entry != nullptr);
    DPRINTF(RubyCache, "Testing Lock for addr: %#llx cur %d con %d\n",
            address, entry->m_locked, context);
    return entry->isLocked(context);
}

void
CacheMemory::savePendingAddr(Addr address, MachineID machineID)
{
    assert(address == makeLineAddress(address));
    if (isTagPresent(address)) {
        return;
    }

    // Find the first open slot
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "savePendingAddr::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    assert(m_set_pending_addr[cacheSet].size() <= m_cache_assoc);

    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        return;
    }
    //m_set_pending_addr[cacheSet].push_back(address);
    m_set_pending_addr[cacheSet][address] = machineID;
}

bool
CacheMemory::isPendingAddr(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "isPendingAddr::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        return true;
    }
    return false;
}

void
CacheMemory::removePendingAddr(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "removePendingAddr::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    assert(m_set_pending_addr[cacheSet].size() <= m_cache_assoc);

    auto it = m_set_pending_addr[cacheSet].find(address);
    if (it != m_set_pending_addr[cacheSet].end()) {
        m_set_pending_addr[cacheSet].erase(it);
    }
}

bool
CacheMemory::isFreetoBackup(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "isFreetoBackup::addr=%x, set=%x, pending size=%d\n"
                     , address, cacheSet, m_set_pending_addr[cacheSet].size());
    if (m_cache_backup_entry == 0) {
        return false;
    }

    // get back up entry number
    int backup_entry = 0;
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        if (m_cache[cacheSet][i]->m_is_backup) {
            backup_entry ++;
            assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
            if (m_cache[cacheSet][i]->m_dir_bkup.size() < m_dir_tag_per_line) {
                return true;
            }
        }
    }
    assert(backup_entry <= m_cache_backup_entry);
    return (backup_entry < m_cache_backup_entry);
}

bool
CacheMemory::isDirinBackup(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "isDirinBackup::addr=%x, set=%x\n"
                     , address, cacheSet);
    if (m_cache_backup_entry == 0) {
        return false;
    }
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        if (m_cache[cacheSet][i]->m_is_backup) {
            assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
            for (auto it = m_cache[cacheSet][i]->m_dir_bkup.begin(); it != m_cache[cacheSet][i]->m_dir_bkup.end(); it ++) {
                if ((*it)->address == address) {
                    assert(findTagInSet(cacheSet, address) == i);
                    return true;
                }
            }
        }
    }
    return false;
}

std::string
CacheMemory::getbkDirState(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "getbkDirState::addr=%x, set=%x\n"
                     , address, cacheSet);
    assert(m_cache_backup_entry != 0);
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        if (m_cache[cacheSet][i]->m_is_backup) {
            assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
            for (auto it = m_cache[cacheSet][i]->m_dir_bkup.begin(); it != m_cache[cacheSet][i]->m_dir_bkup.end(); it ++) {
                if ((*it)->address == address) {
                    assert(findTagInSet(cacheSet, address) == i);
                    return (*it)->state;
                }
            }
        }
    }
    assert(false);
    return "NP";
}

MachineID
CacheMemory::getSharerinBackup(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "getSharerinBackup::addr=%x, set=%x\n"
                     , address, cacheSet);
    assert(m_cache_backup_entry != 0);
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        if (m_cache[cacheSet][i]->m_is_backup) {
            assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
            for (auto it = m_cache[cacheSet][i]->m_dir_bkup.begin(); it != m_cache[cacheSet][i]->m_dir_bkup.end(); it ++) {
                if ((*it)->address == address) {
                    assert(findTagInSet(cacheSet, address) == i);
                    return (*it)->sharer;
                }
            }
        }
    }
    assert(false);
    //return "NP";
}

AbstractCacheEntry*
CacheMemory::insertDirBk(Addr address, MachineID sharer, AbstractCacheEntry *entry) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "insertDirBk::addr=%x, set=%x\n"
                     , address, cacheSet);

    int insert_pos = m_cache_assoc;
    int back_entry = 0;
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            insert_pos = i;
            continue;
        }
        DPRINTF(RubyCache, "insertDirBk::pos=%d, m_is_backup=%d, dir size=%d\n"
                         , i, m_cache[cacheSet][i]->m_is_backup, m_cache[cacheSet][i]->m_dir_bkup.size());
        if (m_cache[cacheSet][i]->m_is_backup) {
            // count back entry
            back_entry ++;
        }
        assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
        // get an entry has been set as backup
        if (m_cache[cacheSet][i]->m_is_backup && 
            m_cache[cacheSet][i]->m_dir_bkup.size() < m_dir_tag_per_line) {
            m_cache[cacheSet][i]->insertDirBk(address, sharer);
            m_cache[cacheSet][i]->setLastAccess(curTick());
            m_tag_index[address] = i;
            delete entry;
            DPRINTF(RubyCache, "insertDirBk::back in existed entry\n");
            return m_cache[cacheSet][i];
        }
    }
    if (insert_pos < m_cache_assoc && back_entry < m_cache_backup_entry) {
        assert(m_cache[cacheSet][insert_pos] == NULL);
        //AbstractCacheEntry* new_cache_entry = new AbstractCacheEntry;
        m_cache[cacheSet][insert_pos] = entry;  // Init entry
        m_cache[cacheSet][insert_pos]->m_Address = -1;
        m_cache[cacheSet][insert_pos]->m_Permission = AccessPermission_Invalid;
        m_cache[cacheSet][insert_pos]->m_locked = -1;
        m_tag_index[address] = insert_pos;
        m_cache[cacheSet][insert_pos]->setPosition(cacheSet, insert_pos);
        m_cache[cacheSet][insert_pos]->setLastAccess(curTick());
        m_cache[cacheSet][insert_pos]->m_is_backup = true;
        m_cache[cacheSet][insert_pos]->insertDirBk(address, sharer);
        DPRINTF(RubyCache, "insertDirBk::back in new entry\n");
        return m_cache[cacheSet][insert_pos];
    }
    panic("can't insert a backup dir entry!!!");
}

void
CacheMemory::removeDirBk(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);

    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
        // get an entry has been set as backup
        if (m_cache[cacheSet][i]->m_is_backup) {
            for (auto it = m_cache[cacheSet][i]->m_dir_bkup.begin(); it != m_cache[cacheSet][i]->m_dir_bkup.end();) {
                if ((*it)->address == address) {
                    assert(findTagInSet(cacheSet, address) == i);
                    it = m_cache[cacheSet][i]->m_dir_bkup.erase(it);
                    // clean up tag index
                    m_tag_index.erase(address);
                    if (m_cache[cacheSet][i]->m_dir_bkup.size() > 0) {
                        DPRINTF(RubyCache, "removeDirBk::entry is keep::addr=%x, set=%x, size=%d\n"
                                         , address, cacheSet, m_cache[cacheSet][i]->m_dir_bkup.size());
                        m_cache[cacheSet][i]->setLastAccess(curTick());
                        return;
                    } else {
                        break;
                    }
                } else {
                    it ++;
                }
            }
            if (m_cache[cacheSet][i]->m_dir_bkup.size() == 0) {
                // clean up the back up entry
                DPRINTF(RubyCache, "removeDirBk::delete entry::addr=%x, set=%x\n"
                                 , address, cacheSet);
                //AbstractCacheEntry* entry = m_cache[cacheSet][i];
                assert(m_cache[cacheSet][i] != nullptr);
                m_cache[cacheSet][i]->setLastAccess(0);
                delete m_cache[cacheSet][i];
                m_cache[cacheSet][i] = NULL;
                return;
            }
        }
    }
    panic("can't remove a backup dir entry!!!");
}

void
CacheMemory::setDirState(Addr address, const std::string& state) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "setDirState::addr=%x, set=%x, state=%s\n"
                     , address, cacheSet, state);

    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        assert(m_cache[cacheSet][i]->m_dir_bkup.size() <= m_dir_tag_per_line);
        DPRINTF(RubyCache, "setDirState::pos=%d, addr=%x, m_is_backup=%d, dir size=%d\n"
                         , i, m_cache[cacheSet][i]->m_Address
                         , m_cache[cacheSet][i]->m_is_backup, m_cache[cacheSet][i]->m_dir_bkup.size());
        // get an entry has been set as backup
        if (m_cache[cacheSet][i]->m_is_backup) {
            for (auto it = m_cache[cacheSet][i]->m_dir_bkup.begin(); it != m_cache[cacheSet][i]->m_dir_bkup.end(); it ++) {
                if ((*it)->address == address) {
                    (*it)->state = state;
                    return;
                }
            }
        }
    }
    panic("can't set a backup dir entry state!!!");
}

Addr
//CacheMemory::cacheProbetoBk(Addr address, AbstractCacheEntry *entry)
CacheMemory::cacheProbetoBk(Addr address)
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    //DPRINTF(RubyCache, "cacheProbetoBk::addr=%x, cacheSet=%x\n",address, cacheSet);
    Tick least_tick = curTick() + 1;
    int victim_pos = m_cache_assoc;
    //int null_pos = m_cache_assoc;
    // get back up entry number
    // int backup_entry = 0;
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] != NULL) {
            DPRINTF(RubyCache, "cacheProbetoBk::addr=%x, pos=%d, lastAccess=%lld, waitEvict=%d, m_is_backup=%d, dir entry size=%d\n"
                             , m_cache[cacheSet][i]->m_Address, i
                             , m_cache[cacheSet][i]->getLastAccess()
                             , m_cache[cacheSet][i]->m_waitEvict
                             , m_cache[cacheSet][i]->m_is_backup
                             , m_cache[cacheSet][i]->m_dir_bkup.size());
            assert(m_cache[cacheSet][i]->getLastAccess() < curTick() + 1);
            // if (m_cache[cacheSet][i]->m_is_backup) {
            //     backup_entry ++;
            // }
            if (m_cache[cacheSet][i]->m_is_backup && m_cache[cacheSet][i]->m_dir_bkup.size() < m_dir_tag_per_line) {
                // this entry has been occupied and is still available
                m_cache[cacheSet][i]->setLastAccess(curTick());
                // set up index
                // m_tag_index[address] = i;
                // delete entry;
                return -1;
            }
            if (m_cache[cacheSet][i]->getLastAccess() < least_tick &&
                   !m_cache[cacheSet][i]->m_waitEvict &&
                   !m_cache[cacheSet][i]->m_is_backup) {
                least_tick = m_cache[cacheSet][i]->getLastAccess();
                victim_pos = i;
            }
        } else {
            DPRINTF(RubyCache, "cacheProbetoBk::entry is null, pos=%d\n", i);
            // could return
            // null_pos = i;
            return -1;
        }
    }
    // if (null_pos < m_cache_assoc && backup_entry < m_dir_tag_per_line) {
    //     // allocate a new entry and set to be backup
    //     assert(m_cache[cacheSet][null_pos] == NULL);
    //     //AbstractCacheEntry* new_cache_entry = new AbstractCacheEntry;
    //     m_cache[cacheSet][null_pos] = entry;  // Init entry
    //     m_cache[cacheSet][null_pos]->m_Address = -1;
    //     m_cache[cacheSet][null_pos]->m_Permission = AccessPermission_Invalid;
    //     m_cache[cacheSet][null_pos]->m_locked = -1;
    //     //m_tag_index[address] = null_pos;
    //     m_cache[cacheSet][null_pos]->setPosition(cacheSet, null_pos);
    //     m_cache[cacheSet][null_pos]->setLastAccess(curTick());
    //     m_cache[cacheSet][null_pos]->m_is_backup = true;
    //     return -1;
    // }

    assert(victim_pos < m_cache_assoc);
    assert(m_cache[cacheSet][victim_pos] != NULL);
    assert(m_cache[cacheSet][victim_pos]->getLastAccess() < curTick() + 1);
    m_cache[cacheSet][victim_pos]->m_waitEvict = true;
    // delete entry;
    return m_cache[cacheSet][victim_pos]->m_Address;
}

void
CacheMemory::reviseIndex(Addr address) {
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    DPRINTF(RubyCache, "reviseIndex::addr=%x, set=%x\n"
                     , address, cacheSet);
    int pos = m_cache_assoc;
    for (int i = 0; i < m_cache_assoc; i ++) {
        if (m_cache[cacheSet][i] == NULL) {
            continue;
        }
        if (m_cache[cacheSet][i]->m_is_backup) {
            for (auto it = m_cache[cacheSet][i]->m_dir_bkup.begin(); it != m_cache[cacheSet][i]->m_dir_bkup.end(); it ++) {
                if ((*it)->address == address) {
                    // dir in back up check
                    assert(false);
                }
            }
        }
        if (m_cache[cacheSet][i]->m_Address) {
            pos = i;
        }
    }
    assert(pos < m_cache_assoc);
    m_tag_index[address] = pos;
}

// void
// CacheMemory::updateSharerNum(Addr address, int sharersNum) {
//     assert(address == makeLineAddress(address));
//     int64_t cacheSet = addressToCacheSet(address);
//     DPRINTF(RubyCache, "updateSharerNum::addr=%x, set=%x, sharersNum=%d\n"
//                      , address, cacheSet, sharersNum);

//     for (int i = 0; i < m_cache_assoc; i ++) {
//         if (m_cache[cacheSet][i] == NULL) {
//             continue;
//         }
//         // get an entry has been set as backup
//         if (m_cache[cacheSet][i]->address == address) {
//             m_cache[cacheSet][i]->m_sharer_num = sharersNum;
//         }
//     }
// }


CacheMemory::
CacheMemoryStats::CacheMemoryStats(statistics::Group *parent)
    : statistics::Group(parent),
      ADD_STAT(numDataArrayReads, "Number of data array reads"),
      ADD_STAT(numDataArrayWrites, "Number of data array writes"),
      ADD_STAT(numTagArrayReads, "Number of tag array reads"),
      ADD_STAT(numTagArrayWrites, "Number of tag array writes"),
      ADD_STAT(numTagArrayStalls, "Number of stalls caused by tag array"),
      ADD_STAT(numDataArrayStalls, "Number of stalls caused by data array"),
      ADD_STAT(numAtomicALUOperations, "Number of atomic ALU operations"),
      ADD_STAT(numAtomicALUArrayStalls, "Number of stalls caused by atomic ALU array"),
      ADD_STAT(htmTransCommitReadSet, "Read set size of a committed "
                                      "transaction"),
      ADD_STAT(htmTransCommitWriteSet, "Write set size of a committed "
                                       "transaction"),
      ADD_STAT(htmTransAbortReadSet, "Read set size of a aborted transaction"),
      ADD_STAT(htmTransAbortWriteSet, "Write set size of a aborted "
                                      "transaction"),
      ADD_STAT(m_demand_hits, "Number of cache demand hits"),
      ADD_STAT(m_demand_misses, "Number of cache demand misses"),
      ADD_STAT(m_demand_accesses, "Number of cache demand accesses",
               m_demand_hits + m_demand_misses),
      ADD_STAT(m_prefetch_hits, "Number of cache prefetch hits"),
      ADD_STAT(m_prefetch_misses, "Number of cache prefetch misses"),
      ADD_STAT(m_prefetch_accesses, "Number of cache prefetch accesses",
               m_prefetch_hits + m_prefetch_misses),
      ADD_STAT(m_accessModeType, ""),
      ADD_STAT(m_evictions, "Number of evictions"),
      ADD_STAT(m_evict_putx, "Number of evictions from PUTX"),
      ADD_STAT(m_evict_dir_evict, "Number of evictions from directory eviction"),
      ADD_STAT(m_evict_dir_evict_dirty, "Number of evictions from directory eviction and is dirty"),
      ADD_STAT(m_evict_share, "Number of evictions from data sharing"),
      ADD_STAT(m_remained_entry_0, "remained entry is 0"),
      ADD_STAT(m_remained_entry_1, "remained entry is 1"),
      ADD_STAT(m_remained_entry_2, "remained entry is 2"),
      ADD_STAT(m_remained_entry_3_8, "remained entry is 3 to 8"),
      ADD_STAT(m_remained_entry_8_, "remained entry is over 8"),
      ADD_STAT(m_sharer_num_1, "evict entry has 1 sharer"),
      ADD_STAT(m_sharer_num_2, "evict entry has 2 sharers"),
      ADD_STAT(m_sharer_num_3, "evict entry has 3 sharers"),
      ADD_STAT(m_sharer_num_4, "evict entry has 4 sharers"),
      ADD_STAT(m_sharer_num_4_, "evict entry has over 4 sharers")
{
    numDataArrayReads
        .flags(statistics::nozero);

    numDataArrayWrites
        .flags(statistics::nozero);

    numTagArrayReads
        .flags(statistics::nozero);

    numTagArrayWrites
        .flags(statistics::nozero);

    numTagArrayStalls
        .flags(statistics::nozero);

    numDataArrayStalls
        .flags(statistics::nozero);

    numAtomicALUOperations
        .flags(statistics::nozero);

    numAtomicALUArrayStalls
        .flags(statistics::nozero);

    htmTransCommitReadSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransCommitWriteSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransAbortReadSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    htmTransAbortWriteSet
        .init(8)
        .flags(statistics::pdf | statistics::dist | statistics::nozero |
            statistics::nonan);

    m_prefetch_hits
        .flags(statistics::nozero);

    m_prefetch_misses
        .flags(statistics::nozero);

    m_prefetch_accesses
        .flags(statistics::nozero);

    m_accessModeType
        .init(RubyRequestType_NUM)
        .flags(statistics::pdf | statistics::total);

    for (int i = 0; i < RubyAccessMode_NUM; i++) {
        m_accessModeType
            .subname(i, RubyAccessMode_to_string(RubyAccessMode(i)))
            .flags(statistics::nozero)
            ;
    }
}

// assumption: SLICC generated files will only call this function
// once **all** resources are granted
void
CacheMemory::recordRequestType(CacheRequestType requestType, Addr addr)
{
    DPRINTF(RubyStats, "Recorded statistic: %s\n",
            CacheRequestType_to_string(requestType));
    switch(requestType) {
    case CacheRequestType_DataArrayRead:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numDataArrayReads++;
        return;
    case CacheRequestType_DataArrayWrite:
        if (m_resource_stalls)
            dataArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numDataArrayWrites++;
        return;
    case CacheRequestType_TagArrayRead:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numTagArrayReads++;
        return;
    case CacheRequestType_TagArrayWrite:
        if (m_resource_stalls)
            tagArray.reserve(addressToCacheSet(addr));
        cacheMemoryStats.numTagArrayWrites++;
        return;
    case CacheRequestType_AtomicALUOperation:
        if (m_resource_stalls)
            atomicALUArray.reserve(addr);
        cacheMemoryStats.numAtomicALUOperations++;
        return;
    default:
        warn("CacheMemory access_type not found: %s",
             CacheRequestType_to_string(requestType));
    }
}

bool
CacheMemory::checkResourceAvailable(CacheResourceType res, Addr addr)
{
    if (!m_resource_stalls) {
        return true;
    }

    if (res == CacheResourceType_TagArray) {
        if (tagArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Tag array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            cacheMemoryStats.numTagArrayStalls++;
            return false;
        }
    } else if (res == CacheResourceType_DataArray) {
        if (dataArray.tryAccess(addressToCacheSet(addr))) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Data array stall on addr %#x in set %d\n",
                    addr, addressToCacheSet(addr));
            cacheMemoryStats.numDataArrayStalls++;
            return false;
        }
    } else if (res == CacheResourceType_AtomicALUArray) {
        if (atomicALUArray.tryAccess(addr)) return true;
        else {
            DPRINTF(RubyResourceStalls,
                    "Atomic ALU array stall on addr %#x in line address %#x\n",
                    addr, makeLineAddress(addr));
            cacheMemoryStats.numAtomicALUArrayStalls++;
            return false;
        }
    } else {
        panic("Unrecognized cache resource type.");
    }
}

bool
CacheMemory::isBlockInvalid(int64_t cache_set, int64_t loc)
{
  return (m_cache[cache_set][loc]->m_Permission == AccessPermission_Invalid);
}

bool
CacheMemory::isBlockNotBusy(int64_t cache_set, int64_t loc)
{
  return (m_cache[cache_set][loc]->m_Permission != AccessPermission_Busy);
}

/* hardware transactional memory */

void
CacheMemory::htmAbortTransaction()
{
    uint64_t htmReadSetSize = 0;
    uint64_t htmWriteSetSize = 0;

    // iterate through every set and way to get a cache line
    for (auto i = m_cache.begin(); i != m_cache.end(); ++i)
    {
        std::vector<AbstractCacheEntry*> set = *i;

        for (auto j = set.begin(); j != set.end(); ++j)
        {
            AbstractCacheEntry *line = *j;

            if (line != nullptr) {
                htmReadSetSize += (line->getInHtmReadSet() ? 1 : 0);
                htmWriteSetSize += (line->getInHtmWriteSet() ? 1 : 0);
                if (line->getInHtmWriteSet()) {
                    line->invalidateEntry();
                }
                line->setInHtmWriteSet(false);
                line->setInHtmReadSet(false);
                line->clearLocked();
            }
        }
    }

    cacheMemoryStats.htmTransAbortReadSet.sample(htmReadSetSize);
    cacheMemoryStats.htmTransAbortWriteSet.sample(htmWriteSetSize);
    DPRINTF(HtmMem, "htmAbortTransaction: read set=%u write set=%u\n",
        htmReadSetSize, htmWriteSetSize);
}

void
CacheMemory::htmCommitTransaction()
{
    uint64_t htmReadSetSize = 0;
    uint64_t htmWriteSetSize = 0;

    // iterate through every set and way to get a cache line
    for (auto i = m_cache.begin(); i != m_cache.end(); ++i)
    {
        std::vector<AbstractCacheEntry*> set = *i;

        for (auto j = set.begin(); j != set.end(); ++j)
        {
            AbstractCacheEntry *line = *j;
            if (line != nullptr) {
                htmReadSetSize += (line->getInHtmReadSet() ? 1 : 0);
                htmWriteSetSize += (line->getInHtmWriteSet() ? 1 : 0);
                line->setInHtmWriteSet(false);
                line->setInHtmReadSet(false);
                line->clearLocked();
             }
        }
    }

    cacheMemoryStats.htmTransCommitReadSet.sample(htmReadSetSize);
    cacheMemoryStats.htmTransCommitWriteSet.sample(htmWriteSetSize);
    DPRINTF(HtmMem, "htmCommitTransaction: read set=%u write set=%u\n",
        htmReadSetSize, htmWriteSetSize);
}

void
CacheMemory::profileDemandHit()
{
    cacheMemoryStats.m_demand_hits++;
}

void
CacheMemory::profileDemandMiss()
{
    cacheMemoryStats.m_demand_misses++;
}

void
CacheMemory::profilePrefetchHit()
{
    cacheMemoryStats.m_prefetch_hits++;
}

void
CacheMemory::profilePrefetchMiss()
{
    cacheMemoryStats.m_prefetch_misses++;
}

void
CacheMemory::profileEvictions()
{
    cacheMemoryStats.m_evictions++;
}

void
CacheMemory::profileBIDirty()
{
    cacheMemoryStats.m_evict_dir_evict_dirty++;
}

void
CacheMemory::profileEvictionsType(int type)
{
    if (type == 1) {
        cacheMemoryStats.m_evict_putx++;
    } else if (type == 2) {
        cacheMemoryStats.m_evict_dir_evict++;
    } else if (type == 3) {
        cacheMemoryStats.m_evict_share++;
    } else {
        assert(0);
    }
}

void
CacheMemory::profileRemainedEntry(Addr address)
{
    assert(address == makeLineAddress(address));
    int64_t cacheSet = addressToCacheSet(address);
    assert(m_set_pending_addr[cacheSet].size() <= m_cache_assoc);

    int set_unused_line = 0;
    for (int i = 0; i < m_cache_assoc; i++) {
        AbstractCacheEntry* entry = m_cache[cacheSet][i];
        if (entry == NULL) {
            set_unused_line ++;
        } else if (entry->m_Permission == AccessPermission_NotPresent ||
                   entry->m_waitEvict) {
            set_unused_line ++;
        }
    }
    assert(set_unused_line <= m_cache_assoc);
    
    if (set_unused_line <= m_set_pending_addr[cacheSet].size()) {
        cacheMemoryStats.m_remained_entry_0 ++;
    } else if ((set_unused_line - m_set_pending_addr[cacheSet].size()) == 1) {
        cacheMemoryStats.m_remained_entry_1 ++;
    } else if ((set_unused_line - m_set_pending_addr[cacheSet].size()) == 2) {
        cacheMemoryStats.m_remained_entry_2 ++;
    } else if ((set_unused_line - m_set_pending_addr[cacheSet].size()) > 8) {
        cacheMemoryStats.m_remained_entry_8_ ++;
    } else {
        cacheMemoryStats.m_remained_entry_3_8 ++;
    }
}

void
CacheMemory::profileSharersNum(int num) {
    assert(num > 0);
    if (num == 1) {
        cacheMemoryStats.m_sharer_num_1 ++;
    } else if (num == 2) {
        cacheMemoryStats.m_sharer_num_2 ++;
    } else if (num == 3) {
        cacheMemoryStats.m_sharer_num_3 ++;
    } else if (num == 4) {
        cacheMemoryStats.m_sharer_num_4 ++;
    } else {
        cacheMemoryStats.m_sharer_num_4_ ++;
    }
}

} // namespace ruby
} // namespace gem5
