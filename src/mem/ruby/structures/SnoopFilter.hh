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

#ifndef __MEM_RUBY_STRUCTURES_DIRECTORYCHIMEMORY_HH__
#define __MEM_RUBY_STRUCTURES_DIRECTORYCHIMEMORY_HH__

#include <iostream>
#include <string>

#include "mem/ruby/common/Address.hh"
#include "mem/ruby/protocol/DirectoryRequestType.hh"
#include "mem/ruby/slicc_interface/AbstractCacheEntry.hh"
#include "mem/ruby/slicc_interface/RubySlicc_ComponentMapping.hh"
#include "params/RubySnoopFilter.hh"
#include "sim/sim_object.hh"

namespace gem5
{

namespace ruby
{

class SnoopFilter : public SimObject
{
  public:
    typedef RubySnoopFilterParams Params;
    SnoopFilter(const Params &p);
    ~SnoopFilter();

    void init();

    void print(std::ostream& out) const;

    AbstractCacheEntry* lookup(Addr addr, bool need_update_access_time);
    const AbstractCacheEntry* lookup(Addr addr) const;
    bool allocate(Addr addr, AbstractCacheEntry * new_entry);
    bool isPresent(Addr addr);
    void deallocate(Addr addr);
    Addr cacheProbe(Addr addr);
    void makeComplete(Addr addr);
    void makeInComplete(Addr addr);

  private:
    // Private copy constructor and assignment operator
    SnoopFilter(const SnoopFilter& obj);
    SnoopFilter& operator=(const SnoopFilter& obj);

    int64_t addressToCacheSet(Addr address) const;
    int findTagInSet(int64_t dirSet, Addr tag) const;
    // Returns true if there is:
    //   a) a tag match on this address or there is
    //   b) an unused line in the same cache "way"
    bool setAvail(Addr address) const;

  private:

    const uint64_t m_dir_bit_num;
    const uint64_t m_dir_ass_num;
    uint64_t m_dir_num_set;
    uint64_t m_dir_num_way;

    std::vector<std::vector<AbstractCacheEntry*> > m_dir;
    std::unordered_map<Addr, int> m_tag_index;
};

inline std::ostream&
operator<<(std::ostream& out, const SnoopFilter& obj)
{
    obj.print(out);
    out << std::flush;
    return out;
}

} // namespace ruby
} // namespace gem5

#endif // __MEM_RUBY_STRUCTURES_DIRECTORYCHIMEMORY_HH__
