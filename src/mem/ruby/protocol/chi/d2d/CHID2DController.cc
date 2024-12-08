/*
 * Copyright (c) 2023 ARM Limited
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

#include "mem/ruby/protocol/chi/d2d/CHID2DController.hh"

#include <sys/types.h>
#include <unistd.h>

#include <cassert>
#include <sstream>
#include <string>
#include <typeinfo>

#include "debug/RubyCHID2D.hh"
#include "mem/ruby/network/Network.hh"
#include "mem/ruby/protocol/MemoryMsg.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/system/Sequencer.hh"

namespace gem5
{

namespace ruby
{

// CHID2DController appears as a Cache_Controller to the rest of CHI
// TODO should this be configurable ?
int
CHID2DController::getNumControllers()
{
    return Cache_Controller::getNumControllers();
}

void
CHID2DController::incNumControllers()
{
    Cache_Controller::incNumControllers();
}

CHID2DController::CHID2DController(const Params &p)
  : AbstractController(p),
    reqOut(p.reqOut), snpOut(p.snpOut),
    rspOut(p.rspOut), datOut(p.datOut),
    reqIn(p.reqIn), snpIn(p.snpIn),
    rspIn(p.rspIn), datIn(p.datIn),
    reqD2DOut(p.reqOut), snpD2DOut(p.snpOut),
    rspD2DOut(p.rspOut), datD2DOut(p.datOut),
    reqD2DIn(p.reqIn), snpD2DIn(p.snpIn),
    rspD2DIn(p.rspIn), datD2DIn(p.datIn),
    cacheLineSize(RubySystem::getBlockSizeBytes()),
    dataChannelSize(p.data_channel_size),
    dataMsgsPerLine(RubySystem::getBlockSizeBytes() / p.data_channel_size)
{
    m_machineID.type = MachineType_Cache;
    m_machineID.num = m_version;
    incNumControllers();
    p.ruby_system->registerAbstractController(this);
}

void
CHID2DController::initNetQueues()
{
    int base = MachineType_base_number(m_machineID.type);

    assert(m_net_ptr != nullptr);

    m_net_ptr->setToNetQueue(m_version + base, reqOut->getOrdered(),
                             CHI_REQ, "none", reqOut);
    m_net_ptr->setToNetQueue(m_version + base, snpOut->getOrdered(),
                             CHI_SNP, "none", snpOut);
    m_net_ptr->setToNetQueue(m_version + base, rspOut->getOrdered(),
                             CHI_RSP, "none", rspOut);
    m_net_ptr->setToNetQueue(m_version + base, datOut->getOrdered(),
                             CHI_DAT, "response", datOut);

    m_net_ptr->setFromNetQueue(m_version + base, reqIn->getOrdered(),
                               CHI_REQ, "none", reqIn);
    m_net_ptr->setFromNetQueue(m_version + base, snpIn->getOrdered(),
                               CHI_SNP, "none", snpIn);
    m_net_ptr->setFromNetQueue(m_version + base, rspIn->getOrdered(),
                               CHI_RSP, "none", rspIn);
    m_net_ptr->setFromNetQueue(m_version + base, datIn->getOrdered(),
                               CHI_DAT, "response", datIn);

    m_net_ptr->setToNetQueue(m_version + base, reqD2DOut->getOrdered(),
                             CHI_REQ, "none", reqD2DOut);
    m_net_ptr->setToNetQueue(m_version + base, snpD2DOut->getOrdered(),
                             CHI_SNP, "none", snpD2DOut);
    m_net_ptr->setToNetQueue(m_version + base, rspD2DOut->getOrdered(),
                             CHI_RSP, "none", rspD2DOut);
    m_net_ptr->setToNetQueue(m_version + base, datD2DOut->getOrdered(),
                             CHI_DAT, "response", datD2DOut);

    m_net_ptr->setFromNetQueue(m_version + base, reqD2DIn->getOrdered(),
                               CHI_REQ, "none", reqD2DIn);
    m_net_ptr->setFromNetQueue(m_version + base, snpD2DIn->getOrdered(),
                               CHI_SNP, "none", snpD2DIn);
    m_net_ptr->setFromNetQueue(m_version + base, rspD2DIn->getOrdered(),
                               CHI_RSP, "none", rspD2DIn);
    m_net_ptr->setFromNetQueue(m_version + base, datD2DIn->getOrdered(),
                               CHI_DAT, "response", datD2DIn);
}

void
CHID2DController::init()
{
    AbstractController::init();

    rspIn->setConsumer(this);
    datIn->setConsumer(this);
    snpIn->setConsumer(this);
    reqIn->setConsumer(this);
    rspD2DIn->setConsumer(this);
    datD2DIn->setConsumer(this);
    snpD2DIn->setConsumer(this);
    reqD2DIn->setConsumer(this);

    resetStats();
}

void
CHID2DController::addSequencer(RubyPort *seq)
{
    assert(seq != nullptr);
    sequencers.emplace_back(seq);
}

void
CHID2DController::print(std::ostream& out) const
{
    out << "[CHID2DController " << m_version << "]";
}

Sequencer*
CHID2DController::getCPUSequencer() const
{
    // CHID2DController doesn't have a CPUSequencer
    return nullptr;
}

DMASequencer*
CHID2DController::getDMASequencer() const
{
    // CHID2DController doesn't have a DMASequencer
    return nullptr;
}

GPUCoalescer*
CHID2DController::getGPUCoalescer() const
{
    // CHID2DController doesn't have a GPUCoalescer
    return nullptr;
}

MessageBuffer*
CHID2DController::getMandatoryQueue() const
{
    // CHID2DController doesn't have a MandatoryQueue
    return nullptr;
}

MessageBuffer*
CHID2DController::getMemReqQueue() const
{
    // CHID2DController doesn't have a MemReqQueue
    return nullptr;
}

MessageBuffer*
CHID2DController::getMemRespQueue() const
{
    // CHID2DController doesn't have a MemRespQueue
    return nullptr;
}

void
CHID2DController::regStats()
{
    AbstractController::regStats();
}

void
CHID2DController::collateStats()
{

}

void
CHID2DController::resetStats()
{
    AbstractController::resetStats();
}


void
CHID2DController::wakeup()
{
    bool pending = false;

    DPRINTF(RubyCHID2D, "wakeup: checking incoming rsp messages\n");
    pending = pending || receiveAllRdyMessages<CHIResponseMsg>(rspIn,
         [this](const CHIResponseMsg* msg){ return recvResponseMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming dat messages\n");
    pending = pending || receiveAllRdyMessages<CHIDataMsg>(datIn,
         [this](const CHIDataMsg* msg){ return recvDataMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming snp messages\n");
    pending = pending || receiveAllRdyMessages<CHIRequestMsg>(snpIn,
         [this](const CHIRequestMsg* msg){ return recvSnoopMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming req messages\n");
    pending = pending || receiveAllRdyMessages<CHIRequestMsg>(reqIn,
         [this](const CHIRequestMsg* msg){ return recvRequestMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming rsp messages\n");
    pending = pending || receiveAllRdyMessages<CHIResponseMsg>(rspD2DIn,
         [this](const CHIResponseMsg* msg){ return recvResponseMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming dat messages\n");
    pending = pending || receiveAllRdyMessages<CHIDataMsg>(datD2DIn,
         [this](const CHIDataMsg* msg){ return recvDataMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming snp messages\n");
    pending = pending || receiveAllRdyMessages<CHIRequestMsg>(snpD2DIn,
         [this](const CHIRequestMsg* msg){ return recvSnoopMsg(msg); });

    DPRINTF(RubyCHID2D, "wakeup: checking incoming req messages\n");
    pending = pending || receiveAllRdyMessages<CHIRequestMsg>(reqD2DIn,
         [this](const CHIRequestMsg* msg){ return recvRequestMsg(msg); });

    if (pending) {
        DPRINTF(RubyCHID2D, "wakeup: messages pending\n");
        scheduleEvent(Cycles(1));
    }
}

void
CHID2DController::recordCacheTrace(int cntrl, CacheRecorder* tr)
{
    panic("CHID2DController doesn't implement recordCacheTrace");
}

AccessPermission
CHID2DController::getAccessPermission(const Addr& param_addr)
{
    return AccessPermission_NotPresent;
}

void
CHID2DController::functionalRead(
    const Addr& param_addr, Packet* param_pkt, WriteMask& param_mask)
{
    panic("CHID2DController doesn't expect functionalRead");
}

int
CHID2DController::functionalWrite(
    const Addr& param_addr, Packet* param_pkt)
{
    panic("CHID2DController doesn't expect functionalRead");
    return 0;
}

int
CHID2DController::functionalWriteBuffers(PacketPtr& pkt)
{
    int num_functional_writes = 0;
    num_functional_writes += reqOut->functionalWrite(pkt);
    num_functional_writes += snpOut->functionalWrite(pkt);
    num_functional_writes += rspOut->functionalWrite(pkt);
    num_functional_writes += datOut->functionalWrite(pkt);
    num_functional_writes += reqIn->functionalWrite(pkt);
    num_functional_writes += snpIn->functionalWrite(pkt);
    num_functional_writes += rspIn->functionalWrite(pkt);
    num_functional_writes += datIn->functionalWrite(pkt);

    num_functional_writes += reqD2DOut->functionalWrite(pkt);
    num_functional_writes += snpD2DOut->functionalWrite(pkt);
    num_functional_writes += rspD2DOut->functionalWrite(pkt);
    num_functional_writes += datD2DOut->functionalWrite(pkt);
    num_functional_writes += reqD2DIn->functionalWrite(pkt);
    num_functional_writes += snpD2DIn->functionalWrite(pkt);
    num_functional_writes += rspD2DIn->functionalWrite(pkt);
    num_functional_writes += datD2DIn->functionalWrite(pkt);
    return num_functional_writes;
}

bool
CHID2DController::functionalReadBuffers(PacketPtr& pkt)
{
    if (reqOut->functionalRead(pkt)) return true;
    if (snpOut->functionalRead(pkt)) return true;
    if (rspOut->functionalRead(pkt)) return true;
    if (datOut->functionalRead(pkt)) return true;
    if (reqIn->functionalRead(pkt)) return true;
    if (snpIn->functionalRead(pkt)) return true;
    if (rspIn->functionalRead(pkt)) return true;
    if (datIn->functionalRead(pkt)) return true;

    if (reqD2DOut->functionalRead(pkt)) return true;
    if (snpD2DOut->functionalRead(pkt)) return true;
    if (rspD2DOut->functionalRead(pkt)) return true;
    if (datD2DOut->functionalRead(pkt)) return true;
    if (reqD2DIn->functionalRead(pkt)) return true;
    if (snpD2DIn->functionalRead(pkt)) return true;
    if (rspD2DIn->functionalRead(pkt)) return true;
    if (datD2DIn->functionalRead(pkt)) return true;
    return false;
}

bool
CHID2DController::functionalReadBuffers(PacketPtr& pkt, WriteMask &mask)
{
    bool read = false;
    if (reqOut->functionalRead(pkt, mask)) read = true;
    if (snpOut->functionalRead(pkt, mask)) read = true;
    if (rspOut->functionalRead(pkt, mask)) read = true;
    if (datOut->functionalRead(pkt, mask)) read = true;
    if (reqIn->functionalRead(pkt, mask)) read = true;
    if (snpIn->functionalRead(pkt, mask)) read = true;
    if (rspIn->functionalRead(pkt, mask)) read = true;
    if (datIn->functionalRead(pkt, mask)) read = true;

    if (reqD2DOut->functionalRead(pkt, mask)) read = true;
    if (snpD2DOut->functionalRead(pkt, mask)) read = true;
    if (rspD2DOut->functionalRead(pkt, mask)) read = true;
    if (datD2DOut->functionalRead(pkt, mask)) read = true;
    if (reqD2DIn->functionalRead(pkt, mask)) read = true;
    if (snpD2DIn->functionalRead(pkt, mask)) read = true;
    if (rspD2DIn->functionalRead(pkt, mask)) read = true;
    if (datD2DIn->functionalRead(pkt, mask)) read = true;
    return read;
}

} // namespace ruby
} // namespace gem5
