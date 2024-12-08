# The gem5 Simulator

This is the repository for the gem5 simulator. It contains the full source code
for the simulator and all tests and regressions.

The gem5 simulator is a modular platform for computer-system architecture
research, encompassing system-level architecture as well as processor
microarchitecture. It is primarily used to evaluate new hardware designs,
system software changes, and compile-time and run-time system optimizations.

The main website can be found at <http://www.gem5.org>.

## Testing status

**Note**: These regard tests run on the develop branch of gem5:
<https://github.com/gem5/gem5/tree/develop>.

[![Daily Tests](https://github.com/gem5/gem5/actions/workflows/daily-tests.yaml/badge.svg?branch=develop)](https://github.com/gem5/gem5/actions/workflows/daily-tests.yaml)
[![Weekly Tests](https://github.com/gem5/gem5/actions/workflows/weekly-tests.yaml/badge.svg?branch=develop)](https://github.com/gem5/gem5/actions/workflows/weekly-tests.yaml)
[![Compiler Tests](https://github.com/gem5/gem5/actions/workflows/compiler-tests.yaml/badge.svg?branch=develop)](https://github.com/gem5/gem5/actions/workflows/compiler-tests.yaml)

## Getting started

A good starting point is <http://www.gem5.org/about>, and for
more information about building the simulator and getting started
please see <http://www.gem5.org/documentation> and
<http://www.gem5.org/documentation/learning_gem5/introduction>.

## Building gem5

To build gem5, you will need the following software: g++ or clang,
Python (gem5 links in the Python interpreter), SCons, zlib, m4, and lastly
protobuf if you want trace capture and playback support. Please see
<http://www.gem5.org/documentation/general_docs/building> for more details
concerning the minimum versions of these tools.

Once you have all dependencies resolved, execute
`scons build/ALL/gem5.opt` to build an optimized version of the gem5 binary
(`gem5.opt`) containing all gem5 ISAs. If you only wish to compile gem5 to
include a single ISA, you can replace `ALL` with the name of the ISA. Valid
options include `ARM`, `NULL`, `MIPS`, `POWER`, `RISCV`, `SPARC`, and `X86`
The complete list of options can be found in the build_opts directory.

See https://www.gem5.org/documentation/general_docs/building for more
information on building gem5.

## The Source Tree

The main source tree includes these subdirectories:

* build_opts: pre-made default configurations for gem5
* build_tools: tools used internally by gem5's build process.
* configs: example simulation configuration scripts
* ext: less-common external packages needed to build gem5
* include: include files for use in other programs
* site_scons: modular components of the build system
* src: source code of the gem5 simulator. The C++ source, Python wrappers, and Python standard library are found in this directory.
* system: source for some optional system software for simulated systems
* tests: regression tests
* util: useful utility programs and files

## gem5 Resources

To run full-system simulations, you may need compiled system firmware, kernel
binaries and one or more disk images, depending on gem5's configuration and
what type of workload you're trying to run. Many of these resources can be
obtained from <https://resources.gem5.org>.

More information on gem5 Resources can be found at
<https://www.gem5.org/documentation/general_docs/gem5_resources/>.

## Getting Help, Reporting bugs, and Requesting Features

We provide a variety of channels for users and developers to get help, report
bugs, requests features, or engage in community discussions. Below
are a few of the most common we recommend using.

* **GitHub Discussions**: A GitHub Discussions page. This can be used to start
discussions or ask questions. Available at
<https://github.com/orgs/gem5/discussions>.
* **GitHub Issues**: A GitHub Issues page for reporting bugs or requesting
features. Available at <https://github.com/gem5/gem5/issues>.
* **Jira Issue Tracker**: A Jira Issue Tracker for reporting bugs or requesting
features. Available at <https://gem5.atlassian.net/>.
* **Slack**: A Slack server with a variety of channels for the gem5 community
to engage in a variety of discussions. Please visit
<https://www.gem5.org/join-slack> to join.
* **gem5-users@gem5.org**: A mailing list for users of gem5 to ask questions
or start discussions. To join the mailing list please visit
<https://www.gem5.org/mailing_lists>.
* **gem5-dev@gem5.org**: A mailing list for developers of gem5 to ask questions
or start discussions. To join the mailing list please visit
<https://www.gem5.org/mailing_lists>.

## Work on DTD project

* Download
0. cd /your/gem5/directory/
1. git clone http://gitlab.hkust-gz.edu.cn/bsag/gem5.git
2. cd gem5
3. git branch -a (watch for die-to-die branch)
4. git checkout remotes/origin/die-to-die

* Commit
0. make sure you are in branch die-to-die
0.1. git stash (keep your revise)
0.2. git pull origin die-to-die:die-to-die
0.3. git stash pop
1. git status (get what you have revised and which you want to commit)
2. git add [file1]..[fileN]
3. git commit -m "[XXX]: describe this commit, like bug fix, add ...function"
   XXX is the module name, you can choose from [DTD Slicc] [DTD interface] [Cache Slicc] [Script] [Config] ...
   !!!!Please follow the naming rules for easier co-work 
4. git push -u origin HEAD:die-to-die

* **Docker**:
docker run -it --cap-add=SYS_PTRACE --security-opt seccomp=unconfined -v $(pwd):/mnt --rm --name Gem5Jfeng gem5_jfeng

* **Build**:
CHI:
scons build/X86_CHI/gem5.opt -j32

MESI_Three_level
scons build/X86_MESI_Three_Level/gem5.opt -j32

* **Run**:
* Syth test:
build/X86_CHI/gem5.opt -d traffic_gen/ configs/example/garnet_synth_traffic.py -n 1 --l2cache --num-l2caches 1 --num-l3caches 1 --l1i_size 64kB --l1i_assoc 4 --l1d_size 64kB --l1d_assoc 4 --l2_size 1MB --l2_assoc 8 --l3_size 1MB --l3_assoc 8 --num-dirs 1 --ruby --topology=CustomMesh --chi-config=configs/example/noc_config/2x4.py --network=garnet --synthetic uniform_random --num-packets-max 5 --inj-vnet 0 -i 0.01

* bw mem:
build/X86_CHI/gem5.opt -d bwmem_chi configs/deprecated/example/se.py -n 4 --num-l2caches 4 --num-l3caches 4 --l1i_size 32kB --l1i_assoc 4 --l1d_size 32kB --l1d_assoc 4 --l2_size 512kB --l2_assoc 8 --l3_size 1MB --l3_assoc 8 --num-dirs 2 --ruby --topology=CustomMesh --chi-config=configs/example/noc_config/2x4.py --network=garnet -m 9333752530000 --xor-low-bit 60 --cpu-type O3CPU --cmd 'tests/test-progs/bw_memrdwr/src/threads' --options='1K 4 1 rd'

build/X86_MESI_Three_Level/gem5.opt -d bwmem_mesi configs/deprecated/example/se.py --num-cpus=4 --l0i_size=32kB --l0i_assoc=4 --l0d_size=32kB --l0d_assoc=4 --cpu-type O3CPU --caches --l2cache --ruby --num-dirs=4 --num-l2caches=4 --l1d_size=512kB --l1d_assoc=8 --l2_size=1MB --l2_assoc=8 --network=garnet --topology=MeshDirCorners_XY --mesh-rows=2 --cmd 'tests/test-progs/bw_memrdwr/src/threads' --options='1K 4 1 rd'
