---
title: CXI_SEND_BW(1) Version 2.1.0 | CXI Diagnostics and Utilities
date: 2023-08-19
---

# NAME

cxi_send_bw - Cassini NIC send bandwidth benchmark using LibCXI


# SYNOPSIS

| **cxi_send_bw** [**-d** *DEV*] [**-p** *PORT*]
| **cxi_send_bw** [*OPTIONS*] *SERVER_ADDR*


# DESCRIPTION

**cxi_send_bw** is a simple benchmark for measuring two-sided send bandwidth. It
can be configured to run a number of iterations or to run for a duration of
time. Bandwidth is measured for a specific size of sends, or for each size
within a range. Either bi-directional or uni-directional bandwidth can be
measured. Users have the option to allocate source and destination buffers from
system memory or from an available GPU device via calls to the HIP, CUDA, or
INTEL Level-zero APIs.

## How to run

Two copies of the program must be launched. The first copy is the server. It
accepts optional arguments to specify which CXI device to use, which port to
listen on for the client connection, and whether or not GPU buffers should be
used. The second copy is the client. It must be supplied with the hostname or
IPv4 address of the server, along with the port to connect to if the default
value was not used. The client must also be supplied with all desired run
options.

## Output

Both the server and client print output, much of which is the same between
the two. Once a connection has been established, a summary of the specified
options is printed along with the server and client NIC fabric addresses.
After the benchmark runs (or after each run for a range of sizes), four
columns of data are printed.

*Send Size[B]*
: The size of the sends.

*Sends*
: The number of sends performed. When measuring bi-directional bandwidth, the
server and client only report the number of sends they initiate, not the
combined count.

*BW[MB/s]*
: The measured bandwidth in megabytes per second. When measuring bi-directional
bandwidth, the server and client bandwidths are combined and reported by both
sides.

*PktRate[Mpkt/s]*
: The measured packet rate in millions of packets per second. When measuring
bi-directional bandwidth, the server and client packet rates are combined and
reported by both sides. Sends larger than the Portals MTU (2048 bytes) are
split into multiple packets.


# OPTIONS

**-d, \-\-device**=*DEV*
: The Cassini NIC device name to use. When unspecified, \"cxi0\" is used.

**-v, \-\-svc-id**=*SVC_ID*
: The service ID to use. This option will be used when running as a
non-privileged user. When unspecified the default service ID is used.

**-p, \-\-port**=*PORT*
: The TCP port number to use for the client/server connection. When
unspecified, port 49194 is used.

**-t, \-\-tx-gpu**
: The GPU to use for TX buffer allocation. By default system memory is used.

**-r, \-\-rx-gpu**
: The GPU to use for RX buffer allocation. By default system memory is used.

**-g, \-\-gpu-type**
: The GPU type to use with this test. Valid values are AMD, NVIDIA, or INTEL. By default
the type is determined by discovered GPU files on the system.

**-n, \-\-iters**=*ITERS*
: The number of iterations to perform. A single iteration consists of sending
*LIST_SIZE* sends and then waiting for each to be acknowledged. When a range
of send sizes is specified, *ITERS* iterations are performed for each size.
When *ITERS* and *DURATION* are both unspecified, 1000 iterations are
performed.

**-D, \-\-duration**=*DURATION*
: This option configures the benchmark to continue running iterations until
*DURATION* seconds has elapsed. When a range of send sizes is specified,
the benchmark runs for *DURATION* seconds for each size.

**-s, \-\-size**=*MIN*[:*MAX*]
: The send size, or range of sizes. Valid send sizes are 1 through 4294697295
bytes (4GiB - 1). When a range is specified, the benchmark runs once for each
size that is a power of two from *MIN* to *MAX*. By default the send size is
65536 bytes.

**-l, \-\-list-size**=*LIST_SIZE*
: The number of sends performed per iteration. The send commands are all
added to the TX command queue before ringing the queue's doorbell to trigger
the sends. By default the list size is 256.

**-b, \-\-bidirectional**
: This option configures the benchmark to measure bi-directional bandwidth. The
client and server calculate their individual rates, share these with each
other, and then both report the combined rates. By default the benchmark
performs sends from the client to the server only. In this case, the client
rates are still shared and reported from both client and server. Running
bi-directionally works best when combined with **\-\-duration** rather than
**\-\-iters**.

**\-\-no-idc**
: Disable the use of Immediate Data Commands. By default IDCs are used for
sizes up to 192 bytes. IDCs are disabled when TX GPU buffers are used.

**-R, \-\-rdzv**
: Use rendezvous transfers.

**\-\-buf-sz**=*BUF_SIZE*
: The maximum allowed data buffer size for both the transmit and target
buffers, specified in bytes. The benchmark may allocate less space if it is
possible to fit all *LIST_SIZE* sends of a single iteration, without overlap,
aligned to *BUF_ALIGN* boundaries. If *BUF_SIZE* is not a multiple of the page
size, it is rounded up to the nearest page boundary. However, send offsets
within the buffers are still chosen as if the buffer is only *BUF_SIZE* bytes.
If the sends for a single iteration do not fit without overlap, their offsets
are wrapped around.

**\-\-buf-align**=*BUF_ALIGN*
: The alignment of sends in the transmit and target buffers. By default this
is 64 bytes.

**\-\-use-hp**=*HP_SIZE*
: The size of huge pages to use when allocating the transmit and target buffers.
This option has no effect when GPU buffers are used. Valid values are 2M and 1G.
By default huge pages are disabled.

**-h, \-\-help**
: Display the help text and exit.

**-V, \-\-version**
: Display the program version and exit.


# EXAMPLES

## Example 1

Running with the default options

*Server*
```
$ cxi_send_bw
Listening on port 49194 for client to connect...
---------------------------------------------------
    CXI RDMA Send Bandwidth Test
Device           : cxi0
Service ID       : 1
Client TX Mem    : System
Server RX Mem    : System
Test Type        : Iteration
Iterations       : 1000
Send Size        : 65536
List Size        : 256
IDC              : Enabled
Bidirectional    : Disabled
Max RDMA Buf     : 4294967296 (16777216 used)
RDMA Buf Align   : 64
Hugepages        : Disabled
Local (server)   : NIC 0x12 PID 0 VNI 10
Remote (client)  : NIC 0x13 PID 0
---------------------------------------------------
Send Size[B]       Sends  BW[MB/s]  PktRate[Mpkt/s]
       65536           -  23772.52        11.607674
---------------------------------------------------
```

*Client*
```
$ cxi_send_bw 192.168.1.1
---------------------------------------------------
    CXI RDMA Send Bandwidth Test
Device           : cxi0
Service ID       : 1
Client TX Mem    : System
Server RX Mem    : System
Test Type        : Iteration
Iterations       : 1000
Send Size        : 65536
List Size        : 256
IDC              : Enabled
Bidirectional    : Disabled
Max RDMA Buf     : 4294967296 (16777216 used)
RDMA Buf Align   : 64
Hugepages        : Disabled
Local (client)   : NIC 0x13 PID 0 VNI 10
Remote (server)  : NIC 0x12 PID 0
---------------------------------------------------
Send Size[B]       Sends  BW[MB/s]  PktRate[Mpkt/s]
       65536      256000  23772.52        11.607674
---------------------------------------------------
```

## Example 2

Running bi-directionally over a range of sizes for 5 seconds each

*Server*
```
$ cxi_send_bw
Listening on port 49194 for client to connect...
---------------------------------------------------
    CXI RDMA Send Bandwidth Test
Device           : cxi0
Service ID       : 1
Client TX Mem    : System
Client RX Mem    : System
Server TX Mem    : System
Server RX Mem    : System
Test Type        : Duration
Duration         : 5 seconds
Min Send Size    : 1024
Max Send Size    : 65536
List Size        : 256
IDC              : Enabled
Bidirectional    : Enabled
Max RDMA Buf     : 4294967296 (16777216 used)
RDMA Buf Align   : 64
Hugepages        : Disabled
Local (server)   : NIC 0x12 PID 0 VNI 10
Remote (client)  : NIC 0x13 PID 0
---------------------------------------------------
Send Size[B]       Sends  BW[MB/s]  PktRate[Mpkt/s]
        1024    49486592  20269.48        19.794417
        2048    32482048  26609.00        12.992678
        4096    17769984  29113.95        14.215795
        8192    11196160  36687.14        17.913645
       16384     6153216  40324.60        19.689748
       32768     3227904  42307.73        20.658069
       65536     1631488  42765.29        20.881491
---------------------------------------------------
```

*Client*
```
$ cxi_send_bw 192.168.1.1 -b -D 5 -s 1024:65536
---------------------------------------------------
    CXI RDMA Send Bandwidth Test
Device           : cxi0
Service ID       : 1
Client TX Mem    : System
Client RX Mem    : System
Server TX Mem    : System
Server RX Mem    : System
Test Type        : Duration
Duration         : 5 seconds
Min Send Size    : 1024
Max Send Size    : 65536
List Size        : 256
IDC              : Enabled
Bidirectional    : Enabled
Max RDMA Buf     : 4294967296 (16777216 used)
RDMA Buf Align   : 64
Hugepages        : Disabled
Local (client)   : NIC 0x13 PID 0 VNI 10
Remote (server)  : NIC 0x12 PID 0
---------------------------------------------------
Send Size[B]       Sends  BW[MB/s]  PktRate[Mpkt/s]
        1024    49486592  20269.48        19.794417
        2048    32482048  26609.00        12.992678
        4096    17769984  29113.95        14.215795
        8192    11196160  36687.14        17.913645
       16384     6153216  40324.60        19.689748
       32768     3227904  42307.73        20.658069
       65536     1631488  42765.29        20.881491
---------------------------------------------------
```


# SEE ALSO

**cxi_diags**(7)
