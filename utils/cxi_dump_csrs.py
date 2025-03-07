#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause
# Copyright 2022 Hewlett Packard Enterprise Development LP

""" Generate an output in C to read most CSRs from a running Cassini
    into a predefined array. To be included by cxi_dump_csrs.c.
"""

import json

try:
    with open('/usr/share/cassini-headers/csr_defs.json', 'r') as f:
        csrdefs = json.load(f)
except FileNotFoundError:
    with open('../cassini-headers/install/share/cassini-headers/csr_defs.json', 'r') as f:
        csrdefs = json.load(f)

print("/* Autogenerated file. Do not modify. */")

# It would be nicer to use an interval tree to store the addresses /
# lengths, but there is no standard implementation, and SLES doesn't
# carry any third party package for it.

for csr, values in csrdefs.items():
    # Remove CSRs that must not be read, or irrelevant ones.
    if "_dbg_" in csr:
        continue

    # Both are aliases to CQ_DBG_TRIG_CMD_MS
    if "trig_cmd_ms" in csr or "dma_amo_payload" in csr:
        continue

    # PCT RAM that cannot be initialized by software, and not read if
    # not used yet by hardware.
    if "pct_cfg_srb_cell_data" in csr:
        continue

    if "pct_cfg_req_redo_ram" in csr:
        continue

    # This one causes read errors. Same case as above?
    if "pct_cfg_srb_link_list" in csr:
        continue

    # Do not read when traffic occurs. CAS-2918
    if "pct_cfg_sct_cam" in csr:
        continue

    # As indicated in the CSDG, that one generates error if not
    # initialized first.
    if "cq_sts_ll_status" in csr:
        continue

    # C_MB_MSC_FLASH_WINDOW0/1 generate read errors at various
    # offsets. They are not needed anyway.
    if "mb_msc_flash_window" in csr:
        continue

    # Do not read MSI-X table
    if "pi_ipd_cfg_vf_msix_table" in csr:
        continue

    if "pi_ipd_cfg_vf_msix_pba" in csr:
        continue

    if csr.startswith("nicsim"):
        continue

    offset = values['__offset']
    length = values['__size'] * values['__entries']

    if csr.startswith('c1_'):
        print("if (is_c1)", end =" ")
    elif csr.startswith(('c2_', 'ss2_')):
        print("if (is_c2)", end =" ")

    # Print addresses instead of CSR names to avoid including the
    # regular Cassini headers
    print(f"cxil_read_csr(dev, {offset}, &buf[{offset}], {length});")
