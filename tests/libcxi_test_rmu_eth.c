/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause */
/* Copyright 2026 Hewlett Packard Enterprise Development LP */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include "libcxi_test_common.h"

#define RMU_ETH_FILTERS 4
#define RMU_ETH_INDIR_ENTRIES 8
#define RMU_ETH_MAC 0x001122334455ULL
#define RMU_ETH_HASH_TYPES (1U << (C_RSS_HASH_IPV4_TCP - 1))

static struct cxil_rmu_eth *rmu_eth;

static const struct cxil_rmu_eth_opts rmu_eth_opts = {
	.filter_entries = RMU_ETH_FILTERS,
	.rss_indir_entries = RMU_ETH_INDIR_ENTRIES,
};

static void rmu_eth_dev_setup(void)
{
	pte_setup();
}

static void rmu_eth_setup(void)
{
	int rc;

	rmu_eth_dev_setup();

	rc = cxil_alloc_rmu_eth(dev, &rmu_eth_opts, &rmu_eth);
	cr_assert_eq(rc, 0, "cxil_alloc_rmu_eth() returns (%d) %s",
		     rc, strerror(-rc));
	cr_assert_neq(rmu_eth, NULL);
	cr_assert_gt(rmu_eth->max_filters, 0);
}

static void rmu_eth_teardown(void)
{
	int rc;

	rc = cxil_destroy_rmu_eth(rmu_eth);
	cr_expect_eq(rc, 0, "%s: cxil_destroy_rmu_eth() returns (%d) %s",
		     __func__, rc, strerror(-rc));
	rmu_eth = NULL;

	pte_teardown();
}

TestSuite(rmu_eth_alloc, .init = rmu_eth_dev_setup, .fini = pte_teardown);

Test(rmu_eth_alloc, null)
{
	struct cxil_rmu_eth *obj = NULL;
	int rc;

	rc = cxil_alloc_rmu_eth(NULL, &rmu_eth_opts, &obj);
	cr_assert_eq(rc, -EINVAL);
	cr_assert_eq(obj, NULL);

	rc = cxil_alloc_rmu_eth(dev, NULL, &obj);
	cr_assert_eq(rc, -EINVAL);
	cr_assert_eq(obj, NULL);

	rc = cxil_alloc_rmu_eth(dev, &rmu_eth_opts, NULL);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_destroy_rmu_eth(NULL);
	cr_assert_eq(rc, -EINVAL);
}

Test(rmu_eth_alloc, basic)
{
	struct cxil_rmu_eth *obj = NULL;
	int rc;

	rc = cxil_alloc_rmu_eth(dev, &rmu_eth_opts, &obj);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
	cr_assert_neq(obj, NULL);

	cr_expect_gt(obj->max_filters, 0);
	cr_expect_leq(obj->max_filters, RMU_ETH_FILTERS);

	/* The driver rounds the indirection table down to a power of two. */
	cr_expect_leq(obj->max_indir_entries, RMU_ETH_INDIR_ENTRIES);

	rc = cxil_destroy_rmu_eth(obj);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
}

Test(rmu_eth_alloc, no_rss)
{
	const struct cxil_rmu_eth_opts opts = {
		.filter_entries = 1,
		.rss_indir_entries = 0,
	};
	struct cxil_rmu_eth *obj = NULL;
	int rc;

	rc = cxil_alloc_rmu_eth(dev, &opts, &obj);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
	cr_expect_eq(obj->max_filters, 1);
	cr_expect_eq(obj->max_indir_entries, 0);

	rc = cxil_destroy_rmu_eth(obj);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
}

Test(rmu_eth_alloc, invalid_opts)
{
	struct cxil_rmu_eth *obj = NULL;
	struct cxil_rmu_eth_opts opts = rmu_eth_opts;
	int expected_rc;
	int rc;

	opts.filter_entries = 0;
	rc = cxil_alloc_rmu_eth(dev, &opts, &obj);
	expected_rc = dev->info.is_vf ? -ENOSPC : -EINVAL;
	cr_assert_eq(rc, expected_rc);
	cr_assert_eq(obj, NULL);

	opts = rmu_eth_opts;
	opts.rss_indir_entries = CXI_ETH_MAX_INDIR_ENTRIES + 1;
	rc = cxil_alloc_rmu_eth(dev, &opts, &obj);
	cr_assert_eq(rc, -EINVAL);
	cr_assert_eq(obj, NULL);
}

TestSuite(rmu_eth_filter, .init = rmu_eth_setup, .fini = rmu_eth_teardown);

Test(rmu_eth_filter, null)
{
	uint8_t table[2] = {};
	struct cxil_pte *ptes[2] = { rx_pte, rx_pte };
	int rc;

	rc = cxil_rmu_eth_add_mac_filter(NULL, RMU_ETH_MAC, rx_pte, false);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC, NULL, false);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_add_all_mcast_filter(NULL, rx_pte, false);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_add_all_mcast_filter(rmu_eth, NULL, false);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_add_promiscuous_filter(NULL, rx_pte, false);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_add_promiscuous_filter(rmu_eth, NULL, false);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_remove_mac_filter(NULL, RMU_ETH_MAC);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_remove_all_mcast_filter(NULL);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_remove_promiscuous_filter(NULL);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_set_rss_queues(NULL, 2, ptes, RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, 2, NULL, RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_set_indir_table(NULL, table, ARRAY_SIZE(table));
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_set_indir_table(rmu_eth, NULL, ARRAY_SIZE(table));
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_set_indir_table(rmu_eth, table, 0);
	cr_assert_eq(rc, -EINVAL);
}

Test(rmu_eth_filter, mac_filter)
{
	int rc;

	rc = cxil_rmu_eth_remove_mac_filter(rmu_eth, RMU_ETH_MAC);
	cr_assert_eq(rc, -ENOENT);

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC, rx_pte, false);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* Re-adding the same address updates the existing slot. */
	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC, rx_pte, false);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* The PtlTE is referenced by the filter and cannot be freed. */
	rc = cxil_destroy_pte(rx_pte);
	cr_assert_eq(rc, -EBUSY);

	rc = cxil_rmu_eth_remove_mac_filter(rmu_eth, RMU_ETH_MAC);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_rmu_eth_remove_mac_filter(rmu_eth, RMU_ETH_MAC);
	cr_assert_eq(rc, -ENOENT);
}

Test(rmu_eth_filter, mac_filter_replaces_pte)
{
	struct cxi_pt_alloc_opts pte_opts = {};
	struct cxil_pte *replacement_pte = NULL;
	int rc;

	rc = cxil_alloc_pte(lni, NULL, &pte_opts, &replacement_pte);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC, rx_pte, false);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC,
					 replacement_pte, false);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* Replacing the filter releases the old PtlTE reference. */
	rc = cxil_destroy_pte(rx_pte);
	cr_assert_eq(rc, 0);
	/* The new filter target remains referenced. */
	rc = cxil_destroy_pte(replacement_pte);
	cr_assert_eq(rc, -EBUSY);

	rc = cxil_rmu_eth_remove_mac_filter(rmu_eth, RMU_ETH_MAC);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_destroy_pte(replacement_pte);
	cr_assert_eq(rc, 0);

	rc = cxil_alloc_pte(lni, NULL, &pte_opts, &rx_pte);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
}

Test(rmu_eth_filter, mac_filter_exhaustion)
{
	unsigned int max_filters = rmu_eth->max_filters;
	unsigned int i;
	int rc;

	for (i = 0; i < max_filters; i++) {
		rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC + i,
						 rx_pte, false);
		cr_assert_eq(rc, 0, "filter %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC + max_filters,
					 rx_pte, false);
	cr_assert_eq(rc, -ENOSPC);

	for (i = 0; i < max_filters; i++) {
		rc = cxil_rmu_eth_remove_mac_filter(rmu_eth, RMU_ETH_MAC + i);
		cr_assert_eq(rc, 0, "filter %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}
}

Test(rmu_eth_filter, all_mcast_filter)
{
	int rc;

	rc = cxil_rmu_eth_remove_all_mcast_filter(rmu_eth);
	cr_assert_eq(rc, -ENOENT);

	rc = cxil_rmu_eth_add_all_mcast_filter(rmu_eth, rx_pte, false);
	cr_assert_eq(rc, -EPERM);

	rc = cxil_rmu_eth_remove_all_mcast_filter(rmu_eth);
	cr_assert_eq(rc, -ENOENT);

	rc = cxil_rmu_eth_remove_all_mcast_filter(rmu_eth);
	cr_assert_eq(rc, -ENOENT);
}

Test(rmu_eth_filter, promiscuous_filter)
{
	int rc;

	rc = cxil_rmu_eth_remove_promiscuous_filter(rmu_eth);
	cr_assert_eq(rc, -ENOENT);

	rc = cxil_rmu_eth_add_promiscuous_filter(rmu_eth, rx_pte, false);
	cr_assert_eq(rc, -EPERM);

	rc = cxil_rmu_eth_remove_promiscuous_filter(rmu_eth);
	cr_assert_eq(rc, -ENOENT);

	rc = cxil_rmu_eth_remove_promiscuous_filter(rmu_eth);
	cr_assert_eq(rc, -ENOENT);
}

/* A filter left behind keeps the PtlTE busy until the object is freed. */
Test(rmu_eth_filter, free_releases_pte)
{
	struct cxi_pt_alloc_opts pte_opts = {};
	int rc;

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC, rx_pte, false);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_destroy_rmu_eth(rmu_eth);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_destroy_pte(rx_pte);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* Restore what the teardown expects to find. */
	rc = cxil_alloc_rmu_eth(dev, &rmu_eth_opts, &rmu_eth);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_alloc_pte(lni, NULL, &pte_opts, &rx_pte);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
}

Test(rmu_eth_filter, rss)
{
	struct cxi_pt_alloc_opts pte_opts = {};
	struct cxil_pte *ptes[4] = {};
	struct cxil_pte *invalid_ptes[2];
	uint8_t table[CXI_ETH_MAX_INDIR_ENTRIES];
	unsigned int indir_size;
	unsigned int i;
	int rc;

	if (rmu_eth->max_indir_entries < 2)
		cr_skip_test("No RSS indirection entries available");

	indir_size = rmu_eth->max_indir_entries;

	for (i = 0; i < ARRAY_SIZE(ptes); i++) {
		rc = cxil_alloc_pte(lni, NULL, &pte_opts, &ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}

	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, 2, ptes,
					 RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* RSS queues hold references on their PtlTEs. */
	rc = cxil_destroy_pte(ptes[0]);
	cr_assert_eq(rc, -EBUSY);

	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, 2, &ptes[2],
					 RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* Replacing RSS queues drops references to the previous PtlTEs. */
	for (i = 0; i < 2; i++) {
		rc = cxil_destroy_pte(ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}
	rc = cxil_destroy_pte(ptes[2]);
	cr_assert_eq(rc, -EBUSY);

	for (i = 0; i < indir_size; i++)
		table[i] = i % 2;

	rc = cxil_rmu_eth_set_indir_table(rmu_eth, table, indir_size);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	/* Size must be a power of two. */
	rc = cxil_rmu_eth_set_indir_table(rmu_eth, table, 3);
	cr_assert_eq(rc, -EINVAL);

	rc = cxil_rmu_eth_set_indir_table(rmu_eth, table,
					  CXI_ETH_MAX_INDIR_ENTRIES + 1);
	cr_assert_eq(rc, -EINVAL);

	/* Entries must index one of the configured RSS queues. */
	table[0] = 2;
	rc = cxil_rmu_eth_set_indir_table(rmu_eth, table, 2);
	cr_assert_eq(rc, -EINVAL);
	table[0] = 0;

	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, CXI_ETH_MAX_RSS_QUEUES + 1,
					 &ptes[2], RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, -EINVAL);

	invalid_ptes[0] = ptes[2];
	invalid_ptes[1] = NULL;
	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, ARRAY_SIZE(invalid_ptes),
					 invalid_ptes, RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, -EINVAL);

	/* Disable RSS and release the PtlTE references. */
	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, 0, NULL, 0);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	for (i = 2; i < ARRAY_SIZE(ptes); i++) {
		rc = cxil_destroy_pte(ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}
}

Test(rmu_eth_filter, rss_without_indirection)
{
	const struct cxil_rmu_eth_opts opts = {
		.filter_entries = 1,
		.rss_indir_entries = 0,
	};
	struct cxi_pt_alloc_opts pte_opts = {};
	struct cxil_pte *ptes[2] = {};
	struct cxil_rmu_eth *obj = NULL;
	unsigned int i;
	int rc;

	rc = cxil_alloc_rmu_eth(dev, &opts, &obj);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));
	cr_assert_eq(obj->max_indir_entries, 0);

	for (i = 0; i < ARRAY_SIZE(ptes); i++) {
		rc = cxil_alloc_pte(lni, NULL, &pte_opts, &ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}

	rc = cxil_rmu_eth_set_rss_queues(obj, ARRAY_SIZE(ptes), ptes,
					 RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, -ENOSPC);

	rc = cxil_destroy_rmu_eth(obj);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	for (i = 0; i < ARRAY_SIZE(ptes); i++) {
		rc = cxil_destroy_pte(ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}
}

Test(rmu_eth_filter, rss_with_mac_filter)
{
	struct cxi_pt_alloc_opts pte_opts = {};
	struct cxil_pte *ptes[2] = {};
	unsigned int i;
	int rc;

	if (rmu_eth->max_indir_entries < 2)
		cr_skip_test("No RSS indirection entries available");

	for (i = 0; i < ARRAY_SIZE(ptes); i++) {
		rc = cxil_alloc_pte(lni, NULL, &pte_opts, &ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}

	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, ARRAY_SIZE(ptes), ptes,
					 RMU_ETH_HASH_TYPES);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_rmu_eth_add_mac_filter(rmu_eth, RMU_ETH_MAC, ptes[0], true);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_rmu_eth_add_all_mcast_filter(rmu_eth, ptes[0], true);
	cr_assert_eq(rc, -EPERM);

	rc = cxil_rmu_eth_add_promiscuous_filter(rmu_eth, ptes[1], true);
	cr_assert_eq(rc, -EPERM);

	rc = cxil_rmu_eth_remove_mac_filter(rmu_eth, RMU_ETH_MAC);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	rc = cxil_rmu_eth_set_rss_queues(rmu_eth, 0, NULL, 0);
	cr_assert_eq(rc, 0, "rc = (%d) %s", rc, strerror(-rc));

	for (i = 0; i < ARRAY_SIZE(ptes); i++) {
		rc = cxil_destroy_pte(ptes[i]);
		cr_assert_eq(rc, 0, "pte %u: rc = (%d) %s", i, rc,
			     strerror(-rc));
	}
}
