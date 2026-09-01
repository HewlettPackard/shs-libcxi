/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause */
/* Copyright 2026 Hewlett Packard Enterprise Development LP */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "libcxi_test_common.h"
#include "libcxi.h"

static struct cxil_dev *pf_dev;
static struct cxil_dev *vf_dev;
static unsigned int pf_dev_id;
static unsigned int vf_idx;
static int pf_svc_id;
static int vf_svc_id;

static int set_vf_parent(int svc_id)
{
	char path[128];
	char value[16];
	ssize_t len;
	ssize_t rc;
	int fd;
	int saved_errno;

	snprintf(path, sizeof(path),
		 "/sys/class/cxi/cxi%u/vf/%u/svc_id", pf_dev_id, vf_idx);
	len = snprintf(value, sizeof(value), "%d\n", svc_id);

	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -errno;

	rc = write(fd, value, len);
	saved_errno = errno;
	close(fd);

	if (rc < 0)
		return -saved_errno;
	if (rc != len)
		return -EIO;

	return 0;
}

static void svc_vf_setup(void)
{
	const char *pf_dev_env = getenv("CXIL_TEST_PF_DEV");
	const char *vf_dev_env = getenv("CXIL_TEST_VF_DEV");
	const char *vf_idx_env = getenv("CXIL_TEST_VF_IDX");
	unsigned int vf_dev_id;
	int rc;

	if (!pf_dev_env || !vf_dev_env || !vf_idx_env)
		cr_skip("VF service-parent environment is not configured");

	pf_dev_id = strtoul(pf_dev_env, NULL, 0);
	vf_dev_id = strtoul(vf_dev_env, NULL, 0);
	vf_idx = strtoul(vf_idx_env, NULL, 0);

	rc = cxil_open_device(pf_dev_id, &pf_dev);
	cr_assert_eq(rc, 0, "cxil_open_device(PF %u) failed: %s",
		     pf_dev_id, strerror(-rc));
	cr_assert_not(pf_dev->info.is_vf, "cxi%u is not a PF", pf_dev_id);

	rc = cxil_open_device(vf_dev_id, &vf_dev);
	cr_assert_eq(rc, 0, "cxil_open_device(VF %u) failed: %s",
		     vf_dev_id, strerror(-rc));
	cr_assert(vf_dev->info.is_vf, "cxi%u is not a VF", vf_dev_id);
}

static void svc_vf_teardown(void)
{
	if (vf_svc_id > 0)
		cxil_destroy_svc(vf_dev, vf_svc_id);
	if (pf_dev)
		set_vf_parent(0);
	if (pf_svc_id > 0)
		cxil_destroy_svc(pf_dev, pf_svc_id);

	if (vf_dev)
		cxil_close_device(vf_dev);
	if (pf_dev)
		cxil_close_device(pf_dev);

	pf_dev = NULL;
	vf_dev = NULL;
	pf_svc_id = 0;
	vf_svc_id = 0;
}

TestSuite(svc_vf, .init = svc_vf_setup, .fini = svc_vf_teardown);

static void enable_and_assign_parent(void)
{
	int rc;

	rc = cxil_svc_enable(pf_dev, pf_svc_id, true);
	cr_assert_eq(rc, 0, "cxil_svc_enable(%d) failed: %d", pf_svc_id, rc);

	rc = set_vf_parent(pf_svc_id);
	cr_assert_eq(rc, 0, "assigning parent service %d failed: %d",
		     pf_svc_id, rc);
}

static void check_child_vni(unsigned int valid_vni, unsigned int invalid_vni)
{
	struct cxi_svc_desc child = {
		.resource_limits = true,
		.restricted_members = 1,
		.restricted_vnis = 1,
		.num_vld_vnis = 1,
	};
	int rc;

	child.vnis[0] = valid_vni;
	vf_svc_id = cxil_alloc_svc(vf_dev, &child, NULL);
	cr_assert_gt(vf_svc_id, 0,
		     "allocating child with VNI %u failed: %d",
		     valid_vni, vf_svc_id);

	rc = cxil_destroy_svc(vf_dev, vf_svc_id);
	cr_assert_eq(rc, 0, "destroying VF child service failed: %d", rc);
	vf_svc_id = 0;

	child.vnis[0] = invalid_vni;
	rc = cxil_alloc_svc(vf_dev, &child, NULL);
	if (rc > 0) {
		vf_svc_id = rc;
		cr_assert_fail("allocated child with disallowed VNI %u",
			       invalid_vni);
	}
	cr_assert_eq(rc, -EINVAL,
		     "child with disallowed VNI %u returned %d, expected %d",
		     invalid_vni, rc, -EINVAL);
}

Test(svc_vf, only_parent_services_can_be_assigned)
{
	struct cxi_svc_desc desc = {
		.resource_limits = true,
		.restricted_members = 1,
	};
	int rc;

	pf_svc_id = cxil_alloc_svc(pf_dev, &desc, NULL);
	cr_assert_gt(pf_svc_id, 0, "cxil_alloc_svc() failed: %d", pf_svc_id);

	rc = cxil_svc_enable(pf_dev, pf_svc_id, true);
	cr_assert_eq(rc, 0, "cxil_svc_enable(%d) failed: %d", pf_svc_id, rc);

	rc = set_vf_parent(pf_svc_id);
	cr_assert_eq(rc, -EINVAL,
		     "assigning non-parent service returned %d, expected %d",
		     rc, -EINVAL);
}

Test(svc_vf, parent_services_cannot_be_created_in_vf)
{
	struct cxi_svc_desc desc = {
		.resource_limits = true,
		.restricted_members = 1,
	};
	int rc;

	rc = cxil_alloc_parent_svc(vf_dev, &desc, NULL);
	cr_assert_eq(rc, -EPERM,
		     "VF parent allocation returned %d, expected %d", rc,
		     -EPERM);
}

Test(svc_vf, child_vnis_must_be_in_parent_list)
{
	struct cxi_svc_desc parent = {
		.resource_limits = true,
		.restricted_members = 1,
		.restricted_vnis = 1,
		.num_vld_vnis = 2,
		.vnis = { 100, 102 },
	};

	pf_svc_id = cxil_alloc_parent_svc(pf_dev, &parent, NULL);
	cr_assert_gt(pf_svc_id, 0, "parent allocation failed: %d", pf_svc_id);

	enable_and_assign_parent();
	check_child_vni(102, 101);
}

Test(svc_vf, child_vnis_must_be_in_parent_range)
{
	struct cxi_svc_desc parent = {
		.resource_limits = true,
		.restricted_members = 1,
	};
	int rc;

	pf_svc_id = cxil_alloc_parent_svc(pf_dev, &parent, NULL);
	cr_assert_gt(pf_svc_id, 0, "parent allocation failed: %d", pf_svc_id);

	rc = cxil_svc_set_vni_range(pf_dev, pf_svc_id, 200, 207);
	cr_assert_eq(rc, 0, "setting parent VNI range failed: %d", rc);

	enable_and_assign_parent();
	check_child_vni(205, 208);
}

Test(svc_vf, disabled_parent_cannot_be_assigned)
{
	struct cxi_svc_desc parent = {
		.resource_limits = true,
		.restricted_members = 1,
	};
	int rc;

	pf_svc_id = cxil_alloc_parent_svc(pf_dev, &parent, NULL);
	cr_assert_gt(pf_svc_id, 0, "parent allocation failed: %d", pf_svc_id);

	rc = cxil_svc_enable(pf_dev, pf_svc_id, false);
	cr_assert_eq(rc, 0, "disabling parent service failed: %d", rc);

	rc = set_vf_parent(pf_svc_id);
	cr_assert_eq(rc, -EKEYREVOKED,
		     "assigning disabled parent returned %d, expected %d",
		     rc, -EKEYREVOKED);
}

Test(svc_vf, parent_in_use_cannot_be_disabled_or_unassigned)
{
	struct cxi_svc_desc parent = {
		.resource_limits = true,
		.restricted_members = 1,
		.restricted_vnis = 1,
		.num_vld_vnis = 1,
		.vnis = { 300 },
	};
	struct cxi_svc_desc child = {
		.resource_limits = true,
		.restricted_members = 1,
		.restricted_vnis = 1,
		.num_vld_vnis = 1,
		.vnis = { 300 },
	};
	int rc;

	pf_svc_id = cxil_alloc_parent_svc(pf_dev, &parent, NULL);
	cr_assert_gt(pf_svc_id, 0, "parent allocation failed: %d", pf_svc_id);
	enable_and_assign_parent();

	vf_svc_id = cxil_alloc_svc(vf_dev, &child, NULL);
	cr_assert_gt(vf_svc_id, 0, "child allocation failed: %d", vf_svc_id);

	rc = cxil_svc_enable(pf_dev, pf_svc_id, false);
	cr_assert_eq(rc, -EBUSY,
		     "disabling in-use parent returned %d, expected %d",
		     rc, -EBUSY);

	rc = set_vf_parent(0);
	cr_assert_eq(rc, -EBUSY,
		     "unassigning in-use parent returned %d, expected %d",
		     rc, -EBUSY);
}
