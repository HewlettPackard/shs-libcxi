/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause */
/* Copyright 2026 Hewlett Packard Enterprise Development LP */

/*
 * Legacy opcode test
 *
 * Why this exists
 * ---------------
 * libcxi talks to the kernel driver by writing small command structures
 * to /dev/cxi*. Each command starts with an opcode number. Users run
 * libcxi inside containers, so the libcxi in the container and the
 * driver on the host often come from different SHS releases, and both
 * directions have to work.
 *
 * It is easy to change an old public function so that it sends a newer
 * opcode without noticing. Older host drivers do not know the new opcode
 * and reject it, so containers with the newer libcxi fail on older
 * hosts. This test detects that automatically.
 *
 * The rule this test enforces
 * ---------------------------
 * A function exported under symbol versions LIBCXI_1.0 through 1.8 (the
 * API as of SHS 12.0) must keep sending the opcode and command struct it
 * sent in SHS 12.0. To use a newer driver feature, add a new function in
 * a new LIBCXI_1.N symbol version and leave the old one alone.
 *
 * Why SHS 12.0 is the baseline
 * ----------------------------
 * 12.0 is the oldest release containers are still built against, and
 * 12.0 and 13.0 share the same API (both end at LIBCXI_1.8). SHS 11.x
 * differs: three of these functions did not exist yet, and five others
 * were deliberately changed in 12.0 to stop sending commands, so an
 * 11.x baseline would need exceptions for shipped, intended changes.
 *
 * How it works
 * ------------
 * The test defines its own write(). Because the executable's symbols
 * take precedence over libc's, libcxi's device_write() ends up calling
 * it. A write to the fake device fd records the opcode and length, then
 * fails with ENODEV so the function under test returns before it tries
 * to mmap anything or read a response. All other fds go to the real
 * syscall. The object handles the functions need are fabricated from
 * libcxi_priv.h, so no device, VM or criterion is required.
 *
 * Output is TAP, consumed by automake's tap-driver through `make check`.
 *
 * Adding a function to this test
 * ------------------------------
 * Only functions in LIBCXI_1.0-1.8 belong here; that set is closed.
 * If one is somehow missing, add a call_<name>() wrapper and a T() row
 * with the opcode and struct it sent in SHS 12.0 (git show
 * origin/release/shs-12.0:src/libcxi.c).
 */

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "libcxi.h"
#include "libcxi_priv.h"

/* ---- write(2) interposer ------------------------------------------------ */

static int fake_fd = -1;
static bool captured;
static enum cxi_command_opcode cap_op;
static size_t cap_len;
static unsigned int cap_writes;

ssize_t write(int fd, const void *buf, size_t count)
{
	if (fd != fake_fd)
		return syscall(SYS_write, fd, buf, count);

	if (!captured) {
		cap_op = CXI_OP_INVALID;
		memcpy(&cap_op, buf,
		       count < sizeof(cap_op) ? count : sizeof(cap_op));
		cap_len = count;
		captured = true;
	}
	cap_writes++;

	errno = ENODEV;
	return -1;
}

/* ---- fabricated handles -------------------------------------------------- */

/* Heap-allocated: the destroy/unmap functions free() their argument. */
static struct cxil_dev_priv dev;
static struct cxil_lni_priv *lni_priv;
static struct cxil_domain_priv *dom_priv;
static struct cxil_cp_priv *cp_priv;
static struct cxil_cq *ccq;
static struct cxil_eq *eq;
static struct cxil_ct *cct;
static struct cxil_md_priv *md_priv;
static struct cxil_pte_priv *pte_priv;
static struct cxil_pte_map *pte_map;
static char buf[4096];

static void *xcalloc(size_t size)
{
	void *p = calloc(1, size);

	if (!p) {
		printf("Bail out! calloc: %s\n", strerror(errno));
		exit(1);
	}
	return p;
}

static void init_handles(void)
{
	dev.fd = fake_fd;
	lni_priv = xcalloc(sizeof(*lni_priv));
	lni_priv->dev = &dev;
	lni_priv->lni.id = 1;
	dom_priv = xcalloc(sizeof(*dom_priv));
	dom_priv->lni_priv = lni_priv;
	cp_priv = xcalloc(sizeof(*cp_priv));
	cp_priv->lni_priv = lni_priv;
	ccq = xcalloc(sizeof(*ccq));
	ccq->lni_priv = lni_priv;
	eq = xcalloc(sizeof(*eq));
	eq->lni_priv = lni_priv;
	cct = xcalloc(sizeof(*cct));
	cct->lni_priv = lni_priv;
	md_priv = xcalloc(sizeof(*md_priv));
	md_priv->lni_priv = lni_priv;
	pte_priv = xcalloc(sizeof(*pte_priv));
	pte_priv->lni_priv = lni_priv;
	pte_map = xcalloc(sizeof(*pte_map));
	pte_map->lni_priv = lni_priv;
}

/* ---- one zero-argument wrapper per function ---------------------------- */
/* (so the loop in main() can call every entry point the same way)         */

/* Device level */

static void call_cxil_alloc_lni(void)
{
	struct cxil_lni *out;

	cxil_alloc_lni(&dev.dev, &out, 1);
}

static void call_cxil_alloc_svc(void)
{
	struct cxi_svc_desc desc = {};
	struct cxi_svc_fail_info fi;

	cxil_alloc_svc(&dev.dev, &desc, &fi);
}

static void call_cxil_update_svc(void)
{
	struct cxi_svc_desc desc = {};
	struct cxi_svc_fail_info fi;

	cxil_update_svc(&dev.dev, &desc, &fi);
}

static void call_cxil_destroy_svc(void)
{
	cxil_destroy_svc(&dev.dev, 2);
}

static void call_cxil_get_svc(void)
{
	struct cxi_svc_desc desc;

	cxil_get_svc(&dev.dev, 1, &desc);
}

static void call_cxil_get_svc_list(void)
{
	struct cxil_svc_list *list;

	cxil_get_svc_list(&dev.dev, &list);
}

static void call_cxil_get_svc_lpr(void)
{
	cxil_get_svc_lpr(&dev.dev, 1);
}

static void call_cxil_set_svc_lpr(void)
{
	cxil_set_svc_lpr(&dev.dev, 1, 1);
}

static void call_cxil_get_svc_rsrc_list(void)
{
	struct cxil_svc_rsrc_list *list;

	cxil_get_svc_rsrc_list(&dev.dev, &list);
}

static void call_cxil_get_svc_rsrc_use(void)
{
	struct cxi_rsrc_use use;

	cxil_get_svc_rsrc_use(&dev.dev, 1, &use);
}

static void call_cxil_inbound_wait(void)
{
	cxil_inbound_wait(&dev.dev);
}

static void call_cxil_map_csr(void)
{
	cxil_map_csr(&dev.dev);
}

static void call_cxil_sbus_op(void)
{
	struct cxi_sbus_op_params p = {};
	uint32_t rsp;
	uint8_t rc, ov;

	cxil_sbus_op(&dev.dev, &p, &rsp, &rc, &ov);
}

static void call_cxil_sbus_op_compat(void)
{
	uint32_t rsp;
	uint8_t rc, ov;

	cxil_sbus_op_compat(&dev.dev, 0, 0, 0, 0, 0, &rsp, &rc, &ov, 0,
			    SBL_FLAG_DELAY_3US | SBL_FLAG_INTERVAL_1MS);
}

static void call_cxil_sbus_op_reset(void)
{
	cxil_sbus_op_reset(&dev.dev);
}

static void call_cxil_sbus_op_reset_compat(void)
{
	cxil_sbus_op_reset_compat(&dev.dev, 0);
}

static void call_cxil_serdes_op(void)
{
	uint16_t res;

	cxil_serdes_op(&dev.dev, 0, 0, 0, 0, &res, 0, 0);
}

/* LNI level */

static void call_cxil_destroy_lni(void)
{
	cxil_destroy_lni(&lni_priv->lni);
}

static void call_cxil_reserve_domain(void)
{
	cxil_reserve_domain(&lni_priv->lni, 1, 0, 1);
}

static void call_cxil_alloc_domain(void)
{
	struct cxil_domain *out;

	cxil_alloc_domain(&lni_priv->lni, 1, 0, &out);
}

static void call_cxil_destroy_domain(void)
{
	cxil_destroy_domain(&dom_priv->domain);
}

static void call_cxil_alloc_cp(void)
{
	struct cxi_cp *out;

	cxil_alloc_cp(&lni_priv->lni, 1, CXI_TC_BEST_EFFORT, CXI_TC_TYPE_DEFAULT,
		      &out);
}

static void call_cxil_destroy_cp(void)
{
	cxil_destroy_cp(&cp_priv->cp);
}

static void call_cxil_alloc_cmdq(void)
{
	struct cxi_cq_alloc_opts opts = { .count = 64 };
	struct cxi_cq *out;

	cxil_alloc_cmdq(&lni_priv->lni, NULL, &opts, &out);
}

static void call_cxil_destroy_cmdq(void)
{
	cxil_destroy_cmdq(&ccq->hw);
}

static void call_cxil_cmdq_ack_counter(void)
{
	unsigned int ack;

	cxil_cmdq_ack_counter(&ccq->hw, &ack);
}

static void call_cxil_alloc_evtq(void)
{
	struct cxi_eq_attr attr = {
		.flags = CXI_EQ_PASSTHROUGH,
		.queue = buf,
		.queue_len = sizeof(buf),
	};
	struct cxi_eq *out;

	cxil_alloc_evtq(&lni_priv->lni, NULL, &attr, NULL, NULL, &out);
}

static void call_cxil_destroy_evtq(void)
{
	cxil_destroy_evtq(&eq->hw);
}

static void call_cxil_evtq_adjust_reserved_fc(void)
{
	cxil_evtq_adjust_reserved_fc(&eq->hw, 1);
}

static void call_cxil_evtq_resize(void)
{
	cxil_evtq_resize(&eq->hw, buf, sizeof(buf), NULL);
}

static void call_cxil_evtq_resize_complete(void)
{
	cxil_evtq_resize_complete(&eq->hw);
}

static void call_cxil_alloc_ct(void)
{
	struct c_ct_writeback wb;
	struct cxi_ct *out;

	cxil_alloc_ct(&lni_priv->lni, &wb, &out);
}

static void call_cxil_destroy_ct(void)
{
	cxil_destroy_ct(&cct->ct);
}

static void call_cxil_ct_wb_update(void)
{
	struct c_ct_writeback wb;

	cxil_ct_wb_update(&cct->ct, &wb);
}

static void call_cxil_alloc_wait_obj(void)
{
	struct cxil_wait_obj *out;

	cxil_alloc_wait_obj(&lni_priv->lni, &out);
}

static void call_cxil_map(void)
{
	struct cxi_md *out;

	cxil_map(&lni_priv->lni, buf, sizeof(buf), 0, NULL, &out);
}

static void call_cxil_unmap(void)
{
	cxil_unmap(&md_priv->md);
}

static void call_cxil_update_md(void)
{
	cxil_update_md(&md_priv->md, buf, sizeof(buf), 0);
}

static void call_cxil_alloc_pte(void)
{
	struct cxi_pt_alloc_opts opts = {};
	struct cxil_pte *out;

	cxil_alloc_pte(&lni_priv->lni, NULL, &opts, &out);
}

static void call_cxil_destroy_pte(void)
{
	cxil_destroy_pte(&pte_priv->pte);
}

static void call_cxil_map_pte(void)
{
	struct cxil_pte_map *out;

	cxil_map_pte(&pte_priv->pte, &dom_priv->domain, 0, false, &out);
}

static void call_cxil_unmap_pte(void)
{
	cxil_unmap_pte(pte_map);
}

static void call_cxil_invalidate_pte_le(void)
{
	cxil_invalidate_pte_le(&pte_priv->pte, 0, C_PTL_LIST_PRIORITY);
}

static void call_cxil_pte_status(void)
{
	struct cxi_pte_status st;

	cxil_pte_status(&pte_priv->pte, &st);
}

static void call_cxil_pte_transition_sm(void)
{
	cxil_pte_transition_sm(&pte_priv->pte, 0);
}

/* ---- golden table: SHS 12.0 wire format for each legacy entry point ----- */

struct legacy_case {
	const char *name;
	enum cxi_command_opcode op;
	size_t len;
	void (*call)(void);
};

/* name, opcode and command struct it sent in SHS 12.0, wrapper to call */
#define T(fn, opcode, cmd) \
	{ #fn, opcode, sizeof(struct cmd), call_##fn }

static const struct legacy_case cases[] = {
	T(cxil_alloc_lni, CXI_OP_LNI_ALLOC,
	  cxi_lni_alloc_cmd),
	T(cxil_destroy_lni, CXI_OP_LNI_FREE,
	  cxi_lni_free_cmd),
	T(cxil_reserve_domain, CXI_OP_DOMAIN_RESERVE,
	  cxi_domain_reserve_cmd),
	T(cxil_alloc_domain, CXI_OP_DOMAIN_ALLOC,
	  cxi_domain_alloc_cmd),
	T(cxil_destroy_domain, CXI_OP_DOMAIN_FREE,
	  cxi_domain_free_cmd),
	T(cxil_alloc_cp, CXI_OP_CP_ALLOC,
	  cxi_cp_alloc_cmd),
	T(cxil_destroy_cp, CXI_OP_CP_FREE,
	  cxi_cp_free_cmd),
	T(cxil_alloc_cmdq, CXI_OP_CQ_ALLOC,
	  cxi_cq_alloc_cmd),
	T(cxil_destroy_cmdq, CXI_OP_CQ_FREE,
	  cxi_cq_free_cmd),
	T(cxil_cmdq_ack_counter, CXI_OP_CQ_ACK_COUNTER,
	  cxi_cq_ack_counter_cmd),
	T(cxil_alloc_evtq, CXI_OP_EQ_ALLOC,
	  cxi_eq_alloc_cmd),
	T(cxil_destroy_evtq, CXI_OP_EQ_FREE,
	  cxi_eq_free_cmd),
	T(cxil_evtq_adjust_reserved_fc, CXI_OP_EQ_ADJUST_RESERVED_FC,
	  cxi_eq_adjust_reserved_fc_cmd),
	T(cxil_evtq_resize, CXI_OP_EQ_RESIZE,
	  cxi_eq_resize_cmd),
	T(cxil_evtq_resize_complete, CXI_OP_EQ_RESIZE_COMPLETE,
	  cxi_eq_resize_complete_cmd),
	T(cxil_alloc_ct, CXI_OP_CT_ALLOC,
	  cxi_ct_alloc_cmd),
	T(cxil_destroy_ct, CXI_OP_CT_FREE,
	  cxi_ct_free_cmd),
	T(cxil_ct_wb_update, CXI_OP_CT_WB_UPDATE,
	  cxi_ct_wb_update_cmd),
	T(cxil_alloc_wait_obj, CXI_OP_WAIT_ALLOC,
	  cxi_wait_alloc_cmd),
	T(cxil_map, CXI_OP_ATU_MAP,
	  cxi_atu_map_cmd),
	T(cxil_unmap, CXI_OP_ATU_UNMAP,
	  cxi_atu_unmap_cmd),
	T(cxil_update_md, CXI_OP_ATU_UPDATE_MD,
	  cxi_atu_update_md_cmd),
	T(cxil_alloc_pte, CXI_OP_PTE_ALLOC,
	  cxi_pte_alloc_cmd),
	T(cxil_destroy_pte, CXI_OP_PTE_FREE,
	  cxi_pte_free_cmd),
	T(cxil_map_pte, CXI_OP_PTE_MAP,
	  cxi_pte_map_cmd),
	T(cxil_unmap_pte, CXI_OP_PTE_UNMAP,
	  cxi_pte_unmap_cmd),
	T(cxil_invalidate_pte_le, CXI_OP_PTE_LE_INVALIDATE,
	  cxi_pte_le_invalidate_cmd),
	T(cxil_pte_status, CXI_OP_PTE_STATUS,
	  cxi_pte_status_cmd),
	T(cxil_pte_transition_sm, CXI_OP_PTE_TRANSITION_SM,
	  cxi_pte_transition_sm_cmd),
	T(cxil_alloc_svc, CXI_OP_SVC_ALLOC,
	  cxi_svc_alloc_cmd),
	T(cxil_update_svc, CXI_OP_SVC_UPDATE,
	  cxi_svc_alloc_cmd),
	T(cxil_destroy_svc, CXI_OP_SVC_DESTROY,
	  cxi_svc_destroy_cmd),
	T(cxil_get_svc, CXI_OP_SVC_GET,
	  cxi_svc_get_cmd),
	T(cxil_get_svc_list, CXI_OP_SVC_LIST_GET,
	  cxi_svc_list_get_cmd),
	T(cxil_get_svc_lpr, CXI_OP_SVC_GET_LPR,
	  cxi_svc_lpr_cmd),
	T(cxil_set_svc_lpr, CXI_OP_SVC_SET_LPR,
	  cxi_svc_lpr_cmd),
	T(cxil_get_svc_rsrc_list, CXI_OP_SVC_RSRC_LIST_GET,
	  cxi_svc_rsrc_list_get_cmd),
	T(cxil_get_svc_rsrc_use, CXI_OP_SVC_RSRC_GET,
	  cxi_svc_rsrc_get_cmd),
	T(cxil_inbound_wait, CXI_OP_INBOUND_WAIT,
	  cxi_inbound_wait_cmd),
	T(cxil_map_csr, CXI_OP_MAP_CSRS,
	  cxi_map_csrs_cmd),
	T(cxil_sbus_op, CXI_OP_SBUS_OP,
	  cxi_sbus_op_cmd),
	T(cxil_sbus_op_compat, CXI_OP_SBUS_OP,
	  cxi_sbus_op_cmd),
	T(cxil_sbus_op_reset, CXI_OP_SBUS_OP_RESET,
	  cxi_sbus_op_reset_cmd),
	T(cxil_sbus_op_reset_compat, CXI_OP_SBUS_OP_RESET,
	  cxi_sbus_op_reset_cmd),
	T(cxil_serdes_op, CXI_OP_SERDES_OP,
	  cxi_serdes_op_cmd),
};

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define NCASES ARRAY_SIZE(cases)

/* Report one case in TAP form; returns true on failure. */
static bool report(size_t n, const struct legacy_case *c)
{
	bool fail = true;

	if (!captured)
		printf("not ok %zu - %s: no device write\n", n, c->name);
	else if (cap_op != c->op || cap_len != c->len)
		printf("not ok %zu - %s: op %d len %zu, want op %d len %zu\n",
		       n, c->name, cap_op, cap_len, c->op, c->len);
	else {
		printf("ok %zu - %s: op %d len %zu\n", n, c->name, c->op,
		       c->len);
		fail = false;
	}
	if (cap_writes > 1)
		printf("# %s issued %u writes; only the first is checked\n",
		       c->name, cap_writes);

	return fail;
}

int main(void)
{
	unsigned int failed = 0;
	size_t i;

	fake_fd = open("/dev/null", O_RDWR);
	if (fake_fd < 0) {
		printf("Bail out! open(/dev/null): %s\n", strerror(errno));
		return 1;
	}
	init_handles();

	printf("TAP version 13\n1..%zu\n", NCASES);

	for (i = 0; i < NCASES; i++) {
		captured = false;
		cap_writes = 0;
		cases[i].call();
		if (report(i + 1, &cases[i]))
			failed++;
	}

	if (failed) {
		printf("# %u function(s) changed the command they send.\n",
		       failed);
		printf("# Containers with this libcxi would fail on older\n");
		printf("# host drivers. Do not change what an old function\n");
		printf("# sends; add a new function in a new LIBCXI_1.N\n");
		printf("# symbol version instead.\n");
	}

	close(fake_fd);
	return failed ? 1 : 0;
}
