/* SPDX-License-Identifier: GPL-2.0-only or BSD-2-Clause */
/* Copyright 2026 Hewlett Packard Enterprise Development LP */

/*
 * Link-time stub implementation of libcxi.
 *
 * Built when configured with --enable-stub-only so container images and
 * cross-build environments can link against libcxi.so.1 with full ELF
 * symbol versioning (LIBCXI_1.0 .. LIBCXI_1.13) without installing
 * runtime daemon/utility dependencies, then bind-mount the host
 * libcxi.so.1 at runtime.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "libcxi.h"

__attribute__((noreturn)) static void cxil_stub_abort(const char *func)
{
	fprintf(stderr,
		"libcxi: link-time stub invoked (%s); bind-mount the host libcxi.so.1 at runtime\n",
		func);
	fflush(stderr);
	abort();
}

CXIL_API int cxil_get_device_list(struct cxil_device_list **dev_list)
{
	(void)dev_list;
	cxil_stub_abort(__func__);
}

CXIL_API void cxil_free_device_list(struct cxil_device_list *dev_list)
{
	(void)dev_list;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_open_device(uint32_t dev_id, struct cxil_dev **dev)
{
	(void)dev_id;
	(void)dev;
	cxil_stub_abort(__func__);
}

CXIL_API void cxil_close_device(struct cxil_dev *dev)
{
	(void)dev;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_svc(struct cxil_dev *dev, unsigned int svc_id,
			  struct cxi_svc_desc *svc_desc)
{
	(void)dev;
	(void)svc_id;
	(void)svc_desc;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_svc_list(struct cxil_dev *dev,
			       struct cxil_svc_list **svc_list)
{
	(void)dev;
	(void)svc_list;
	cxil_stub_abort(__func__);
}

CXIL_API void cxil_free_svc_list(struct cxil_svc_list *svc_list)
{
	(void)svc_list;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_svc_rsrc_use(struct cxil_dev *dev, unsigned int svc_id,
				   struct cxi_rsrc_use *rsrcs)
{
	(void)dev;
	(void)svc_id;
	(void)rsrcs;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_svc_rsrc_list(struct cxil_dev *dev,
				    struct cxil_svc_rsrc_list **rsrc_list)
{
	(void)dev;
	(void)rsrc_list;
	cxil_stub_abort(__func__);
}

CXIL_API void cxil_free_svc_rsrc_list(struct cxil_svc_rsrc_list *rsrc_list)
{
	(void)rsrc_list;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_update_svc(struct cxil_dev *dev,
			     const struct cxi_svc_desc *desc,
			     struct cxi_svc_fail_info *fail_info)
{
	(void)dev;
	(void)desc;
	(void)fail_info;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_svc(struct cxil_dev *dev,
			    const struct cxi_svc_desc *desc,
			    struct cxi_svc_fail_info *fail_info)
{
	(void)dev;
	(void)desc;
	(void)fail_info;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_parent_svc(struct cxil_dev *dev,
				   const struct cxi_svc_desc *desc,
				   struct cxi_svc_fail_info *fail_info)
{
	(void)dev;
	(void)desc;
	(void)fail_info;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_is_parent(struct cxil_dev *dev, unsigned int svc_id,
				bool *is_parent)
{
	(void)dev;
	(void)svc_id;
	(void)is_parent;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_svc(struct cxil_dev *dev, unsigned int svc_id)
{
	(void)dev;
	(void)svc_id;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_set_svc_lpr(struct cxil_dev *dev, unsigned int svc_id,
			      unsigned int lnis_per_rgid)
{
	(void)dev;
	(void)svc_id;
	(void)lnis_per_rgid;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_svc_lpr(struct cxil_dev *dev, unsigned int svc_id)
{
	(void)dev;
	(void)svc_id;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_enable(struct cxil_dev *dev, unsigned int svc_id,
			     bool enable)
{
	(void)dev;
	(void)svc_id;
	(void)enable;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_set_exclusive_cp(struct cxil_dev *dev,
				       unsigned int svc_id,
				       bool exclusive_cp)
{
	(void)dev;
	(void)svc_id;
	(void)exclusive_cp;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_get_exclusive_cp(struct cxil_dev *dev,
				       unsigned int svc_id, bool *exclusive_cp)
{
	(void)dev;
	(void)svc_id;
	(void)exclusive_cp;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_set_vni_range(struct cxil_dev *dev, unsigned int svc_id,
				    uint16_t vni_min, uint16_t vni_max)
{
	(void)dev;
	(void)svc_id;
	(void)vni_min;
	(void)vni_max;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_get_vni_range(struct cxil_dev *dev, unsigned int svc_id,
				    uint16_t *vni_min, uint16_t *vni_max)
{
	(void)dev;
	(void)svc_id;
	(void)vni_min;
	(void)vni_max;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_set_netns(struct cxil_dev *dev, unsigned int svc_id,
				unsigned int netns)
{
	(void)dev;
	(void)svc_id;
	(void)netns;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_svc_get_netns(struct cxil_dev *dev, unsigned int svc_id,
				unsigned int *netns)
{
	(void)dev;
	(void)svc_id;
	(void)netns;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_lni(struct cxil_dev *dev, struct cxil_lni **lni,
			    unsigned int svc_id)
{
	(void)dev;
	(void)lni;
	(void)svc_id;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_lni(struct cxil_lni *lni)
{
	(void)lni;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_cp(struct cxil_lni *lni, unsigned int vni,
			   enum cxi_traffic_class tc,
			   enum cxi_traffic_class_type tc_type,
			   struct cxi_cp **cp)
{
	(void)lni;
	(void)vni;
	(void)tc;
	(void)tc_type;
	(void)cp;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_trig_cp(struct cxil_lni *lni, unsigned int vni,
				enum cxi_traffic_class tc,
				enum cxi_traffic_class_type tc_type,
				enum cxi_trig_cp cp_type,
				struct cxi_cp **cp)
{
	(void)lni;
	(void)vni;
	(void)tc;
	(void)tc_type;
	(void)cp_type;
	(void)cp;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_cp(struct cxi_cp *cp)
{
	(void)cp;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_modify_cp(struct cxil_lni *lni, struct cxi_cp *cp,
			    unsigned int vni)
{
	(void)lni;
	(void)cp;
	(void)vni;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_reserve_domain(struct cxil_lni *lni, unsigned int vni,
				 unsigned int pid, unsigned int count)
{
	(void)lni;
	(void)vni;
	(void)pid;
	(void)count;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_domain(struct cxil_lni *lni, unsigned int vni,
			       unsigned int pid, struct cxil_domain **domain)
{
	(void)lni;
	(void)vni;
	(void)pid;
	(void)domain;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_domain(struct cxil_domain *domain)
{
	(void)domain;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_cmdq(struct cxil_lni *lni, struct cxi_eq *evtq,
			     const struct cxi_cq_alloc_opts *opts,
			     struct cxi_cq **cmdq)
{
	(void)lni;
	(void)evtq;
	(void)opts;
	(void)cmdq;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_buf_cmdq(struct cxil_lni *lni, struct cxi_eq *evtq,
				 const struct cxi_cq_alloc_opts_buf *opts,
				 struct cxi_cq **cmdq)
{
	(void)lni;
	(void)evtq;
	(void)opts;
	(void)cmdq;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_cmdq(struct cxi_cq *cmdq)
{
	(void)cmdq;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_cmdq_ack_counter(struct cxi_cq *cmdq,
				   unsigned int *ack_counter)
{
	(void)cmdq;
	(void)ack_counter;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_map(struct cxil_lni *lni, void *va, size_t len,
		      uint32_t flags, struct cxi_md_hints *hints,
		      struct cxi_md **md)
{
	(void)lni;
	(void)va;
	(void)len;
	(void)flags;
	(void)hints;
	(void)md;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_unmap(struct cxi_md *md)
{
	(void)md;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_update_md(struct cxi_md *md, void *va, size_t len,
			    uint32_t flags)
{
	(void)md;
	(void)va;
	(void)len;
	(void)flags;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_pte(struct cxil_lni *lni, struct cxi_eq *evtq,
			    struct cxi_pt_alloc_opts *opts,
			    struct cxil_pte **pte)
{
	(void)lni;
	(void)evtq;
	(void)opts;
	(void)pte;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_pte(struct cxil_pte *pte)
{
	(void)pte;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_map_pte(struct cxil_pte *pte, struct cxil_domain *domain,
			  unsigned int pid_offset, bool is_multicast,
			  struct cxil_pte_map **pte_map)
{
	(void)pte;
	(void)domain;
	(void)pid_offset;
	(void)is_multicast;
	(void)pte_map;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_unmap_pte(struct cxil_pte_map *pte_map)
{
	(void)pte_map;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_invalidate_pte_le(struct cxil_pte *pte,
				    unsigned int buffer_id,
				    enum c_ptl_list list)
{
	(void)pte;
	(void)buffer_id;
	(void)list;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_pte_status(struct cxil_pte *pte,
			     struct cxi_pte_status *status)
{
	(void)pte;
	(void)status;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_pte_transition_sm(struct cxil_pte *pte,
				    unsigned int drop_count)
{
	(void)pte;
	(void)drop_count;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_rmu_eth(struct cxil_dev *dev,
				const struct cxil_rmu_eth_opts *opts,
				struct cxil_rmu_eth **rmu_eth)
{
	(void)dev;
	(void)opts;
	(void)rmu_eth;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_rmu_eth(struct cxil_rmu_eth *rmu_eth)
{
	(void)rmu_eth;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_add_mac_filter(struct cxil_rmu_eth *rmu_eth,
					 uint64_t mac_addr,
					 struct cxil_pte *pte, bool use_rss)
{
	(void)rmu_eth;
	(void)mac_addr;
	(void)pte;
	(void)use_rss;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_add_all_mcast_filter(struct cxil_rmu_eth *rmu_eth,
					       struct cxil_pte *pte,
					       bool use_rss)
{
	(void)rmu_eth;
	(void)pte;
	(void)use_rss;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_add_promiscuous_filter(struct cxil_rmu_eth *rmu_eth,
						 struct cxil_pte *pte,
						 bool use_rss)
{
	(void)rmu_eth;
	(void)pte;
	(void)use_rss;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_remove_mac_filter(struct cxil_rmu_eth *rmu_eth,
					    uint64_t mac_addr)
{
	(void)rmu_eth;
	(void)mac_addr;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_remove_all_mcast_filter(struct cxil_rmu_eth *rmu_eth)
{
	(void)rmu_eth;
	cxil_stub_abort(__func__);
}

CXIL_API int
cxil_rmu_eth_remove_promiscuous_filter(struct cxil_rmu_eth *rmu_eth)
{
	(void)rmu_eth;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_set_rss_queues(struct cxil_rmu_eth *rmu_eth,
					 unsigned int num_queues,
					 struct cxil_pte **ptes,
					 uint32_t hash_types)
{
	(void)rmu_eth;
	(void)num_queues;
	(void)ptes;
	(void)hash_types;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_rmu_eth_set_indir_table(struct cxil_rmu_eth *rmu_eth,
					  const uint8_t *indir_table,
					  unsigned int indir_size)
{
	(void)rmu_eth;
	(void)indir_table;
	(void)indir_size;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_evtq(struct cxil_lni *lni, const struct cxi_md *md,
			     const struct cxi_eq_attr *attr,
			     struct cxil_wait_obj *event_wait,
			     struct cxil_wait_obj *status_wait,
			     struct cxi_eq **evtq)
{
	(void)lni;
	(void)md;
	(void)attr;
	(void)event_wait;
	(void)status_wait;
	(void)evtq;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_evtq(struct cxi_eq *evtq)
{
	(void)evtq;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_evtq_adjust_reserved_fc(struct cxi_eq *evtq, int value)
{
	(void)evtq;
	(void)value;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_evtq_resize(struct cxi_eq *evtq, void *queue,
			      size_t queue_len, struct cxi_md *queue_md)
{
	(void)evtq;
	(void)queue;
	(void)queue_len;
	(void)queue_md;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_evtq_resize_complete(struct cxi_eq *evtq)
{
	(void)evtq;
	cxil_stub_abort(__func__);
}

CXIL_API void cxil_clear_wait_obj(struct cxil_wait_obj *wait)
{
	(void)wait;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_wait_obj(struct cxil_lni *lni,
				 struct cxil_wait_obj **wait)
{
	(void)lni;
	(void)wait;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_wait_obj(struct cxil_wait_obj *wait)
{
	(void)wait;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_wait_obj_fd(struct cxil_wait_obj *wait)
{
	(void)wait;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_alloc_ct(struct cxil_lni *lni, struct c_ct_writeback *wb,
			   struct cxi_ct **ct)
{
	(void)lni;
	(void)wb;
	(void)ct;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_ct_wb_update(struct cxi_ct *ct, struct c_ct_writeback *wb)
{
	(void)ct;
	(void)wb;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_destroy_ct(struct cxi_ct *ct)
{
	(void)ct;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_map_csr(struct cxil_dev *dev)
{
	(void)dev;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_read_csr(struct cxil_dev *dev, unsigned int csr,
			   void *value, size_t csr_len)
{
	(void)dev;
	(void)csr;
	(void)value;
	(void)csr_len;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_write_csr(struct cxil_dev *dev, unsigned int csr,
			    const void *value, size_t csr_len)
{
	(void)dev;
	(void)csr;
	(void)value;
	(void)csr_len;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_write8_csr(struct cxil_dev *dev, unsigned int csr,
			     unsigned int offset, const void *value,
			     size_t csr_len)
{
	(void)dev;
	(void)csr;
	(void)offset;
	(void)value;
	(void)csr_len;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_read_cntr(struct cxil_dev *dev, unsigned int cntr,
			    uint64_t *value, struct timespec *ts)
{
	(void)dev;
	(void)cntr;
	(void)value;
	(void)ts;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_read_n_cntrs(struct cxil_dev *dev, unsigned int count,
			       const enum c_cntr_type *cntr, uint64_t *value,
			       struct timespec *ts)
{
	(void)dev;
	(void)count;
	(void)cntr;
	(void)value;
	(void)ts;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_read_all_cntrs(struct cxil_dev *dev, uint64_t *value,
				 struct timespec *ts)
{
	(void)dev;
	(void)value;
	(void)ts;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_inbound_wait(struct cxil_dev *dev)
{
	(void)dev;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_sbus_op(struct cxil_dev *dev,
			  const struct cxi_sbus_op_params *params,
			  uint32_t *rsp_data, uint8_t *result_code,
			  uint8_t *overrun)
{
	(void)dev;
	(void)params;
	(void)rsp_data;
	(void)result_code;
	(void)overrun;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_sbus_op_compat(struct cxil_dev *dev, int ring,
				 uint32_t req_data, uint8_t data_addr,
				 uint8_t rx_addr, uint8_t command,
				 uint32_t *rsp_data, uint8_t *result_code,
				 uint8_t *overrun, int timeout,
				 unsigned int flags)
{
	(void)dev;
	(void)ring;
	(void)req_data;
	(void)data_addr;
	(void)rx_addr;
	(void)command;
	(void)rsp_data;
	(void)result_code;
	(void)overrun;
	(void)timeout;
	(void)flags;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_sbus_op_reset(struct cxil_dev *dev)
{
	(void)dev;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_sbus_op_reset_compat(struct cxil_dev *dev, int ring)
{
	(void)dev;
	(void)ring;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_serdes_op(struct cxil_dev *dev, int port_num,
			    uint64_t serdes_sel, uint64_t op, uint64_t data,
			    uint16_t *result, int timeout, unsigned int flags)
{
	(void)dev;
	(void)port_num;
	(void)serdes_sel;
	(void)op;
	(void)data;
	(void)result;
	(void)timeout;
	(void)flags;
	cxil_stub_abort(__func__);
}

CXIL_API int cxil_get_amo_remap_to_pcie_fadd(struct cxil_dev *dev,
					     int *amo_remap_to_pcie_fadd)
{
	(void)dev;
	(void)amo_remap_to_pcie_fadd;
	cxil_stub_abort(__func__);
}

CXIL_API size_t cxil_page_size(void *base)
{
	(void)base;
	cxil_stub_abort(__func__);
}

CXIL_API bool cxil_is_copy_on_fork(void)
{
	cxil_stub_abort(__func__);
}
