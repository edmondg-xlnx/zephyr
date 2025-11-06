/*
 * Copyright (c) 2022 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PMP_H_
#define PMP_H_

#ifdef CONFIG_CUSTOM_PMP_ENTRIES
struct custom_pmp_entries {
        unsigned long addr;
        unsigned long size;
        uint8_t flags;
};

struct custom_n_pmp_entries {
        unsigned int nentries;
        struct custom_pmp_entries *entries;
};
#endif

void z_riscv_pmp_init(void);
void z_riscv_pmp_stackguard_prepare(struct k_thread *thread);
void z_riscv_pmp_stackguard_enable(struct k_thread *thread);
void z_riscv_pmp_stackguard_disable(void);
void z_riscv_pmp_usermode_init(struct k_thread *thread);
void z_riscv_pmp_usermode_prepare(struct k_thread *thread);
void z_riscv_pmp_usermode_enable(struct k_thread *thread);

#endif /* PMP_H_ */
