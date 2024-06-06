/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include <stdio.h>
#include <string.h>
// #include <irq.h>
#include "rpmsg_platform.h"
#include "rpmsg_env.h"
// #include <zephyr/drivers/ipm.h>
#include <miv_ihc.h>
#include <zephyr/kernel.h>
#include "mss_extra_sw_config.h"
// #include "mpfs_hal/mss_hal.h"

#ifdef REMOTEPROC
#include "remoteproc.h"
#endif

#if defined(RL_USE_ENVIRONMENT_CONTEXT) && (RL_USE_ENVIRONMENT_CONTEXT == 1)
#error "This RPMsg-Lite port requires RL_USE_ENVIRONMENT_CONTEXT set to 0"
#endif

#ifndef IHC_COM_ID_MASTER
#define IHC_COM_ID_MASTER RL_PLATFORM_MIV_IHC_CH8_ID
#endif

#ifndef IHC_COM_ID_REMOTE
#define IHC_COM_ID_REMOTE RL_PLATFORM_MIV_IHC_CH21_ID
#endif

static int32_t isr_counter0 = 0;
static int32_t isr_counter1 = 0;
static int32_t isr_counter2 = 0;
static int32_t isr_counter3 = 0;
static int32_t isr_counter4 = 0;

static int32_t disable_counter0 = 0;
static int32_t disable_counter1 = 0;
static int32_t disable_counter2 = 0;
static int32_t disable_counter3 = 0;
static int32_t disable_counter4 = 0;

static void *platform_lock;

enum miv_rp_mbox_messages {
	MIV_RP_MBOX_READY = 0xFFFFFF00,
	MIV_RP_MBOX_PENDING_MSG = 0xFFFFFF01,
    MIV_RP_MBOX_STOP = 0xFFFFFF02,
    MIV_RP_MBOX_END_MSG = 0xFFFFFF03,
};

static struct k_event msg_event;

/**
 * Message Rx Handler
 */
static uint32_t QUEUE_IHC_MP_ISR_CALLBACK(uint8_t channel,
                           const uint32_t *message,
                           uint32_t message_size, uint32_t * ext_msg_ptr)
{
    uint32_t msg = message[0];

    switch(msg) {
        case MIV_RP_MBOX_STOP:
#ifdef REMOTEPROC
            rproc_stop(channel);
#endif
            break;
        default:
            if (msg >= MIV_RP_MBOX_READY && msg < MIV_RP_MBOX_END_MSG) {
                return 0;
            }
            env_isr((uint32_t)((msg) | (channel << 3)));
    }

    return 0;
}


static uint32_t QUEUE_IHC_MC_ISR_CALLBACK(uint8_t channel,
                           const uint32_t *message,
                           uint32_t message_size, uint32_t * ext_msg_ptr)
{

    k_event_set(&msg_event, 0x001);
    return 0;
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    /* Register ISR to environment layer */
    env_register_isr(vector_id, isr_data);

    env_lock_mutex(platform_lock);

    switch (RL_GET_COM_ID(vector_id))
    {
        case RL_PLATFORM_MIV_IHC_CH0_ID:
        case RL_PLATFORM_MIV_IHC_CH1_ID:
        case RL_PLATFORM_MIV_IHC_CH2_ID:
        case RL_PLATFORM_MIV_IHC_CH3_ID:
        case RL_PLATFORM_MIV_IHC_CH4_ID:
            RL_ASSERT(0 <= isr_counter0);
            if (isr_counter0 == 0)
            {
                IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0),
                            DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 0, priority),
                            IHC_APP_X_H0_IRQHandler, NULL, 0);
            }
            isr_counter0++;
        case RL_PLATFORM_MIV_IHC_CH5_ID:
        case RL_PLATFORM_MIV_IHC_CH6_ID:
        case RL_PLATFORM_MIV_IHC_CH7_ID:
        case RL_PLATFORM_MIV_IHC_CH8_ID:
        case RL_PLATFORM_MIV_IHC_CH9_ID:
            RL_ASSERT(0 <= isr_counter1);
            if (isr_counter1 == 0)
            {
               IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1),
                            DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 1, priority),
                            IHC_APP_X_H1_IRQHandler, NULL, 0);
            }
            isr_counter1++;
            break;
        case RL_PLATFORM_MIV_IHC_CH10_ID:
        case RL_PLATFORM_MIV_IHC_CH11_ID:
        case RL_PLATFORM_MIV_IHC_CH12_ID:
        case RL_PLATFORM_MIV_IHC_CH13_ID:
        case RL_PLATFORM_MIV_IHC_CH14_ID:
            RL_ASSERT(0 <= isr_counter2);
            if (isr_counter2 == 0)
            {
                IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2),
                            DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 2, priority),
                            IHC_APP_X_H2_IRQHandler, NULL, 0);
            }
            isr_counter2++;
        case RL_PLATFORM_MIV_IHC_CH15_ID:
        case RL_PLATFORM_MIV_IHC_CH16_ID:
        case RL_PLATFORM_MIV_IHC_CH17_ID:
        case RL_PLATFORM_MIV_IHC_CH18_ID:
        case RL_PLATFORM_MIV_IHC_CH19_ID:
            RL_ASSERT(0 <= isr_counter3);
            if (isr_counter3 == 0)
            {
               IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3),
                            DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 3, priority),
                            IHC_APP_X_H3_IRQHandler, NULL, 0);;
            }
            isr_counter3++;
            break;
        case RL_PLATFORM_MIV_IHC_CH20_ID:
        case RL_PLATFORM_MIV_IHC_CH21_ID:
        case RL_PLATFORM_MIV_IHC_CH22_ID:
        case RL_PLATFORM_MIV_IHC_CH23_ID:
        case RL_PLATFORM_MIV_IHC_CH24_ID:
            RL_ASSERT(0 <= isr_counter4);
            if (isr_counter4 == 0)
            {
               IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 4),
                            DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 4, priority),
                            IHC_APP_X_H4_IRQHandler, NULL, 0);
            }
            isr_counter4++;
            break;
        default:
            /* All the cases have been listed above, the default clause should not be reached. */
            break;
    }

    env_unlock_mutex(platform_lock);

    return 0;
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
    env_lock_mutex(platform_lock);

    switch (RL_GET_COM_ID(vector_id))
    {
        case RL_PLATFORM_MIV_IHC_CH0_ID:
        case RL_PLATFORM_MIV_IHC_CH1_ID:
        case RL_PLATFORM_MIV_IHC_CH2_ID:
        case RL_PLATFORM_MIV_IHC_CH3_ID:
        case RL_PLATFORM_MIV_IHC_CH4_ID:
            RL_ASSERT(0 <= isr_counter0);
            isr_counter0--;
            if (isr_counter0 == 0)
            {
                irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0));
            }
        case RL_PLATFORM_MIV_IHC_CH5_ID:
        case RL_PLATFORM_MIV_IHC_CH6_ID:
        case RL_PLATFORM_MIV_IHC_CH7_ID:
        case RL_PLATFORM_MIV_IHC_CH8_ID:
        case RL_PLATFORM_MIV_IHC_CH9_ID:
            RL_ASSERT(0 <= isr_counter1);
            isr_counter1--;
            if (isr_counter1 == 0)
            {
               irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1));
            }
            break;
        case RL_PLATFORM_MIV_IHC_CH10_ID:
        case RL_PLATFORM_MIV_IHC_CH11_ID:
        case RL_PLATFORM_MIV_IHC_CH12_ID:
        case RL_PLATFORM_MIV_IHC_CH13_ID:
        case RL_PLATFORM_MIV_IHC_CH14_ID:
            RL_ASSERT(0 <= isr_counter2);
            isr_counter2--;
            if (isr_counter2 == 0)
            {
                irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2));
            }
        case RL_PLATFORM_MIV_IHC_CH15_ID:
        case RL_PLATFORM_MIV_IHC_CH16_ID:
        case RL_PLATFORM_MIV_IHC_CH17_ID:
        case RL_PLATFORM_MIV_IHC_CH18_ID:
        case RL_PLATFORM_MIV_IHC_CH19_ID:
            RL_ASSERT(0 <= isr_counter3);
            isr_counter3--;
            if (isr_counter3 == 0)
            {
               irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3));
            }
            break;
        case RL_PLATFORM_MIV_IHC_CH20_ID:
        case RL_PLATFORM_MIV_IHC_CH21_ID:
        case RL_PLATFORM_MIV_IHC_CH22_ID:
        case RL_PLATFORM_MIV_IHC_CH23_ID:
        case RL_PLATFORM_MIV_IHC_CH24_ID:
            RL_ASSERT(0 <= isr_counter4);
            isr_counter4--;
            if (isr_counter4 == 0)
            {
               irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 4));
            }
            break;
        default:
            /* All the cases have been listed above, the default clause should not be reached. */
            break;
    }

    /* Unregister ISR from environment layer */
    env_unregister_isr(vector_id);

    env_unlock_mutex(platform_lock);

    return 0;
}

#ifdef REMOTEPROC
void platform_ready(uint32_t link_id)
{
	uint32_t msg = MIV_RP_MBOX_READY;

    env_lock_mutex(platform_lock);

	(void)IHC_tx_message(RL_GET_COM_ID_FROM_LINK_ID(link_id), (uint32_t *) &msg, sizeof(msg));
    k_event_wait(&msg_event, 0x001, true, K_FOREVER);
    env_unlock_mutex(platform_lock);
}
#endif

void platform_notify(uint32_t vector_id)
{
    /* Only vring id and queue id is needed in msg */
    uint32_t msg = RL_GEN_MU_MSG(vector_id);

	env_lock_mutex(platform_lock);

	IHC_tx_message(RL_GET_COM_ID(vector_id), (uint32_t *) &msg, sizeof(msg));
    k_event_wait(&msg_event, 0x0001, true, K_FOREVER);
    env_unlock_mutex(platform_lock);
    return;

}

/**
 * platform_in_isr
 *
 * Return whether CPU is processing IRQ
 *
 * @return True for IRQ, false otherwise.
 *
 */
int32_t platform_in_isr(void)
{
    return (0 != k_is_in_isr());
}

/**
 * platform_interrupt_enable
 *
 * Enable peripheral-related interrupt
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_enable(uint32_t vector_id)
{
    unsigned int irq_key;

    irq_key = irq_lock();

    switch (RL_GET_COM_ID(vector_id))
    {
        case RL_PLATFORM_MIV_IHC_CH0_ID:
        case RL_PLATFORM_MIV_IHC_CH1_ID:
        case RL_PLATFORM_MIV_IHC_CH2_ID:
        case RL_PLATFORM_MIV_IHC_CH3_ID:
        case RL_PLATFORM_MIV_IHC_CH4_ID:
            RL_ASSERT(0 <= disable_counter0);
            disable_counter0--;
            if (disable_counter0 == 0)
            {
                irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0));
            }
        case RL_PLATFORM_MIV_IHC_CH5_ID:
        case RL_PLATFORM_MIV_IHC_CH6_ID:
        case RL_PLATFORM_MIV_IHC_CH7_ID:
        case RL_PLATFORM_MIV_IHC_CH8_ID:
        case RL_PLATFORM_MIV_IHC_CH9_ID:
            RL_ASSERT(0 <= disable_counter1);
            disable_counter1--;
            if (disable_counter1 == 0)
            {
               irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1));
            }
            break;
        case RL_PLATFORM_MIV_IHC_CH10_ID:
        case RL_PLATFORM_MIV_IHC_CH11_ID:
        case RL_PLATFORM_MIV_IHC_CH12_ID:
        case RL_PLATFORM_MIV_IHC_CH13_ID:
        case RL_PLATFORM_MIV_IHC_CH14_ID:
            RL_ASSERT(0 <= disable_counter2);
            disable_counter2--;
            if (disable_counter2 == 0)
            {
                irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2));
            }
        case RL_PLATFORM_MIV_IHC_CH15_ID:
        case RL_PLATFORM_MIV_IHC_CH16_ID:
        case RL_PLATFORM_MIV_IHC_CH17_ID:
        case RL_PLATFORM_MIV_IHC_CH18_ID:
        case RL_PLATFORM_MIV_IHC_CH19_ID:
            RL_ASSERT(0 <= disable_counter3);
            disable_counter3--;
            if (disable_counter3 == 0)
            {
               irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3));
            }
            break;
        case RL_PLATFORM_MIV_IHC_CH20_ID:
        case RL_PLATFORM_MIV_IHC_CH21_ID:
        case RL_PLATFORM_MIV_IHC_CH22_ID:
        case RL_PLATFORM_MIV_IHC_CH23_ID:
        case RL_PLATFORM_MIV_IHC_CH24_ID:
            RL_ASSERT(0 <= disable_counter4);
            disable_counter4--;
            if (disable_counter4 == 0)
            {
               irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 4));
            }
            break;
        default:
            /* All the cases have been listed above, the default clause should not be reached. */
            break;
    }

    irq_unlock(irq_key);

    return ((int32_t)vector_id);
}

/**
 * platform_interrupt_disable
 *
 * Disable peripheral-related interrupt.
 *
 * @param vector_id Virtual vector ID that needs to be converted to IRQ number
 *
 * @return vector_id Return value is never checked.
 *
 */
int32_t platform_interrupt_disable(uint32_t vector_id)
{
    unsigned int irq_key;

    irq_key = irq_lock();
    switch (RL_GET_COM_ID(vector_id))
    {
        case RL_PLATFORM_MIV_IHC_CH0_ID:
        case RL_PLATFORM_MIV_IHC_CH1_ID:
        case RL_PLATFORM_MIV_IHC_CH2_ID:
        case RL_PLATFORM_MIV_IHC_CH3_ID:
        case RL_PLATFORM_MIV_IHC_CH4_ID:
            RL_ASSERT(0 <= disable_counter0);
            disable_counter0++;
            if (disable_counter0 == 0)
            {
                irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0));
            }
        case RL_PLATFORM_MIV_IHC_CH5_ID:
        case RL_PLATFORM_MIV_IHC_CH6_ID:
        case RL_PLATFORM_MIV_IHC_CH7_ID:
        case RL_PLATFORM_MIV_IHC_CH8_ID:
        case RL_PLATFORM_MIV_IHC_CH9_ID:
            RL_ASSERT(0 <= disable_counter1);
            disable_counter1++;
            if (disable_counter1 == 0)
            {
               irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1));
            }
            break;
        case RL_PLATFORM_MIV_IHC_CH10_ID:
        case RL_PLATFORM_MIV_IHC_CH11_ID:
        case RL_PLATFORM_MIV_IHC_CH12_ID:
        case RL_PLATFORM_MIV_IHC_CH13_ID:
        case RL_PLATFORM_MIV_IHC_CH14_ID:
            RL_ASSERT(0 <= disable_counter2);
            disable_counter2++;
            if (disable_counter2 == 0)
            {
                irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2));
            }
        case RL_PLATFORM_MIV_IHC_CH15_ID:
        case RL_PLATFORM_MIV_IHC_CH16_ID:
        case RL_PLATFORM_MIV_IHC_CH17_ID:
        case RL_PLATFORM_MIV_IHC_CH18_ID:
        case RL_PLATFORM_MIV_IHC_CH19_ID:
            RL_ASSERT(0 <= disable_counter3);
            disable_counter3++;
            if (disable_counter3 == 0)
            {
               irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3));
            }
            break;
        case RL_PLATFORM_MIV_IHC_CH20_ID:
        case RL_PLATFORM_MIV_IHC_CH21_ID:
        case RL_PLATFORM_MIV_IHC_CH22_ID:
        case RL_PLATFORM_MIV_IHC_CH23_ID:
        case RL_PLATFORM_MIV_IHC_CH24_ID:
            RL_ASSERT(0 <= disable_counter4);
            disable_counter4++;
            if (disable_counter4 == 0)
            {
               irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 4));
            }
            break;
        default:
            /* All the cases have been listed above, the default clause should not be reached. */
            break;
    }

    irq_unlock(irq_key);
    return ((int32_t)vector_id);
}

/**
 * platform_map_mem_region
 *
 * Dummy implementation
 *
 */
void platform_map_mem_region(uint32_t vrt_addr, uint32_t phy_addr, uint32_t size, uint32_t flags)
{
}

/**
 * platform_cache_all_flush_invalidate
 *
 * Dummy implementation
 *
 */
void platform_cache_all_flush_invalidate(void)
{
}

/**
 * platform_cache_disable
 *
 * Dummy implementation
 *
 */
void platform_cache_disable(void)
{
}

/**
 * platform_vatopa
 *
 * Dummy implementation
 *
 */
uintptr_t platform_vatopa(void *addr)
{
    return ((uintptr_t)(char *)addr);
}

/**
 * platform_patova
 *
 * Dummy implementation
 *
 */
void *platform_patova(uintptr_t addr)
{
    return ((void *)(char *)addr);
}

/**
 * platform_init
 *
 * platform/environment init
 */
int32_t platform_init(void)
{
#ifdef RPMSG_MASTER
    IHC_init(IHC_COM_ID_MASTER);
    IHC_config_mp_callback_handler(IHC_COM_ID_MASTER, QUEUE_IHC_MP_ISR_CALLBACK);
    IHC_config_mc_callback_handler(IHC_COM_ID_MASTER, QUEUE_IHC_MC_ISR_CALLBACK);
    IHC_enable_mp_interrupt(IHC_COM_ID_MASTER);
    IHC_enable_mc_interrupt(IHC_COM_ID_MASTER);
#else
    IHC_init(IHC_COM_ID_REMOTE);
    IHC_config_mp_callback_handler(IHC_COM_ID_REMOTE, QUEUE_IHC_MP_ISR_CALLBACK);
    IHC_config_mc_callback_handler(IHC_COM_ID_REMOTE, QUEUE_IHC_MC_ISR_CALLBACK);
    IHC_enable_mp_interrupt(IHC_COM_ID_REMOTE);
    IHC_enable_mc_interrupt(IHC_COM_ID_REMOTE); //ojo
#endif 

    /* Create lock used in multi-instanced RPMsg */
    if (0 != env_create_mutex(&platform_lock, 1))
    {
        return -1;
    }

    k_event_init(&msg_event);

    return 0;
}

/**
 * platform_deinit
 *
 * platform/environment deinit process
 */
int32_t platform_deinit(void)
{
    /* Delete lock used in multi-instanced RPMsg */
    env_delete_mutex(platform_lock);
    platform_lock = ((void *)0);
    return 0;
}

