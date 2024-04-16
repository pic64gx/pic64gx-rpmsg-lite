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
#include "mpfs_hal/mss_hal.h"

#ifdef REMOTEPROC
#include "remoteproc.h"
#endif

#if defined(RL_USE_ENVIRONMENT_CONTEXT) && (RL_USE_ENVIRONMENT_CONTEXT == 1)
#error "This RPMsg-Lite port requires RL_USE_ENVIRONMENT_CONTEXT set to 0"
#endif

static int32_t isr_counter     = 0;
static int32_t disable_counter = 0;
static void *platform_lock;
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
static LOCK_STATIC_CONTEXT platform_lock_static_ctxt;
#endif

enum miv_rp_mbox_messages {
	MIV_RP_MBOX_READY = 0xFFFFFF00,
	MIV_RP_MBOX_PENDING_MSG = 0xFFFFFF01,
    MIV_RP_MBOX_STOP = 0xFFFFFF02,
    MIV_RP_MBOX_END_MSG = 0xFFFFFF03,
};

static struct k_event msg_event;
static uint32_t rx_handler(uint32_t remote_hart_id, uint32_t * message, uint32_t message_size, bool is_ack, uint32_t *message_storage_ptr );
static void rpmsg_handler(bool is_ack, uint32_t msg);

/**
 *
 * CoreIHC Rx handler
 */
static uint32_t rx_handler( uint32_t remote_hart_id, uint32_t * message, uint32_t message_size, bool is_ack, uint32_t *message_storage_ptr )
{
    (void)remote_hart_id; /* message coming from here */

    if( is_ack == true )
    {
        rpmsg_handler(true, 0);
    }
    else
    {
        rpmsg_handler(false, (uint32_t) *message);
    }

    return(0U);
}

void rpmsg_handler(bool is_ack, uint32_t msg)
{
    if(is_ack)
    {
        k_event_set(&msg_event, 0x001);
        return;
    }

    switch(msg) {
        case MIV_RP_MBOX_STOP:
#ifdef REMOTEPROC
            rproc_stop();
#endif
            break;
        default:
            /* silently handle all other valid messages */
            if (msg >= MIV_RP_MBOX_READY && msg < MIV_RP_MBOX_END_MSG)
                return;
            env_isr((uint32_t) (msg));
    }
}

int32_t platform_init_interrupt(uint32_t vector_id, void *isr_data)
{
    uint64_t my_hart_id = read_csr(mhartid);

    uint32_t context_hart_id = 0u;
    /* Register ISR to environment layer */
    env_register_isr(vector_id, isr_data);

    env_lock_mutex(platform_lock);

    RL_ASSERT(0 <= isr_counter);
    if (isr_counter == 0)
    {
        switch (RL_GET_LINK_ID(vector_id))
        {
            case RL_PLATFORM_MIV_IHC_CONTEXT_A_B_LINK_ID:
                context_hart_id = IHC_context_to_context_hart_id(my_hart_id);
                switch(context_hart_id) {
                    case 1:
                        IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0),
                                    DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 0, priority),
                                    IHCIA_hart1_IRQHandler, NULL, 0);
                        break;
                    case 2:
                        IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1),
                                    DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 1, priority),
                                    IHCIA_hart2_IRQHandler, NULL, 0);
                        break;
                    case 3:
                        IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2),
                                    DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 2, priority),
                                    IHCIA_hart3_IRQHandler, NULL, 0);
                        break;
                    case 4:
                        IRQ_CONNECT(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3),
                                    DT_IRQ_BY_IDX(DT_NODELABEL(ihc), 3, priority),
                                    IHCIA_hart4_IRQHandler, NULL, 0);
                        break;
                    default:
                        /*  Unsupported configuration value*/
                        break;
                }
                break;
            default:
                break;
        }
    }

    isr_counter++;

    env_unlock_mutex(platform_lock);

    return 0;
}

int32_t platform_deinit_interrupt(uint32_t vector_id)
{
    uint32_t context_hart_id = 0u;
    uint64_t my_hart_id = read_csr(mhartid);

    /* Prepare the MU Hardware */
    env_lock_mutex(platform_lock);

    RL_ASSERT(0 < isr_counter);
    isr_counter--;

    if (isr_counter == 0)
    {
        switch (RL_GET_LINK_ID(vector_id))
        {
            case RL_PLATFORM_MIV_IHC_CONTEXT_A_B_LINK_ID:

                context_hart_id = IHC_context_to_context_hart_id(my_hart_id);

                switch(context_hart_id) {
                    case 1:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0));
                        break;
                    case 2:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1));
                        break;
                    case 3:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2));
                        break;
                    case 4:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3));
                        break;
                    default:
                        /*  Unsupported configuration value*/
                        break;
                }
                break;
            default:
                break;
        }
    }

    /* Unregister ISR from environment layer */
    env_unregister_isr(vector_id);

    env_unlock_mutex(platform_lock);

    return 0;
}

#ifdef REMOTEPROC
void platform_ready(void)
{
    uint32_t ihc_tx_message[IHC_MAX_MESSAGE_SIZE];
    ihc_tx_message[0] = MIV_RP_MBOX_READY;

    env_lock_mutex(platform_lock);
#ifdef IHC_CHANNEL_SIDE_A
    (void)IHC_tx_message_from_context(IHC_CHANNEL_TO_CONTEXTB, (uint32_t *) &ihc_tx_message);
#else
    (void)IHC_tx_message_from_context(IHC_CHANNEL_TO_CONTEXTA, (uint32_t *) &ihc_tx_message);
#endif

    k_event_wait(&msg_event, 0x001, true, K_FOREVER);
    env_unlock_mutex(platform_lock);
}
#endif

void platform_notify(uint32_t vector_id)
{
    // uint32_t tx_status;
    uint32_t ihc_tx_message[IHC_MAX_MESSAGE_SIZE];

    ihc_tx_message[0] = (uint32_t)(vector_id);

    switch (RL_GET_LINK_ID(vector_id))
    {
        case RL_PLATFORM_MIV_IHC_CONTEXT_A_B_LINK_ID:
            env_lock_mutex(platform_lock);
#ifdef IHC_CHANNEL_SIDE_A
            (void)IHC_tx_message_from_context(IHC_CHANNEL_TO_CONTEXTB,
                                              (uint32_t *) &ihc_tx_message);
#else
            (void)IHC_tx_message_from_context(IHC_CHANNEL_TO_CONTEXTA,
                                              (uint32_t *) &ihc_tx_message);
#endif
            k_event_wait(&msg_event, 0x001, true, K_FOREVER);
            env_unlock_mutex(platform_lock);
            return;

        default:
            return;
    }
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
    uint32_t context_hart_id = 0u;
    uint64_t my_hart_id = read_csr(mhartid);
    unsigned int irq_key;
    RL_ASSERT(0 < disable_counter);

    irq_key = irq_lock();
    disable_counter--;

    if (disable_counter == 0)
    {
        switch (RL_GET_LINK_ID(vector_id))
        {
            case RL_PLATFORM_MIV_IHC_CONTEXT_A_B_LINK_ID:

                context_hart_id = IHC_context_to_context_hart_id(my_hart_id);

                switch(context_hart_id) {
                    case 1:
                        irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0));
                        break;
                    case 2:
                        irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1));
                        break;
                    case 3:
                        irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2));
                        break;
                    case 4:
                        irq_enable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3));
                        break;
                    default:
                        /*  Unsupported configuration value*/
                        break;
                }
                break;
            default:
                break;
        }
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
    uint32_t context_hart_id = 0u;
    uint64_t my_hart_id = read_csr(mhartid);
    unsigned int irq_key;

    RL_ASSERT(0 <= disable_counter);

    irq_key = irq_lock();
    /* virtqueues use the same NVIC vector
       if counter is set - the interrupts are disabled */
    if (disable_counter == 0)
    {
        switch (RL_GET_LINK_ID(vector_id))
        {
            case RL_PLATFORM_MIV_IHC_CONTEXT_A_B_LINK_ID:
                context_hart_id = IHC_context_to_context_hart_id(my_hart_id);
                switch(context_hart_id) {
                    case 1:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 0));
                        break;
                    case 2:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 1));
                        break;
                    case 3:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 2));
                        break;
                    case 4:
                        irq_disable(DT_IRQN_BY_IDX(DT_NODELABEL(ihc), 3));
                        break;
                    default:
                        /*  Unsupported configuration value*/
                        break;
                }
                break;
            default:
                break;
        }
    }

    disable_counter++;
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
    uint64_t hartid = read_csr(mhartid);

    IHC_local_context_init((uint32_t)hartid);

    uint32_t remote_hart_id = IHC_partner_context_hart_id(hartid);

    IHC_local_remote_config((uint32_t)hartid, remote_hart_id, rx_handler, true, true);

    /* Create lock used in multi-instanced RPMsg */
#if defined(RL_USE_STATIC_API) && (RL_USE_STATIC_API == 1)
    if (0 != env_create_mutex(&platform_lock, 1, &platform_lock_static_ctxt))
#else
    if (0 != env_create_mutex(&platform_lock, 1))
#endif
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
